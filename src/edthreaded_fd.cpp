#include <edthreaded_fd.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <edutility.h>
#include <errno.h>

edthreaded_fd::edthreaded_fd(uint readbuf_, uint writebuf_):
	m_fd(-1),
	m_read_rawindex(0),
	m_read_curindex(0),
	m_write_rawindex(0),
	m_write_curindex(0),
	m_running(false)
{
	pthread_mutex_init(&m_send_lock, NULL);
	pthread_mutex_init(&m_recv_lock, NULL);
	pthread_mutex_init(&m_error_lock, NULL);
	pthread_mutex_init(&m_running_lock, NULL);
	m_read_buffer.resize(readbuf_, 0);
	m_write_buffer.resize(writebuf_, 0);
}
	
edthreaded_fd::~edthreaded_fd()
{
	if (running())
		stop();
	pthread_join(m_thread, NULL);
	pthread_mutex_destroy(&m_send_lock);
	pthread_mutex_destroy(&m_recv_lock);
	pthread_mutex_destroy(&m_error_lock);
	pthread_mutex_destroy(&m_running_lock);
	close(m_fd);
}

uint edthreaded_fd::read(char * buffer, uint max_size)
{
	uint count = 0;
	pthread_mutex_lock(&m_recv_lock);
	while (m_read_curindex != m_read_rawindex)
	{
		buffer[count] = m_read_buffer[m_read_curindex];
		++count;
		++m_read_curindex;
		if (m_read_curindex == m_read_buffer.size())
			m_read_curindex = 0;
		if (count == max_size)
			break;
	}
	pthread_mutex_unlock(&m_recv_lock);
	return count;
}

uint edthreaded_fd::write(char * buffer, uint size)
{
	pthread_mutex_lock(&m_send_lock);
	if (size > m_write_buffer.size())
		size = m_write_buffer.size();
    for (uint i = 0; i < size; ++i)
	{
		m_write_buffer[m_write_curindex] = buffer[i];
		++m_write_curindex;
		if (m_write_curindex == m_write_rawindex)
			size = i;
		if (m_write_curindex == m_write_buffer.size())
			m_write_curindex = 0;
	}
	pthread_mutex_unlock(&m_send_lock);
	return size;
}

bool edthreaded_fd::running()
{
	pthread_mutex_lock(&m_running_lock);
	bool ret = m_running;
	pthread_mutex_unlock(&m_running_lock);	
	return ret;
}

int edthreaded_fd::fd()
{
	return m_fd;
}

edthreaded_fd::Error edthreaded_fd::error()
{
	pthread_mutex_lock(&m_error_lock);
	Error ret = m_err;
	pthread_mutex_unlock(&m_error_lock);
	return ret;
}

void edthreaded_fd::_setError(ErrorVal err_val, int _errno)
{
	pthread_mutex_lock(&m_error_lock);
	m_err.err_val = err_val;
	m_err._errno = _errno;
	pthread_mutex_unlock(&m_error_lock);
}

bool edthreaded_fd::start()
{
	// lock mutex in case thread is already running
	pthread_mutex_lock(&m_running_lock);
	if (m_running)
		return false;
	pthread_mutex_unlock(&m_running_lock);

	// Thread not running - no need for mutex anymore
	m_running = true;
	if (pthread_create(&m_thread, NULL, edthreaded_fd::thread_exec, (void*)this) != 0)
	{
		_setError(ThreadCreation, errno);
		m_running = false;
		return false;
	}
	return true;
	// thread created
}

void edthreaded_fd::set_fd(int fd_)
{
	if (m_fd != -1)
		close(m_fd);
	m_fd = fd_;
}

void edthreaded_fd::stop()
{
	pthread_mutex_lock(&m_running_lock);
	m_running = false;
	pthread_mutex_unlock(&m_running_lock);	
}

void edthreaded_fd::_exec()
{
    // Do blocking calls to read and write here - first read then write
    static char buf[FD_TMP_BUFFER_SIZE];
	bool running = true;
	
    while (running)
    {
        int cnt = _raw_read(buf, FD_TMP_BUFFER_SIZE);
        if (cnt < 0)
        {
            int err_no = errno;
            if (err_no != EAGAIN && err_no != EWOULDBLOCK)
            {
                _setError(InvalidRead, err_no);
                stop();
            }
        }
		pthread_mutex_lock(&m_recv_lock);
		for (int i = 0; i < cnt; ++i)
		{
			m_read_buffer[m_read_rawindex] = buf[i];
			++m_read_rawindex;

			if (m_read_rawindex == m_read_buffer.size())
				m_read_rawindex = 0;

			// This check may go away eventually
			if (m_read_rawindex == m_read_curindex)
			{
				_setError(InvalidRead, 0);
				stop();
			}
		}
		pthread_mutex_unlock(&m_recv_lock);

        int tosend = 0;
        int retval = 0;
        int sent = 0;		
        pthread_mutex_lock(&m_send_lock);
		while (m_write_rawindex != m_write_curindex)
		{
			buf[tosend] = m_write_buffer[m_write_rawindex];
			++m_write_rawindex;
            ++tosend;
			if (m_write_rawindex == m_write_buffer.size())
				m_write_rawindex = 0;
			if (tosend == FD_TMP_BUFFER_SIZE)
				break;
		}
        pthread_mutex_unlock(&m_send_lock);

        while (sent != tosend)
        {
            retval = _raw_write(buf+sent, tosend-sent);
            if (retval == -1)
            {
                _setError(InvalidWrite, errno);
                stop();
            }
            sent += retval;
        }
		
		pthread_mutex_lock(&m_running_lock);
		running = m_running;
		pthread_mutex_unlock(&m_running_lock);
    }
    pthread_exit(NULL);
}

void * edthreaded_fd::thread_exec(void * _this)
{
	static_cast<edthreaded_fd*>(_this)->_exec();
	return NULL;
}

