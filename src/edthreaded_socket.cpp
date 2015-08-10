#include <edthreaded_socket.h>

#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <edutility.h>
#include <errno.h>

edthreaded_socket::edthreaded_socket(uint socket_fd):
	m_sckt_fd(socket_fd),
	m_recv_rawindex(0),
	m_recv_curindex(0),
	m_send_rawindex(0),
	m_send_curindex(0),
	m_running(false)
{
	pthread_mutex_init(&m_send_lock, NULL);
	pthread_mutex_init(&m_recv_lock, NULL);
	pthread_mutex_init(&m_error_lock, NULL);
	pthread_mutex_init(&m_running_lock, NULL);
	zero_buf(m_send_buffer, SCKT_SEND_BUF_SIZE);
	zero_buf(m_recv_buffer, SCKT_RECV_BUF_SIZE);
	start();
}
	
edthreaded_socket::~edthreaded_socket()
{
	if (running())
		stop();
	pthread_join(m_thread, NULL);
	pthread_mutex_destroy(&m_send_lock);
	pthread_mutex_destroy(&m_recv_lock);
	pthread_mutex_destroy(&m_error_lock);
	pthread_mutex_destroy(&m_running_lock);
	close(m_sckt_fd);
}

uint edthreaded_socket::read(char * buffer, uint max_size)
{
	uint count = 0;
	pthread_mutex_lock(&m_recv_lock);
	while (m_recv_curindex != m_recv_rawindex)
	{
		buffer[count] = m_recv_buffer[m_recv_curindex];
		++count;
		++m_recv_curindex;
		if (m_recv_curindex == SCKT_RECV_BUF_SIZE)
			m_recv_curindex = 0;
		if (count == max_size)
			break;
	}
	pthread_mutex_unlock(&m_recv_lock);
	return count;
}

uint edthreaded_socket::write(char * buffer, uint size)
{
	if (size > SCKT_SEND_BUF_SIZE)
		size = SCKT_SEND_BUF_SIZE;
	
	pthread_mutex_lock(&m_send_lock);
    for (uint i = 0; i < size; ++i)
	{
		m_send_buffer[m_send_curindex] = buffer[i];
		++m_send_curindex;
		if (m_send_curindex == m_send_rawindex)
			size = i;
		if (m_send_curindex == SCKT_SEND_BUF_SIZE)
			m_send_curindex = 0;
	}
	pthread_mutex_unlock(&m_send_lock);
	return size;
}

bool edthreaded_socket::running()
{
	pthread_mutex_lock(&m_running_lock);
	bool ret = m_running;
	pthread_mutex_unlock(&m_running_lock);	
	return ret;
}

int edthreaded_socket::fd()
{
	return m_sckt_fd;
}

edthreaded_socket::Error edthreaded_socket::error()
{
	pthread_mutex_lock(&m_error_lock);
	Error ret = m_err;
	pthread_mutex_unlock(&m_error_lock);
	return ret;
}

void edthreaded_socket::_setError(ErrorVal err_val, int _errno)
{
	pthread_mutex_lock(&m_error_lock);
	m_err.err_val = err_val;
	m_err._errno = _errno;
	pthread_mutex_unlock(&m_error_lock);
}

bool edthreaded_socket::start()
{
	// lock mutex in case thread is already running
	pthread_mutex_lock(&m_running_lock);
	if (m_running)
		return false;
	pthread_mutex_unlock(&m_running_lock);

	// Thread not running - no need for mutex anymore
	m_running = true;
	if (pthread_create(&m_thread, NULL, edthreaded_socket::thread_exec, (void*)this) != 0)
	{
		_setError(ThreadCreation, errno);
		m_running = false;
		return false;
	}
	return true;
	// thread created
}

void edthreaded_socket::stop()
{
	pthread_mutex_lock(&m_running_lock);
	m_running = false;
	pthread_mutex_unlock(&m_running_lock);	
}

void edthreaded_socket::_exec()
{
    // Do blocking calls to read and write here - first read then write
    static char buf[SCKT_TMP_BUF_SIZE];
	bool running = true;
	
    while (running)
    {
        int cnt = ::recv(m_sckt_fd, buf, SCKT_TMP_BUF_SIZE, MSG_DONTWAIT);
        if (cnt < 0)
        {
            int err_no = errno;
            if (err_no != EAGAIN && err_no != EWOULDBLOCK)
            {
                _setError(InvalidRead, err_no);
                stop();
            }
        }
        if (cnt == 0)
        {
            int err_no = errno;
            _setError(ConnectionClosed, err_no);
            stop();
        }
		pthread_mutex_lock(&m_recv_lock);
		for (int i = 0; i < cnt; ++i)
		{
			m_recv_buffer[m_recv_rawindex] = buf[i];
			++m_recv_rawindex;

			if (m_recv_rawindex == SCKT_RECV_BUF_SIZE)
				m_recv_rawindex = 0;

			// This check may go away eventually
			if (m_recv_rawindex == m_recv_curindex)
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
		while (m_send_rawindex != m_send_curindex)
		{
			buf[tosend] = m_send_buffer[m_send_rawindex];
			++m_send_rawindex;
            ++tosend;
			if (m_send_rawindex == SCKT_SEND_BUF_SIZE)
				m_send_rawindex = 0;
			if (tosend == SCKT_TMP_BUF_SIZE)
				break;
		}
        pthread_mutex_unlock(&m_send_lock);

        while (sent != tosend)
        {
            retval = ::write(m_sckt_fd, buf+sent, tosend-sent);
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

void * edthreaded_socket::thread_exec(void * _this)
{
	static_cast<edthreaded_socket*>(_this)->_exec();
	return NULL;
}
