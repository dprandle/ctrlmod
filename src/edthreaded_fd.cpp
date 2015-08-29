#include <edthreaded_fd.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <edutility.h>
#include <errno.h>
#include <edtimer.h>
#include <edcallback.h>

edthreaded_fd::edthreaded_fd(uint32_t readbuf_, uint32_t writebuf_):
	m_fd(-1),
	m_read_rawindex(0),
	m_read_curindex(0),
	m_write_rawindex(0),
	m_write_curindex(0),
	m_running(false),
	m_current_wait_for_byte_count(0),
	m_wait_timer(new edtimer())
{
	pthread_mutex_init(&m_send_lock, NULL);
	pthread_mutex_init(&m_recv_lock, NULL);
	pthread_mutex_init(&m_error_lock, NULL);
	pthread_mutex_init(&m_running_lock, NULL);
	m_read_buffer.resize(readbuf_, 0);
	m_write_buffer.resize(writebuf_);
	
	m_wait_timer->set_callback_delay(COMMAND_WAIT_DELAY);
	m_wait_timer->set_callback_mode(edtimer::single_shot);
	m_wait_timer->set_callback(new command_wait_callback(this));
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
	delete m_wait_timer;
	close(m_fd);
}

uint32_t edthreaded_fd::read(uint8_t * buffer, uint32_t max_size)
{
	uint32_t count = 0;
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

uint32_t edthreaded_fd::write(uint8_t * buffer, uint32_t size, int32_t response_size)
{
	int32_t resp = 0;
	pthread_mutex_lock(&m_send_lock);
	if (size > m_write_buffer.size())
		size = m_write_buffer.size();
    for (uint32_t i = 0; i < size; ++i)
	{
		if (i == size-1)
			resp = response_size;
		m_write_buffer[m_write_curindex] = WriteVal(buffer[i], resp);
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

int32_t edthreaded_fd::fd()
{
	return m_fd;
}

edthreaded_fd::Error edthreaded_fd::error()
{
	pthread_mutex_lock(&m_error_lock);
	Error ret = m_err;
	m_err._errno = 0;
	m_err.err_val = NoError;
	pthread_mutex_unlock(&m_error_lock);
	return ret;
}

void edthreaded_fd::_setError(ErrorVal err_val, int32_t _errno)
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
	{
		_setError(AlreadyRunning, 0);
		return false;
	}
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

bool edthreaded_fd::set_fd(int32_t fd_)
{
	if (running())
	{
		_setError(AlreadyRunning, 0);
		return false;
	}
	if (m_fd != -1)
		close(m_fd);
	m_fd = fd_;
	return true;
}

void edthreaded_fd::stop()
{
	pthread_mutex_lock(&m_running_lock);
	m_running = false;
	pthread_mutex_unlock(&m_running_lock);	
}


void edthreaded_fd::_do_read()
{
	static uint8_t buf[FD_TMP_BUFFER_SIZE];
	
	int32_t cnt = _raw_read(buf, FD_TMP_BUFFER_SIZE);
	if (cnt < 0)
	{
		int32_t err_no = errno;
		if (err_no != EAGAIN && err_no != EWOULDBLOCK)
		{
			_setError(InvalidRead, err_no);
			stop();
		}
	}
	pthread_mutex_lock(&m_recv_lock);
	for (int32_t i = 0; i < cnt; ++i)
	{
		m_read_buffer[m_read_rawindex] = buf[i];
		++m_read_rawindex;

		if (m_current_wait_for_byte_count > 0)
		{
			--m_current_wait_for_byte_count;
			cprint("Received 1 byte: " + std::to_string(m_current_wait_for_byte_count) + " remaining");
			if (m_current_wait_for_byte_count == 0 && m_wait_timer->running())
			{
				m_wait_timer->stop();
				cprint("Received all bytes for command - elapsed time: " + std::to_string(m_wait_timer->elapsed()) + " ms");
			}
		}
		
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
}

void edthreaded_fd::_do_write()
{
		static uint8_t buf[FD_TMP_BUFFER_SIZE];		
        int32_t tosend = 0;
        int32_t retval = 0;
        int32_t sent = 0;
		
        pthread_mutex_lock(&m_send_lock);
		while (m_write_rawindex != m_write_curindex)
		{
			WriteVal wv = m_write_buffer[m_write_rawindex];
			buf[tosend] = wv.byte;
			m_current_wait_for_byte_count = wv.response_size;
			++m_write_rawindex;
            ++tosend;

			if (m_write_rawindex == m_write_buffer.size())
				m_write_rawindex = 0;

			if (m_current_wait_for_byte_count > 0)
			{
				cprint("Command found: waiting for " + std::to_string(m_current_wait_for_byte_count) + " bytes");
				m_wait_timer->start();
				break;
			}

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
}

void edthreaded_fd::_exec()
{
	bool running = true;
    while (running)
    {
		m_wait_timer->update();

		if (!m_wait_timer->running())
			_do_write();
		_do_read();

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

