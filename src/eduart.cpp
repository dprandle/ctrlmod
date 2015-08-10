#include <string.h>
#include <edutility.h>
#include <eduart.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

eduart::eduart():
	m_fd(-1),
	m_df(),
	m_baud(b9600)
{
	pthread_mutex_init(&m_write_lock, NULL);
	pthread_mutex_init(&m_read_lock, NULL);
	pthread_mutex_init(&m_running_lock, NULL);
	zero_buf(m_write_buffer, UART_WRITE_BUF_MAX_SIZE);
	zero_buf(m_read_buffer, UART_READ_BUF_MAX_SIZE);
}

eduart::~eduart()
{
	if (running())
		stop();
	pthread_join(m_thread, NULL);
	pthread_mutex_destroy(&m_write_lock);
	pthread_mutex_destroy(&m_read_lock);
	pthread_mutex_destroy(&m_running_lock);
	close(m_fd);
	if (m_devpath == "/dev/ttyMFD2")
		_reattach_console();
	m_fd = -1;
}

uint eduart::read(char * buf, uint max_bytes)
{
	uint count = 0;
	pthread_mutex_lock(&m_read_lock);
	while (m_read_cur_index != m_read_raw_index)
	{
		buf[count] = m_read_buffer[m_read_cur_index];
		++count;
		++m_read_cur_index;
		if (m_read_cur_index == UART_READ_BUF_MAX_SIZE)
			m_read_cur_index = 0;
		if (count == max_bytes)
			break;
	}
	pthread_mutex_unlock(&m_read_lock);
	return count;
}

uint eduart::write(char * buf, uint to_write)
{
	if (to_write > UART_WRITE_BUF_MAX_SIZE)
		to_write = UART_WRITE_BUF_MAX_SIZE;
	
	pthread_mutex_lock(&m_write_lock);
	for (uint i = 0; i < to_write; ++i)
	{
		m_write_buffer[m_write_cur_index] = buf[i];
		++m_write_cur_index;
		if (m_write_cur_index == UART_WRITE_BUF_MAX_SIZE)
			m_write_cur_index = 0;
		if (m_write_cur_index == m_write_raw_index)
		{
			std::cout << "Write buffer overflow" << std::endl;
			break;
		}
	}
	pthread_mutex_unlock(&m_write_lock);
	return to_write;
}
	
const std::string & eduart::device_path()
{
	return m_devpath;
}

void eduart::set_baud(BaudRate baud)
{
	m_baud = baud;
	if (m_fd != -1)
		_set_attribs();
}

eduart::BaudRate eduart::baud()
{
	return m_baud;
}

void eduart::set_format(DataBits db, Parity p, StopBits sb)
{
	m_df.db = db;
	m_df.p = p;
	m_df.sb = sb;

	if (m_fd != -1)
		_set_attribs();
}

void eduart::set_format(const DataFormat & data_format)
{
	m_df = data_format;
	if (m_fd != -1)
		_set_attribs();	
}

const eduart::DataFormat & eduart::format()
{
	return m_df;
}

void eduart::_set_attribs()
{
	struct termios tty;
	memset(&tty, 0, sizeof(tty));
	if (tcgetattr(m_fd, &tty) != 0)
	{
		if (errno == EBADF)
			log_message("_set_attribs: Error getting termios - m_fd is not valid file descriptor");
		else if (errno == ENOTTY)
			log_message("_set_attribs: Error getting termios - the file associated with fildes is not a terminal");
		return;
	}
	
	tty.c_iflag = 0;
	tty.c_lflag = 0;
	tty.c_oflag = 0;
	tty.c_cflag = 0;
	
	tty.c_cflag |= (CLOCAL | CREAD | m_df.db | m_df.p | m_df.sb);

	// Set baud rates
	cfsetospeed (&tty, m_baud);
	cfsetispeed (&tty, m_baud);

	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 1;            // 0.1 seconds read timeout


	if (tcsetattr(m_fd, TCSANOW, &tty) != 0)
		log_message("_set_attribs: Error - could not set tty");
}


void eduart::_detach_console()
{
	// Need to write these as soon as intel tells me how
}

void eduart::_reattach_console()
{
	// Same thing here
}

bool eduart::start(SerialPort uart_num)
{
	pthread_mutex_lock(&m_running_lock);
	if (m_running)
		return false;
	pthread_mutex_unlock(&m_running_lock);

	switch(uart_num)
	{
	  case (Uart1):
		  m_devpath = "/dev/ttyMFD1";
		  break;
	  case (Uart2):
		  m_devpath = "/dev/ttyMFD2";
		  _detach_console();
		  break;
	}
	
	m_fd = open(m_devpath.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
	if (m_fd < 0)
	{
        log_message("Error opening " + m_devpath);
        return false;
	}
	_set_attribs();
	
	// Thread not running - no need for mutex anymore
	m_running = true;
	if (pthread_create(&m_thread, NULL, eduart::thread_exec, (void*)this) != 0)
	{
		m_running = false;
		close(m_fd);
		m_fd = -1;
		return false;
	}
	return true;	
}

void eduart::stop()
{
	pthread_mutex_lock(&m_running_lock);
	m_running = false;
	pthread_mutex_unlock(&m_running_lock);	
}

bool eduart::running()
{
	pthread_mutex_lock(&m_running_lock);
	bool ret = m_running;
	pthread_mutex_unlock(&m_running_lock);	
	return ret;		
}

void eduart::_exec()
{
	// Do blocking calls to read and write here - first read then write
    static char buf[UART_TMP_BUF_SIZE];
	bool running = true;
    while (running)
    {		
        int cnt = ::read(m_fd, buf, UART_TMP_BUF_SIZE);
        if (cnt < 0)
        {
            int err_no = errno;
			std::cout << err_no;
        }
		pthread_mutex_lock(&m_read_lock);
		for (int i = 0; i < cnt; ++i)
		{
			m_read_buffer[m_read_raw_index] = buf[i];
			++m_read_raw_index;

			if (m_read_raw_index == UART_READ_BUF_MAX_SIZE)
				m_read_raw_index = 0;

			// This check may go away eventually
			if (m_read_raw_index == m_read_cur_index)
			{
				std::cout << "Read buffer overflow";
			}
		}
		pthread_mutex_unlock(&m_read_lock);

		
		int tosend = 0;
        int retval = 0;
        int sent = 0;		
        pthread_mutex_lock(&m_write_lock);
		while (m_write_raw_index != m_write_cur_index)
		{
			buf[tosend] = m_write_buffer[m_write_raw_index];
			++m_write_raw_index;
            ++tosend;
			if (m_write_raw_index == UART_WRITE_BUF_MAX_SIZE)
				m_write_raw_index = 0;
			if (tosend == UART_TMP_BUF_SIZE)
				break;
		}
        pthread_mutex_unlock(&m_write_lock);

        while (sent != tosend)
        {
            retval = ::write(m_fd, buf+sent, tosend-sent);
            if (retval == -1)
            {
				std::cout << "Write error" << std::endl;
            }
            sent += retval;
        }
		
		pthread_mutex_lock(&m_running_lock);
		running = m_running;
		pthread_mutex_unlock(&m_running_lock);
	}
    pthread_exit(NULL);
}

void * eduart::thread_exec(void * e_uart)
{
	static_cast<eduart*>(e_uart)->_exec();
	return NULL;
}

