#include <string.h>
#include <edutility.h>
#include <eduart.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

eduart::eduart(SerialPort uart_num):
	edthreaded_fd(),
	m_df(),
	m_baud(b115200)
{
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
}

bool eduart::start()
{
	set_fd(open(m_devpath.c_str(), O_RDWR | O_NOCTTY | O_SYNC));
	if (m_fd < 0)
	{
        log_message("Error opening " + m_devpath);
		return false;
	}
    _set_attribs();
	return edthreaded_fd::start();
}

eduart::~eduart()
{
	if (m_devpath == "/dev/ttyMFD2")
		_reattach_console();
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

int32_t eduart::_raw_read(uint8_t * buffer, uint32_t max_size)
{
    return ::read(m_fd, buffer, max_size);
}

int32_t eduart::_raw_write(uint8_t * buffer, uint32_t max_size)
{
	return ::write(m_fd, buffer, max_size);
}
