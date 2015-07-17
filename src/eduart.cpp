#include <string.h>
#include <edutility.h>
#include <eduart.h>
#include <fcntl.h>
#include <unistd.h>

eduart::eduart():
	m_fd(-1)
{}

eduart::~eduart()
{}

void eduart::init()
{
	m_fd = open(m_devpath.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
	if (m_fd < 0)
	{
        log_message("Error opening " + m_devpath);
        return;
	}
	_set_attribs();
}

uint eduart::read(char * buf, uint max_bytes)
{
	if (m_fd == -1)
		return 0;
	return ::read(m_fd, buf, max_bytes);
}

void eduart::write(char * buf, uint to_write)
{
	if (m_fd == -1)
		return;
	::write(m_fd, buf, to_write);
}
	
const std::string & eduart::device_path()
{
	return m_devpath;
}

void eduart::set_device_path(const std::string & path)
{
	m_devpath = path;
	if (m_fd == -1)
	{
		release();
		init();
	}
}

void eduart::set_baud(BaudRate baud)
{
	m_baud = baud;
	if (m_fd == -1)
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

void eduart::release()
{
	close(m_fd);
	m_fd = -1;
}

void eduart::_set_attribs()
{
	struct termios tty;
	memset(&tty, 0, sizeof(tty));
	if (tcgetattr(m_fd, &tty) != 0)
	{
		log_message("_set_attribs: Error setting termios");
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
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout


	if (tcsetattr(m_fd, TCSANOW, &tty) != 0)
		log_message("_set_attribs: Error - could not set tty");
}
