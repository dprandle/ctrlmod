#include <edi2c.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <edutility.h>

edi2c::edi2c(uint adapterNum):
	m_use_smbus(false),
	m_address(0)
{
	m_adapter_name = "/dev/i2c-" + std::to_string(adapterNum);
	pthread_mutex_init(&m_smbus_lock, NULL);
}

edi2c::~edi2c()
{
	pthread_mutex_destroy(&m_smbus_lock);
}

void edi2c::set_target_address(int addr)
{
	m_address = addr;
	if (m_fd != -1)
	{
		if (ioctl(m_fd, I2C_SLAVE, m_address) < 0)
		{
			m_err.err_val = Configuration;
			m_err._errno = errno;
		}
	}
}

int edi2c::target_address()
{
	return m_address;
}

void edi2c::enable_smbus(bool enable)
{
	pthread_mutex_lock(&m_smbus_lock);
	m_use_smbus = enable;
	pthread_mutex_unlock(&m_smbus_lock);	
}

bool edi2c::smbus_enabled()
{
	bool ret;
	pthread_mutex_lock(&m_smbus_lock);
	ret = m_use_smbus;
	pthread_mutex_unlock(&m_smbus_lock);
	return ret;
}

bool edi2c::start()
{
	if (m_fd != -1)
	{
		_setError(FDAlreadyOpen, 0);
		return false;
	}
	
	m_fd = open(m_adapter_name.c_str(),O_RDWR);
	if (m_fd < 0)
	{
		_setError(OpenFileDescriptor, errno);
		return false;
	}
	if (ioctl(m_fd, I2C_SLAVE, m_address) < 0)
	{
		_setError(Configuration, errno);
		return false;
	}
	return edthreaded_fd::start();
}

bool edi2c::commandRead(char reg, uint bytes_to_read)
{
	return (write(&reg, 1, bytes_to_read) == 1);
}

char edi2c::readByte(char reg)
{
	char ret = 0x00;
	uint cnt = 0;
	
	commandRead(reg, 1);
	while (cnt != 1)
	{
		cnt += read(&ret, 1);
		Error err = error();
		if (err.err_val != NoError)
		{
			log_message("edi2c::readByte error with retrieving byte: Error num " + std::to_string(err.err_val));
			break;
		}
	}
	return ret;
}

sint edi2c::readWord(char reg)
{
	sint ret = 0x0000;
	uint cnt = 0;
	
	commandRead(reg, 2);
	while (cnt != 2)
	{
		cnt = read((char*)&ret, 2);
		Error err = error();
		if (err.err_val != NoError)
		{
			log_message("edi2c::readByte error with retrieving byte: Error num " + std::to_string(err.err_val));
			break;
		}
	}
	return ret;
}

bool edi2c::command(char reg)
{
	return (write(&reg,1) == 1);
}

bool edi2c::writeByte(char reg, char byte)
{
	static char buf[2];
	buf[0] = reg;
	buf[1] = byte;
	return (write(buf,2) == 2);
}

bool edi2c::writeWord(char reg, sint word)
{
	static char buf[3];
	buf[0] = reg;
	buf[1] = char(word);
	buf[2] = char(word >> 8);
	return (write(buf, 3));
}

int edi2c::_raw_read(char * buffer, uint size)
{
	bool use_smbus = smbus_enabled();
	if (use_smbus)
	{
		
	}
	else
		return ::read(m_fd, buffer, size);
    return 0;
}

int edi2c::_raw_write(char * buffer, uint size)
{
	bool use_smbus = smbus_enabled();
	if (use_smbus)
	{
		
	}
	else
		return ::write(m_fd, buffer, size);
    return 0;
}
