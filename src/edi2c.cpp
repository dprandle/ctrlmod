#include <edi2c.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <edutility.h>

edi2c::edi2c(uint32_t adapterNum):
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

void edi2c::set_target_address(int32_t addr)
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

int32_t edi2c::target_address()
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

void edi2c::readBytes(uint8_t reg, uint8_t * buffer, uint32_t size)
{
	uint32_t cnt = 0;
	
	commandRead(reg, size);
	while (cnt != size)
	{
		cnt += read(buffer+cnt, size - cnt);
		Error err = error();
		if (err.err_val != NoError)
		{
			log_message("edi2c::readByte error with retrieving byte: Error num " + std::to_string(err.err_val));
			break;
		}
	}
}

bool edi2c::commandRead(uint8_t reg, uint32_t bytes_to_read)
{
	return (write(&reg, 1, bytes_to_read) == 1);
}

uint8_t edi2c::readByte(uint8_t reg)
{
	uint8_t ret = 0x00;
	readBytes(reg, (uint8_t*)&ret, 1);
	return ret;
}

int16_t edi2c::readWord(uint8_t reg)
{
	int16_t ret = 0x0000;
	readBytes(reg, (uint8_t*)&ret, 2);
	return ret;
}

bool edi2c::command(uint8_t reg)
{
	return (write(&reg,1) == 1);
}

bool edi2c::writeByte(uint8_t reg, uint8_t byte)
{
	static uint8_t buf[2];
	buf[0] = reg;
	buf[1] = byte;
	return (write(buf,2) == 2);
}

bool edi2c::writeWord(uint8_t reg, int16_t word)
{
	static uint8_t buf[3];
	buf[0] = reg;
	buf[1] = uint8_t(word);
	buf[2] = uint8_t(word >> 8);
	return (write(buf, 3));
}

bool edi2c::writeBytes(uint8_t reg, uint8_t * bytes, uint32_t size)
{
	if (write(&reg,1) != 1)
		return false;
	return (write(bytes, size) == size);
}

int32_t edi2c::_raw_read(uint8_t * buffer, uint32_t size)
{
	bool use_smbus = smbus_enabled();
	if (use_smbus)
	{
		
	}
	else
		return ::read(m_fd, buffer, size);
    return 0;
}

int32_t edi2c::_raw_write(uint8_t * buffer, uint32_t size)
{
	bool use_smbus = smbus_enabled();
	if (use_smbus)
	{
		
	}
	else
		return ::write(m_fd, buffer, size);
    return 0;
}
