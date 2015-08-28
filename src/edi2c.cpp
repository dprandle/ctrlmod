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
	pthread_mutex_init(&m_rw_delay_lock, NULL);
}

edi2c::~edi2c()
{
	pthread_mutex_destroy(&m_smbus_lock);
	pthread_mutex_destroy(&m_rw_delay_lock);
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

uint8_t edi2c::read_byte()
{
	uint8_t ret;
	uint32_t cnt = 0;
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

uint16_t edi2c::read_word()
{
	uint16_t ret;
	uint32_t cnt = 0;
	while (cnt != 2)
	{
		cnt += read(((uint8_t*)&ret)+cnt, 2 - cnt);
		Error err = error();
		if (err.err_val != NoError)
		{
			log_message("edi2c::readWord error with retrieving byte: Error num " + std::to_string(err.err_val));
			break;
		}
	}
	return ret;	
}

void edi2c::read_reg_bytes(uint8_t reg, uint8_t * buffer, uint32_t size)
{
	uint32_t cnt = 0;
	
	command_read(reg, size);
	while (cnt != size)
	{
		cnt += read(buffer+cnt, size - cnt);
		Error err = error();
		if (err.err_val != NoError)
		{
			log_message("edi2c::readRegByte error with retrieving byte: Error num " + std::to_string(err.err_val));
			break;
		}
	}
}

bool edi2c::command_read(uint8_t reg, uint32_t bytes_to_read)
{
	return (write(&reg, 1, bytes_to_read) == 1);
}

uint8_t edi2c::read_reg_byte(uint8_t reg)
{
	uint8_t ret = 0x00;
	read_reg_bytes(reg, (uint8_t*)&ret, 1);
	return ret;
}

int16_t edi2c::read_reg_word(uint8_t reg)
{
	int16_t ret = 0x0000;
	read_reg_bytes(reg, (uint8_t*)&ret, 2);
	return ret;
}

bool edi2c::write_byte(uint8_t byte)
{
	return write(&byte, 1) == 1;
}

bool edi2c::write_word(uint16_t word)
{
	return write((uint8_t*)&word, 2) == 2;		
}

bool edi2c::write_reg_byte(uint8_t reg, uint8_t byte)
{
	static uint8_t buf[2];
	buf[0] = reg;
	buf[1] = byte;
	return (write(buf,2) == 2);
}

bool edi2c::write_reg_word(uint8_t reg, int16_t word)
{
	static uint8_t buf[3];
	buf[0] = reg;
	buf[1] = uint8_t(word);
	buf[2] = uint8_t(word >> 8);
	return (write(buf, 3) == 3);
}

bool edi2c::write_reg_bytes(uint8_t reg, uint8_t * bytes, uint32_t size)
{
	if (write(&reg,1) != 1)
		return false;
	return (write(bytes, size) == size+1);
}

void edi2c::set_read_delay(uint16_t ms)
{
	pthread_mutex_lock(&m_rw_delay_lock);
	m_read_delay = ms;
	pthread_mutex_unlock(&m_rw_delay_lock);
}

void edi2c::set_write_delay(uint16_t ms)
{
	pthread_mutex_lock(&m_rw_delay_lock);
	m_write_delay = ms;
	pthread_mutex_unlock(&m_rw_delay_lock);
}

uint16_t edi2c::read_delay()
{
	uint16_t ret;
	pthread_mutex_lock(&m_rw_delay_lock);
	ret = m_read_delay;
	pthread_mutex_unlock(&m_rw_delay_lock);
	return ret;
}

uint16_t edi2c::write_delay()
{
	uint16_t ret;
	pthread_mutex_lock(&m_rw_delay_lock);
	ret = m_write_delay;
	pthread_mutex_unlock(&m_rw_delay_lock);
	return ret;
}

int32_t edi2c::_raw_read(uint8_t * buffer, uint32_t size)
{
	bool use_smbus = smbus_enabled();
	if (m_current_wait_for_byte_count > 0)
	{
		if (use_smbus)
		{
			
		}
		else
		{
			uint32_t cnt = ::read(m_fd, buffer, size);
			delay(write_delay());
			return cnt;
		}
	}
    return 0;
}

int32_t edi2c::_raw_write(uint8_t * buffer, uint32_t size)
{
	bool use_smbus = smbus_enabled();
	if (use_smbus)
	{
		
	}
	else
	{
		uint32_t cnt = ::write(m_fd, buffer, size);
		delay(read_delay());
		return cnt;
	}
	return 0;
}
