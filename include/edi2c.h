#ifndef EDI2C_H
#define EDI2C_H

#include <edthreaded_fd.h>
#include <string>

class edi2c : public edthreaded_fd
{
  public:

	edi2c(uint32_t adapterNum);
	~edi2c();

	bool command(uint8_t reg);

	bool commandRead(uint8_t reg, uint32_t bytes_to_read);

	void enable_smbus(bool enable);

	uint8_t readByte(uint8_t reg);

	void readBytes(uint8_t reg, uint8_t * buffer, uint32_t size);

	int16_t readWord(uint8_t reg);

	void set_target_address(int32_t addr);

	bool smbus_enabled();

	bool start();	

	int32_t target_address();

	bool writeByte(uint8_t reg, uint8_t byte);

	bool writeWord(uint8_t reg, int16_t word);

	bool writeBytes(uint8_t reg, uint8_t * bytes, uint32_t size);
	
  private:

	int32_t _raw_read(uint8_t * buffer, uint32_t size);

	int32_t _raw_write(uint8_t * buffer, uint32_t size);

	bool m_use_smbus;

	pthread_mutex_t m_smbus_lock;

	int32_t m_address;

	std::string m_adapter_name;
};

#endif
