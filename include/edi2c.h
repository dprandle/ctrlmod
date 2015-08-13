#ifndef EDI2C_H
#define EDI2C_H

#include <edthreaded_fd.h>
#include <string>

class edi2c : public edthreaded_fd
{
  public:

	edi2c(uint adapterNum);
	~edi2c();

	bool command(char reg);

	bool commandRead(char reg, uint bytes_to_read);

	void enable_smbus(bool enable);

	char readByte(char reg);

	sint readWord(char reg);

	void set_target_address(int addr);

	bool smbus_enabled();

	bool start();	

	int target_address();

	bool writeByte(char reg, char byte);

	bool writeWord(char reg, sint word);
	
	
  private:

	int _raw_read(char * buffer, uint size);

	int _raw_write(char * buffer, uint size);

	bool m_use_smbus;

	pthread_mutex_t m_smbus_lock;

	int m_address;

	std::string m_adapter_name;
};

#endif
