/*!
  \file   edi2c.h
  \author Daniel <dprandle@dprandle-CZ-17>
  \date   Thu Aug 27 17:39:59 2015
  
  \brief  Declaration file for edi2c class
*/


#ifndef EDI2C_H
#define EDI2C_H

#include <edthreaded_fd.h>
#include <string>

#define DEFAULT_READ_DELAY 20
#define DEFAULT_WRITE_DELAY 20

//! edi2c
/*! 
  Creates a new thread to run all communication transactions using i2c protocol. The way to use this
  class is just like any other "edthreaded_fd" subclass - with a main exception: The read_* functions are
  all blocking. The read function itself is not blocking. If you want to read a value from a register in
  a non blocking fashion then you must use command_read instead of read_* functions. The command_read
  function takes how many bytes to read as a parameter - and you can access theses bytes with "read".
 */
class edi2c : public edthreaded_fd
{
  public:

	edi2c(uint32_t adapterNum = 1);
    ~edi2c();

	//! command_read
/*!
  Read from a register in a non-blocking fashion - once the bytes have been read they will be available
  through read function. Nothing more will be written to the i2c device until bytes_to_read bytes have been
  read or the timeout period of time has been reached.
  
  \param reg register to read bytes from
  \param bytes_to_read amount of bytes to read
  
  \return Whether the command was successful or not. Will not be, for example, if no device is connected.
*/
	bool command_read(uint8_t reg, uint32_t bytes_to_read);

	//! enaable_smbus
/*!
  Disabled by default, this will enable the smbus functions. Smbus supports more advanced styles of messaging
  between devices but not all devices support the smbus protocol.
  
  \param enable Enable (true) or disable (false)
*/
	void enable_smbus(bool enable);

	//! read_byte
/*! 
  Blocks until one byte has been read or the maximum wait timeout has been reached.
  
  \return byte that has been read
*/
	uint8_t read_byte();

	//! read_word
/*! 
  Blocks until one word has been read or the maximum wait timeout has been reached.  
  
  \return 16 bit word
*/
	uint16_t read_word();

	//! read_reg_byte
/*! 
  Blocking read until 1 byte is read from a register.
  
  \param reg Register to read from
  
  \return Byte value read from register
*/
	uint8_t read_reg_byte(uint8_t reg);

	//! read_reg_word
/*! 
  Blocking read until 1 word is read from a register
  
  \param reg Register to read from
  
  \return 2 byte value read from register
*/
	int16_t read_reg_word(uint8_t reg);

	//! read_reg_bytes
/*! 
  Blocking read until size bytes has been read. This is a convenience function which is the same as calling
  command_read(reg, size);
  int cnt = 0;
  while (cnt != size)
      cnt += read(buffer+cnt, size-cnt);
  
  \param reg Register to read bytes from
  \param buffer Buffer to store read in bytes - bounds are not checked so make sure it is big enough
  \param size Number of bytes to read
*/
	void read_reg_bytes(uint8_t reg, uint8_t * buffer, uint32_t size);

	//! read_delay
/*! 
  \return number of milliseconds to delay the thread after each read command.
*/
	uint16_t read_delay();

	//! write_delay
/*! 
  \return number of milliseconds to delay the thread after each write command.
*/
	uint16_t write_delay();

	//! set_read_delay
/*! 
  Set the thread read delay - how many milliseconds to delay the thread after each read
  \param ms Number of milliseconds
*/
	void set_read_delay(uint16_t ms);

	//! 
/*! 
  Set the thread write delay - how many milliseconds to delay the thread after each write
  \param ms Number of milliseconds
*/
	void set_write_delay(uint16_t ms);

	//! set_target_address
/*! 
  Set the target device address - all reads and writes will be done using this address
  \param addr usually 7 bit device address for the slave (can be 10 bit)
*/
	void set_target_address(int32_t addr);

	//! smbus_enables
/*! 
  \return Is smbus mode enabled?
*/
	bool smbus_enabled();

	//! start
/*!
  Starts a new thread for communication with the device. It also opens the file descriptor using the
  bus number supplied in the constructor. By default this is bus 1. If a thread has already been started,
  or a file descriptor is already open, this function will fail.
  \return true for success and false for fail - check error code on fail 
*/
	bool start();

	int32_t target_address();

	bool write_byte(uint8_t byte);

	bool write_word(uint16_t word);

	bool write_reg_byte(uint8_t reg, uint8_t byte);

	bool write_reg_word(uint8_t reg, int16_t word);

	bool write_reg_bytes(uint8_t reg, uint8_t * bytes, uint32_t size);
	
  private:

	int32_t _raw_read(uint8_t * buffer, uint32_t size);

	int32_t _raw_write(uint8_t * buffer, uint32_t size);

	bool m_use_smbus;

	pthread_mutex_t m_smbus_lock;

	pthread_mutex_t m_rw_delay_lock;

	int32_t m_address;

	std::string m_adapter_name;

	uint16_t m_read_delay, m_write_delay;
};

#endif
