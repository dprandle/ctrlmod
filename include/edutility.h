#ifndef UTILITY_H
#define UTILITY_H

#include <edglobal.h>
#include <iostream>
#include <string>
#include <sstream>
#include <pthread.h>


uint32_t hash_id(const std::string & to_hash);

bool log_message(const std::string & msg, const std::string & fname="status.log", bool timestamp=true);

bool log_message_no_console(const std::string & msg, const std::string & fname, bool tmstmp);

void cprint(const std::string & str);

void cprint_flush();

std::string timestamp();

void delay(double ms);

std::string to_hex(uint8_t byte);

std::string to_hex(int16_t two_bytes);

std::string to_hex(uint16_t two_bytes);

std::string to_hex(int32_t four_bytes);

std::string to_hex(uint32_t four_bytes);

void zero_buf(uint8_t * buf, uint32_t size);

//! Copy buffer
/*! Copy the source buffer to destination buffer with possible offsets in each buffer
  
  \param src The source buffer
  \param dest The destination buffer
  \param size Amount of items to copy
  \param src_offset Offset in to the source buffer (defaults to 0)
  \param dest_offset Offset in to the destination buffer (defaults to 0)
*/
void copy_buf(const uint8_t * src, uint8_t * dest, uint32_t size, uint32_t src_offset=0, uint32_t dest_offset=0);


#endif
