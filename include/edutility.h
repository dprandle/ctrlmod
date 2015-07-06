#ifndef UTILITY_H
#define UTILITY_H

#include <edglobal.h>
#include <string>

uint hash_id(const std::string & to_hash);

void log_message(const std::string & msg, const std::string & fname="status.log", bool timestamp=true);

std::string timestamp();

std::string to_hex(char byte);

void zero_buf(char * buf, uint size);

//! Copy buffer
/*! Copy the source buffer to destination buffer with possible offsets in each buffer
  
  \param src The source buffer
  \param dest The destination buffer
  \param size Amount of items to copy
  \param src_offset Offset in to the source buffer (defaults to 0)
  \param dest_offset Offset in to the destination buffer (defaults to 0)
*/
void copy_buf(char * src, char * dest, uint size, uint src_offset=0, uint dest_offset=0);


#endif
