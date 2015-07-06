#include <fstream>
#include <unistd.h>
#include <exception>
#include <stdexcept>
#include <edutility.h>
#include <ctime>
#include <iostream>
#include <sstream>
#include <iomanip>

uint hash_string(const std::string & strng)
{
	uint hash = 5381;
	int c;
	const char * str = strng.c_str();
	while (c = *str++)
		hash = ((hash << 5) + hash) + c;

	return hash;
}

void log_message(const std::string & msg, const std::string & fname, bool tmstmp)
{
    std::ofstream fout(fname.c_str(), std::ios_base::app);
    if (!fout.is_open())
        throw std::exception();
	
	if (tmstmp)
		fout << timestamp() << " ";

    fout << msg << "\n";
    fout.close();
}

std::string timestamp()
{	
    time_t ltime = std::time(NULL); /* calendar time */
	return std::string(std::asctime(std::localtime(&ltime)));
}

std::string to_hex(char byte)
{
	std::ostringstream ostr;
	ostr << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << (unsigned int)(unsigned char)byte;
	return ostr.str();
}

void zero_buf(char * buf, uint size)
{
	for (uint i = 0; i < size; ++i) {
		buf[i] = 0;
	}
}

void copy_buf(char * src, char * dest, uint size, uint src_offset, uint dest_offset)
{
	char * src_with_offset = src  + src_offset;
	char * dest_with_offset = dest + dest_offset;
	for (uint i = 0; i < size; ++i)
		dest_with_offset[i] = src_with_offset[i];
}
