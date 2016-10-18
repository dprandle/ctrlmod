#include <fstream>
#include <unistd.h>
#include <exception>
#include <stdexcept>
#include <edutility.h>
#include <ctime>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <edglobal.h>
#include <edtimer.h>

// Allow safe multithreaded use of cout
static pthread_mutex_t ss_lock = PTHREAD_MUTEX_INITIALIZER;

static std::string locked_str;

void delay(double ms)
{
	edtimer t;
	t.start();
	while (t.elapsed() < ms)
		t.update();
}

void cprint(const std::string & str)
{
    pthread_mutex_lock(&ss_lock);
	locked_str += str + "\n";
    pthread_mutex_unlock(&ss_lock);
}

void cprint_flush()
{
	std::string local_str;
    pthread_mutex_lock(&ss_lock);
	local_str = locked_str;
	locked_str.clear();
    pthread_mutex_unlock(&ss_lock);

	if (!local_str.empty())
	{
#ifdef CONSOLE_OUT
		std::cout << local_str << std::endl;
#endif
		log_message_no_console(local_str, CONSOLE_OUT_LOG, false);
	}
}

uint32_t hash_id(const std::string & strng)
{
	uint32_t hash = 5381;
	int32_t c;
	const char * str = strng.c_str();
    while ((c = *str++))
		hash = ((hash << 5) + hash) + c;

	return hash;
}

bool log_message_no_console(const std::string & msg, const std::string & fname, bool tmstmp)
{
    std::ofstream fout(fname.c_str(), std::ios_base::app);
    if (!fout.is_open())
		return false;
	
	if (tmstmp)
		fout << timestamp();

    fout << msg << "\n";
    fout.close();
	return true;
}

bool log_message(const std::string & msg, const std::string & fname, bool tmstmp)
{
    std::ofstream fout(fname.c_str(), std::ios_base::app);
    if (!fout.is_open())
		return false;
	
	if (tmstmp)
		fout << timestamp();

    fout << msg << "\n";
    fout.close();
	cprint(msg);
	return true;
}

std::string timestamp()
{
    time_t ltime = std::time(NULL); /* calendar time */
    std::string ret(std::asctime(std::localtime(&ltime)));
    return ret;
}

std::string to_hex(uint8_t byte)
{
	std::ostringstream ostr;
	ostr << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int32_t>(byte);
	return ostr.str();
}

std::string to_hex(int16_t two_bytes)
{
	std::ostringstream ostr;
	ostr << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << static_cast<int32_t>(two_bytes);
	return ostr.str();
}

std::string to_hex(uint16_t two_bytes)
{
	return to_hex(static_cast<int16_t>(two_bytes));
}

std::string to_hex(int32_t four_bytes)
{
	std::ostringstream ostr;
	ostr << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << four_bytes;
	return ostr.str();	
}

std::string to_hex(uint32_t four_bytes)
{
	return to_hex(static_cast<int32_t>(four_bytes));
}

void zero_buf(uint8_t * buf, uint32_t size)
{
	for (uint32_t i = 0; i < size; ++i) {
		buf[i] = 0;
	}
}

void copy_buf(const uint8_t * src, uint8_t * dest, uint32_t size, uint32_t src_offset, uint32_t dest_offset)
{
	const uint8_t * src_with_offset = src  + src_offset;
	uint8_t * dest_with_offset = dest + dest_offset;
	for (uint32_t i = 0; i < size; ++i)
		dest_with_offset[i] = src_with_offset[i];
}
