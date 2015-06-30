#include <fstream>
#include <unistd.h>
#include <exception>
#include <stdexcept>
#include <edutility.h>
#include <ctime>
#include <iostream>

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
    std::ofstream fout(fname.c_str());
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

