#include <edutility.h>

uint hash_string(const std::string & strng)
{
	uint hash = 5381;
	int c;
	const char * str = strng.c_str();
	while (c = *str++)
		hash = ((hash << 5) + hash) + c;

	return hash;
}
