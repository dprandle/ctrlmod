#ifndef MESSAGE_H
#define MESSAGE_H

#include <edglobal.h>
#include <string>
#include <edutility.h>

struct edmessage
{
    edmessage(const std::string & t_name) {id = hash_id(t_name);}
	uint id;
};

#endif
