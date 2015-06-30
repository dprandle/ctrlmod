#ifndef UTILITY_H
#define UTILITY_H

#include <edglobal.h>
#include <string>

uint hash_id(const std::string & to_hash);

void log_message(const std::string & msg, const std::string & fname="status.log", bool timestamp=true);

std::string timestamp();

#endif
