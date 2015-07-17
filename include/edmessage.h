#ifndef MESSAGE_H
#define MESSAGE_H

#include <edglobal.h>
#include <edutility.h>
#include <nsmath.h>

struct edmessage
{
	virtual ~edmessage() {}
	virtual std::string type()=0;
	uint ref_count;
};

struct pulsed_light_message : public edmessage
{
	double distance;
	uint mraa_pin;
	vec3 pos;
	quat orientation;

	virtual std::string type() {return Type();}
	static std::string Type() {return "pulsed_light_message";}
};

struct rplidar_request : public edmessage
{
	enum req_type
	{
		StartScan,
		ForceScan,
		StopScan,
		Reset,
		InfoReq,
		HealthReq
	};
	
	req_type r_type;
	virtual std::string type() {return Type();}
	static std::string Type() {return "lplidar_request";}
};
#endif
