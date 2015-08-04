#ifndef MESSAGE_H
#define MESSAGE_H

#include <edglobal.h>
#include <nsmath.h>
#include <edrplidar_packets.h>

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
        HealthReq,
        InfoReq,
		StartScan,
		ForceScan,
		StopScan,
        Reset
	};
	
	req_type r_type;
	virtual std::string type() {return Type();}
    static std::string Type() {return "rplidar_request";}
};

struct rplidar_scan_message : public edmessage
{
    complete_scan_data_packet scan_data;
	uint millis_timestamp;
	virtual std::string type() {return Type();}
	static std::string Type() {return "lplidar_scan_message";}		
};

struct rplidar_error_message : public edmessage
{
    rplidar_error_message();
	char message[100];
	virtual std::string type() {return Type();}
	static std::string Type() {return "rplidar_error_message";}	
};

struct rplidar_info_message : public edmessage
{
	info_data_packet device_info;
	virtual std::string type() {return Type();}
	static std::string Type() {return "rplidar_info_message";}
};

struct rplidar_health_message : public edmessage
{
	health_data_packet device_health;
	virtual std::string type() {return Type();}
	static std::string Type() {return "rplidar_health_message";}
};

struct rplidar_firmware_message : public edmessage
{
	firmware_data_packet device_firmware;
	virtual std::string type() {return Type();}
	static std::string Type() {return "rplidar_firmware_message";}
};

#endif
