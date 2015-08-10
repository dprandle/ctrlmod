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
	union
	{
		struct
		{
			double distance1;
			double distance2;
			uint mraa_pin1;
			uint mraa_pin2;
			double pos1[3];
			double pos2[3];
			double orientation1[4];
			double orientation2[4];
		};
		char data[136];
	};

	uint size() {return 136;}
	std::string type() {return Type();}
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
