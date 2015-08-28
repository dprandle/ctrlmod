#ifndef MESSAGE_H
#define MESSAGE_H

#include <edglobal.h>
#include <nsmath.h>
#include <edrplidar_packets.h>

struct edmessage
{
	virtual ~edmessage() {}
	virtual std::string type()=0;
	uint32_t ref_count;
};

struct pulsed_light_message : public edmessage
{
	union
	{
		struct
		{
			double distance1;
			double distance2;
			uint32_t mraa_pin1;
			uint32_t mraa_pin2;
			double pos1[3];
			double pos2[3];
			double orientation1[4];
			double orientation2[4];
		};
		uint8_t data[136];
	};

	uint32_t size() {return 136;}
	std::string type() {return Type();}
	static std::string Type() {return "pulsed_light_message";}
};

struct nav_message : public edmessage
{
	union
	{
		struct
		{
			int16_t throttle;
			int16_t pitch;
			int16_t roll;
			int16_t yaw;
			double rvec_raw[2];
			double rvec_corrected[2];
		};
		uint8_t data[40];
	};

	uint32_t size() {return 40;}
	std::string type() {return Type();}
	static std::string Type() {return "nav_message";}   
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

struct nav_system_request : public edmessage
{
	vec3 pid;
	double ramp_limit;
	vec2 bias_vec;
	double g_factor;
	double bias_threshold_dist;
	bool complex_der;
	bool anti_reset_winding;
	bool threshold_dropout;
	
	virtual std::string type() {return Type();}
    static std::string Type() {return "nav_system_request";}
};

struct rplidar_scan_message : public edmessage
{
    complete_scan_data_packet scan_data;
	uint32_t millis_timestamp;
	virtual std::string type() {return Type();}
	static std::string Type() {return "lplidar_scan_message";}		
};

struct rplidar_error_message : public edmessage
{
    rplidar_error_message();
	uint8_t message[100];
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
