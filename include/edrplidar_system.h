#ifndef EDLPLIDAR_SYSTEM_H
#define EDLPLIDAR_SYSTEM_H

#include <edglobal.h>
#include <edsystem.h>
#include <vector>
#include <edutility.h>
#include <edcallback.h>
#include <mraa/uart.h>
#include <edmessage.h>

#define XV_BAUD 115200


class edtimer;
class eduart;

struct data_packet
{
	virtual std::string type()=0;
	virtual uint size()=0;
	virtual char & operator[](uint index)=0;
};

struct scan_data_packet : public data_packet
{
	scan_data_packet()
	{
		zero_buf(data,5);
	}
	
	std::string type(){return "scan";}
	virtual uint size() {return 5;}
	virtual char & operator[](uint index) {return data[index];}
	
	union
	{
		struct
		{
			char qual_s_sn;
			char angle6to0_C;
			char angle14to7;
			char distance7to0;
			char distance15to0;
		};

		char data[5];
	};
};

struct health_data_packet : public data_packet
{
	health_data_packet()
	{
		zero_buf(data,3);
	}

	virtual std::string type(){return "health";}
	virtual uint size() {return 3;}
	virtual char & operator[](uint index) {return data[index];}

	union
	{
		struct
		{
			char status;
			char error_code7to0;
			char error_code15to8;
		};
		char data[3];
	};
};

struct info_data_packet : public data_packet
{
	info_data_packet()
	{
		zero_buf(data,20);
	}

	virtual std::string type(){return "info";}
	virtual uint size() {return 20;}
	virtual char & operator[](uint index) {return data[index];};

	union
	{
		struct
		{
			char model;
			char firmware_minor;
			char firmware_major;
			char hardware;
			char serialnumber[16];
		};
		char data[20];
	};
};

struct request_packet
{
	request_packet(char MSB, char LSB):
		msB(MSB),
		lsB(LSB) {}
	char msB;
	char lsB;
};


struct stop_scan_request : public request_packet
{
	stop_scan_request():request_packet(0xA5,0x25){}
};

struct start_scan_request : public request_packet
{
	start_scan_request():request_packet(0xA5,0x21){}
};

struct force_scan_request : public request_packet
{
	force_scan_request():request_packet(0xA5,0x21){}
};

struct reset_request : public request_packet
{
	reset_request(): request_packet(0xA5,0x40){}
};

struct device_info_request : public request_packet
{
	device_info_request(): request_packet(0xA5, 0x50) {}
};

struct device_health_request : public request_packet
{
	device_health_request(): request_packet(0xA5, 0x52) {}
};

struct descriptor_packet
{
	descriptor_packet(char drlen0_=0x00, char drlen1_=0x00, char drlen2_=0x00, char drlen3_smode_=0x00, char datatype_=0x00):
		s1(0xA5),
		s2(0x5A),
		drlen0(drlen0_),
		drlen1(drlen1_),
		drlen2(drlen2_),
		drlen3_smode(drlen3_smode_),
		datatype(datatype_) {}
		
	virtual std::string type()=0;
	uint size() {return 7;}
	char & operator[](uint index) {return data[index];}

	union
	{
		struct {
			char s1;
			char s2;
			char drlen0;
			char drlen1;
			char drlen2;
			char drlen3_smode;
			char datatype;
		};
		char data[7];
	};
};

struct scan_descriptor : public descriptor_packet
{
	scan_descriptor():
		descriptor_packet(0x05,0x00,0x00,0x01,0x81){}
	virtual std::string type(){return "scan";}
};

struct device_info_descriptor : public descriptor_packet
{
	device_info_descriptor():
		descriptor_packet(0x14,0x00,0x00,0x00,0x04){}
	virtual std::string type(){return "info";}
};

struct device_health_descriptor : public descriptor_packet
{
	device_health_descriptor():
		descriptor_packet(0x03,0x00,0x00,0x00,0x06){}
	virtual std::string type(){return "health";}
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

class edrplidar_system : public edsystem
{
  public:

	enum ExchangeType
	{
		Scan,
		Info,
		Health,
		None
	};
	
    edrplidar_system();
    ~edrplidar_system();

    void init();

	void release();
    
    bool process(edmessage * msg);
	
    void update();

    std::string typestr() {return TypeString();}

	static std::string TypeString() {return "edrplidar_system";}

  protected:
	
	bool startScan();

	bool forceScan();

	bool stopScan();

	bool reset();

	bool requestInfo();
	
	bool requestHealth();
	
  private:

	void _handle_byte(char byte);
	void _reset_state();
	bool _check_packet_for_scan_start();
	
	eduart * m_uart;
	edtimer * m_wait_timer;
	edtimer * m_timeout_timer;

	uint m_rec_index;
	uint m_scan_index;
	bool m_rec_descript;
	bool m_rec_start_scan;
	
	ExchangeType m_current_type;

	std::vector<descriptor_packet*> m_desc_packets;
	std::vector<data_packet*> m_data_packets;
	std::vector<scan_data_packet> m_current_scan;
};

struct wait_ready_callback : public edtimer_callback
{
	void exec();
};

#endif
