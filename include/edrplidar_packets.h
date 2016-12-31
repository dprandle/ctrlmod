#ifndef RPLIDAR_PACKETS_H
#define RPLIDAR_PACKETS_H

#include <string>

struct data_packet
{
    data_packet() {}
    virtual ~data_packet() {}
    virtual std::string toString()=0;
    virtual std::string type()=0;
    virtual uint32_t size()=0;
    virtual uint8_t & operator[](uint32_t index)=0;
    virtual uint8_t * dataptr()=0;
};

struct scan_data_packet : public data_packet
{
    scan_data_packet();

    virtual std::string toString();
    std::string type(){return Type();}
    virtual uint32_t size() {return Size();}
    virtual uint8_t & operator[](uint32_t index) {return data[index];}
    virtual uint8_t * dataptr(){return data;}
    static std::string Type() {return "scan";}
	static uint32_t Size() {return 5;}
    union
    {
        struct
        {
            uint8_t qual_s_sn;
            uint8_t angle6to0_C;
            uint8_t angle14to7;
            uint8_t distance7to0;
            uint8_t distance15to8;
        };

        uint8_t data[5];
    };
};

struct complete_scan_data_packet : public data_packet
{
    complete_scan_data_packet();

    virtual std::string toString();
    std::string type(){return Type();}
    virtual uint32_t size() {return Size();}
    virtual uint8_t & operator[](uint32_t index)
    {
        uint32_t data_ind = index % 5;
        uint32_t packet_ind = index / 5;
        return data[packet_ind][data_ind];
    }
    virtual uint8_t * dataptr(){return data[0].data;}
    static std::string Type() {return "complete_scan";}
	static uint32_t Size() {return 5 * 360;}
    scan_data_packet data[360];
};

struct health_data_packet : public data_packet
{
    health_data_packet();

    virtual std::string toString();
    virtual std::string type(){return Type();}
    virtual uint32_t size() {return Size();}
    virtual uint8_t & operator[](uint32_t index) {return data[index];}
    virtual uint8_t * dataptr(){return data;}
    static std::string Type() {return "health";}
	static uint32_t Size() {return 3;}
    union
    {
        struct
        {
            uint8_t status;
            uint8_t error_code7to0;
            uint8_t error_code15to8;
        };
        uint8_t data[3];
    };
};

struct info_data_packet : public data_packet
{

    info_data_packet();

    virtual std::string toString();
    virtual std::string type(){return Type();}
    virtual uint32_t size() {return Size();}
    virtual uint8_t & operator[](uint32_t index) {return data[index];}
    virtual uint8_t * dataptr(){return data;}
    static std::string Type() {return "info";}
	static uint32_t Size() {return 20;}
    union
    {
        struct
        {
            uint8_t model;
            uint8_t firmware_minor;
            uint8_t firmware_major;
            uint8_t hardware;
            uint8_t serialnumber[16];
        };
        uint8_t data[20];
    };
};

struct firmware_data_packet : public data_packet
{
    firmware_data_packet();
    virtual std::string toString();
    virtual std::string type(){return Type();}
    virtual uint32_t size() {return Size();}
    virtual uint8_t & operator[](uint32_t index) {return data[index];}
    virtual uint8_t * dataptr(){return data;}
    static std::string Type() {return "firmware";}
	static uint32_t Size() {return 56;}
    union
    {
        struct
        {
            uint8_t line1[18];
            uint8_t line2[29];
            uint8_t line3[9];
        };
        uint8_t data[56];
    };
};

struct request_packet
{
    request_packet(uint8_t MSB, uint8_t LSB):
        msB(MSB),
        lsB(LSB) {}
    virtual ~request_packet() {}
	union
	{
		struct
		{
            uint8_t msB;
            uint8_t lsB;
		};
        uint8_t data[2];
	};
};


struct stop_scan_request : public request_packet
{
    stop_scan_request():request_packet(0xA5,0x25){}
};

struct start_scan_request : public request_packet
{
    start_scan_request():request_packet(0xA5,0x20){}
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
    descriptor_packet(uint8_t drlen0_=0x00, uint8_t drlen1_=0x00, uint8_t drlen2_=0x00, uint8_t drlen3_smode_=0x00, uint8_t datatype_=0x00):
        s1(0xA5),
        s2(0x5A),
        drlen0(drlen0_),
        drlen1(drlen1_),
        drlen2(drlen2_),
        drlen3_smode(drlen3_smode_),
        datatype(datatype_) {}

    virtual ~descriptor_packet() {}

    virtual std::string type()=0;
    uint32_t size() {return Size();}
    uint8_t & operator[](uint32_t index) {return data[index];}
	static uint32_t Size() {return 7;}
    union
    {
        struct {
            uint8_t s1;
            uint8_t s2;
            uint8_t drlen0;
            uint8_t drlen1;
            uint8_t drlen2;
            uint8_t drlen3_smode;
            uint8_t datatype;
        };
        uint8_t data[7];
    };
};

struct scan_descriptor : public descriptor_packet
{
    scan_descriptor():
        descriptor_packet(0x05,0x00,0x00,0x40,0x81){}
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

#endif
