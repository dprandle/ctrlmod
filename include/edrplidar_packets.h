#ifndef RPLIDAR_PACKETS_H
#define RPLIDAR_PACKETS_H

#include <string>

struct data_packet
{
    data_packet() {}
    virtual ~data_packet() {}
    virtual std::string toString()=0;
    virtual std::string type()=0;
    virtual unsigned int size()=0;
    virtual char & operator[](unsigned int index)=0;
    virtual char * dataptr()=0;
};

struct scan_data_packet : public data_packet
{
    scan_data_packet();

    virtual std::string toString();
    std::string type(){return Type();}
    virtual unsigned int size() {return 5;}
    virtual char & operator[](unsigned int index) {return data[index];}
    virtual char * dataptr(){return data;}
    static std::string Type() {return "scan";}

    union
    {
        struct
        {
            char qual_s_sn;
            char angle6to0_C;
            char angle14to7;
            char distance7to0;
            char distance15to8;
        };

        char data[5];
    };
};

struct complete_scan_data_packet : public data_packet
{
    complete_scan_data_packet();

    virtual std::string toString();
    std::string type(){return Type();}
    virtual unsigned int size() {return 5*360;}
    virtual char & operator[](unsigned int index)
    {
        unsigned int data_ind = index % 5;
        unsigned int packet_ind = index / 5;
        return data[packet_ind][data_ind];
    }
    virtual char * dataptr(){return data[0].data;}
    static std::string Type() {return "complete_scan";}
    scan_data_packet data[360];
};

struct health_data_packet : public data_packet
{
    health_data_packet();

    virtual std::string toString();
    virtual std::string type(){return Type();}
    virtual unsigned int size() {return 3;}
    virtual char & operator[](unsigned int index) {return data[index];}
    virtual char * dataptr(){return data;}
    static std::string Type() {return "health";}

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

    info_data_packet();

    virtual std::string toString();
    virtual std::string type(){return Type();}
    virtual unsigned int size() {return 20;}
    virtual char & operator[](unsigned int index) {return data[index];}
    virtual char * dataptr(){return data;}
    static std::string Type() {return "info";}

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

struct firmware_data_packet : public data_packet
{
    firmware_data_packet();
    virtual std::string toString();
    virtual std::string type(){return Type();}
    virtual unsigned int size() {return 56;}
    virtual char & operator[](unsigned int index) {return data[index];}
    virtual char * dataptr(){return data;}
    static std::string Type() {return "firmware";}
    union
    {
        struct
        {
            char line1[18];
            char line2[29];
            char line3[9];
        };
        char data[56];
    };
};

struct request_packet
{
    request_packet(char MSB, char LSB):
        msB(MSB),
        lsB(LSB) {}
    virtual ~request_packet() {}
	union
	{
		struct
		{
            char msB;
            char lsB;
		};
        char data[2];
	};
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

    virtual ~descriptor_packet() {}

    virtual std::string type()=0;
    unsigned int size() {return 7;}
    char & operator[](unsigned int index) {return data[index];}

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
