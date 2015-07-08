#ifndef EDXV11SYSTEM_H
#define EDXV11SYSTEM_H

#include <edglobal.h>
#include <edsystem.h>
#include <vector>

#define MAX_SCANS 20
#define XV_BAUD 115200
#define PACKET_COUNT 90
#define PACKET_START 0xFA

#define TMP_BUF_SIZE 1024
#define RX_BUF_SIZE 256

namespace mraa
{
class Uart;
}

//! xv11packet 
/*! 
  Contains 22 bytes
 */
struct xv11packet
{
	xv11packet();
	void clear();

	union
	{
		struct
		{
			uchar start;
			uchar packet_index;
			uint16_t speed;
			uint32_t data[4]; // Total of 16 bytes - 4 data vals four bytes each
			uint16_t checksum;
		};
		char mem[22];
	};
};

struct xv11scan
{
	xv11scan();
	void clear();

	std::string timestamp;
	xv11packet packets[PACKET_COUNT];
};

class edxv11_system : public edsystem
{
  public:

    edxv11_system();
    ~edxv11_system();

	void clear_scans();
	
    void init();
    
    void process(edmessage * msg);
	
    void update();

	void readByte(char byte);

    std::string typestr() {return TypeString();}

	static std::string TypeString() {return "edxc11_system";}
    
  private:	
	char m_rcvdbuf[RX_BUF_SIZE];
	uchar m_rec_index;
	bool m_rec_pckt;
	
	uint m_current_scan_ind;
	uint m_current_packet_ind;
	uint m_current_data_ind;
	
	mraa::Uart * m_uart;
	std::vector<xv11scan*> m_scans;
};


#endif
