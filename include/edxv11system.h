#ifndef EDXV11SYSTEM_H
#define EDXV11SYSTEM_H

#include <edglobal.h>
#include <edsystem.h>
#include <vector>

#define MAX_SCANS 20
#define XV_BAUD 115200
#define PING_COUNT 360
#define RECV_FLAG 0xC000A55A

namespace mraa
{
class Uart;
}

struct xv11scan
{
	xv11scan();
	void clear();
	
	uint16_t speed;
	uint32_t pings[PING_COUNT];
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

    std::string typestr() {return TypeString();}

	static std::string TypeString() {return "edxc11_system";}
    
  private:
	void _clrbuf();
	
	bool _checkHeader();
	
	char m_rcvdbuf[16];
	uchar m_rec_index;
	bool m_rec_scan;
	
	uint m_current_scan_ind;
	uint m_current_ping_ind;
	mraa::Uart * m_uart;
	std::vector<xv11scan*> m_scans;
};


#endif
