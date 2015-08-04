#ifndef EDLOGGING_SYSTEM_H
#define EDLOGGING_SYSTEM_H

#include <edsystem.h>
#include <edglobal.h>
#include <edrplidar_packets.h>

class edlogging_system : public edsystem
{
  public:
    edlogging_system() {}
    virtual ~edlogging_system() {}

    virtual void init();
	virtual void release();
    virtual bool process(edmessage * msg);
    virtual void update();
	
    virtual std::string typestr() {return TypeString();}
	static std::string TypeString() {return "edlogging_system";}
	
  private:

	void log_device_info(info_data_packet * data);
	void log_device_health(health_data_packet * data);
	void log_device_firware(firmware_data_packet * data);
    void log_scan(complete_scan_data_packet * scand);
	
};



#endif
