#include <edlogging_system.h>
#include <edmctrl.h>
#include <edmessage_dispatch.h>
#include <edmessage.h>
#include <bitset>
void edlogging_system::init()
{
	edm.message_dispatch()->register_listener<rplidar_scan_message>(this);
	edm.message_dispatch()->register_listener<rplidar_health_message>(this);
	edm.message_dispatch()->register_listener<rplidar_info_message>(this);
	edm.message_dispatch()->register_listener<rplidar_firmware_message>(this);
	edm.message_dispatch()->register_listener<rplidar_error_message>(this);
}

void edlogging_system::release()
{
	
}

bool edlogging_system::process(edmessage * msg)
{
	rplidar_info_message * imsg;
	rplidar_error_message * emsg;
	rplidar_health_message * hmsg;
	rplidar_firmware_message * fmsg;
	rplidar_scan_message * smsg;
	
    if ( (smsg = dynamic_cast<rplidar_scan_message*>(msg)) )
	{
        log_scan(&smsg->scan_data);
		return true;
	}
    else if ( (imsg = dynamic_cast<rplidar_info_message*>(msg)) )
	{
		log_device_info(&imsg->device_info);
		return true;
	}
    else if ( (emsg = dynamic_cast<rplidar_error_message*>(msg)) )
	{
		std::cout << "Received error message!" << std::endl;
		return true;
	}
    else if ( (hmsg = dynamic_cast<rplidar_health_message*>(msg)) )
	{
		log_device_health(&hmsg->device_health);
		return true;		
	}
    else if ( (fmsg = dynamic_cast<rplidar_firmware_message*>(msg)) )
	{
		log_device_firware(&fmsg->device_firmware);
		return true;		
	}
	else
	{
		std::cout << "ERROR!!!!!" << std::endl;
		return false;
	}
}

void edlogging_system::update()
{
}

void edlogging_system::log_device_info(info_data_packet * data)
{
    log_message("\n" + data->toString());
}

void edlogging_system::log_device_health(health_data_packet * data)
{
    log_message("\n" + data->toString());
}

void edlogging_system::log_device_firware(firmware_data_packet * data)
{
    log_message("\n" + data->toString());
}

void edlogging_system::log_scan(complete_scan_data_packet * data)
{
	//log_message(data->toString());
}
