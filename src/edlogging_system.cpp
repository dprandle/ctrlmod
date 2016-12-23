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
	edm.message_dispatch()->register_listener<nav_message>(this);
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
	nav_message * nmsg;
	
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
		cprint("Received error message!");
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
    else if ( (nmsg = dynamic_cast<nav_message*>(msg)) )
	{
		log_nav_message(nmsg);
		return true;		
	}
	else
	{
		cprint("ERROR!!!!!");
		return false;
	}
}

void edlogging_system::update()
{
	cprint_flush();
}

void edlogging_system::log_device_info(info_data_packet * data)
{
    cprint("\n" + data->toString());
}

void edlogging_system::log_device_health(health_data_packet * data)
{
    cprint("\n" + data->toString());
}

void edlogging_system::log_device_firware(firmware_data_packet * data)
{
    cprint("\n" + data->toString());
}

void edlogging_system::log_scan(complete_scan_data_packet * data)
{
    //cprint(data->toString());
}

void edlogging_system::log_nav_message(nav_message * nmsg)
{
    // cprint("\nPitch: " + std::to_string(nmsg->pitch));
    // cprint("\nRoll: " + std::to_string(nmsg->roll));
    // cprint("\nYaw: " + std::to_string(nmsg->yaw));
    // cprint("\nThrottle: " + std::to_string(nmsg->throttle));
}
