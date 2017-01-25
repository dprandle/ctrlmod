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
    if (msg->type() == "lplidar_scan_message")
	{
        rplidar_scan_message * smsg = static_cast<rplidar_scan_message*>(msg);
        log_scan(&smsg->scan_data);
		return true;
	}
    else if (msg->type() == "rplidar_info_message")
	{
        rplidar_info_message * imsg = static_cast<rplidar_info_message*>(msg);
		log_device_info(&imsg->device_info);
		return true;
	}
    else if (msg->type() == "rplidar_error_message")
	{
        rplidar_error_message * emsg = static_cast<rplidar_error_message*>(msg);
		log_message("Received error message!");
		return true;
	}
    else if (msg->type() == "rplidar_health_message")
	{
        rplidar_health_message * hmsg = static_cast<rplidar_health_message*>(msg);
		log_device_health(&hmsg->device_health);
		return true;		
	}
    else if (msg->type() == "rplidar_firmware_message")
	{
        rplidar_firmware_message * fmsg = static_cast<rplidar_firmware_message*>(msg);
		log_device_firware(&fmsg->device_firmware);
		return true;		
	}
    else if (msg->type() == "nav_message")
	{
        nav_message * nmsg = static_cast<nav_message*>(msg);
		log_nav_message(nmsg);
		return true;		
	}
	else
	{
		log_message("ERROR!!!!!");
		return false;
	}
}

void edlogging_system::update()
{
    cprint_flush();
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

void edlogging_system::log_nav_message(nav_message * nmsg)
{
    // log_message("\nPitch: " + std::to_string(nmsg->pitch));
    // log_message("\nRoll: " + std::to_string(nmsg->roll));
    // log_message("\nYaw: " + std::to_string(nmsg->yaw));
    // log_message("\nThrottle: " + std::to_string(nmsg->throttle));
}
