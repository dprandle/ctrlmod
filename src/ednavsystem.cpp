/*!
  \file   ednavsystem.cpp
  \author Daniel <dprandle@dprandle-CZ-17>
  \date   Fri Jul 10 10:44:40 2015
  
  \brief  Definition file for navigation system  
*/

#include <edutility.h>
#include <ednavsystem.h>
#include <edmctrl.h>
#include <edmessage_dispatch.h>
#include <edmessage.h>

ednav_system::ednav_system()
{
	
}

ednav_system::~ednav_system()
{
	
}

void ednav_system::init()
{
	edmessage_dispatch * mh = edm.message_dispatch();
	mh->register_listener<pulsed_light_message>(this);
}

void ednav_system::release()
{	
}

bool ednav_system::process(edmessage * msg)
{
	if (msg->type() == pulsed_light_message::Type())
		return _process_pulse_light(static_cast<pulsed_light_message*>(msg));
    return false;
}

void ednav_system::update()
{
}

bool ednav_system::_process_pulse_light(pulsed_light_message * msg)
{
	//std::cout << "Pin: " << msg->mraa_pin << "    Distance: " <<  msg->distance << " cm" << std::endl;
	return true;
}
