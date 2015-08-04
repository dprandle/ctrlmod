/*!
  \file   edplsystem.cpp
  \author Daniel <dprandle@dprandle-CZ-17>
  \date   Tue Jul  7 09:32:32 2015
  
  \brief  Definitions for system
  
  
*/


#include <edplsystem.h>
#include <edutility.h>
#include <mraa.hpp>
#include <iostream>
#include <edtimer.h>
#include <vector>
#include <edmsghandler.h>
#include <edmessage.h>
#include <edmctrl.h>

edpl_system::edpl_system()
{
}

edpl_system::~edpl_system()
{
}

void edpl_system::init()
{
	std::vector<uint> gpio_pins;
	gpio_pins.push_back(GPIO_14);
	gpio_pins.push_back(GPIO_15);
	gpio_pins.push_back(GPIO_48);
	gpio_pins.push_back(GPIO_49);

	for (uint i = 0; i < gpio_pins.size(); ++i)
    {
		pl_gpio * pnt = add_pl(gpio_pins[i],-20.0);
        if (pnt == NULL)
        {
            std::ostringstream ss;
            ss << "Error initializing GPIO pin " << gpio_pins[i];
            log_message(ss.str());
        }
    }

}

edpl_system::pl_gpio * edpl_system::add_pl(uint mraa_pin,double c_offset, const vec3 & pos_offset, const quat & orient_offset)
{
	if (pl_pin_taken(mraa_pin))
	{
		std::ostringstream oss;
		oss << "Cannot initialize pin " << mraa_pin << " as its already initialized";
		log_message(oss.str());
		return NULL;
	}

	pl_gpio * pl = new pl_gpio(mraa_pin, c_offset, pos_offset, orient_offset);
	if (pl->pin == NULL)
	{
		std::ostringstream oss;
		oss << "Unable to initialize mraa pin " << mraa_pin;
		log_message(oss.str());
		delete pl;
		return NULL;
	}

	mraa_result_t res = pl->pin->dir(mraa::DIR_IN);
	if (res != MRAA_SUCCESS)
	{
		mraa::printError(res);
		delete pl;
		return NULL;
	}
	pl->pin->isr(mraa::EDGE_BOTH,edpl_system::pl_gpio::isr, pl);
	m_pl_sensors[mraa_pin] = pl;
	std::ostringstream ss;
	ss << "Successfully initialized pin " << mraa_pin;
	log_message(ss.str());
	return pl;
}

edpl_system::pl_gpio * edpl_system::get_pl(uint mraa_pin)
{
	plmap::iterator fiter = m_pl_sensors.find(mraa_pin);
	if (fiter != m_pl_sensors.end())
		return fiter->second;
	return NULL;
}

void edpl_system::pl_set_cal_offset(uint mraa_pin, double offset)
{
	pl_gpio * pl = get_pl(mraa_pin);
	if (pl !=NULL)
		pl->cal_offset = offset;
}

bool edpl_system::pl_pin_taken(uint mraa_pin)
{
	plmap::iterator fiter = m_pl_sensors.find(mraa_pin);
	return (fiter != m_pl_sensors.end());	
}

void edpl_system::rm_pl(uint mraa_pin)
{
	plmap::iterator fiter = m_pl_sensors.find(mraa_pin);
	if (fiter != m_pl_sensors.end())
	{
		delete fiter->second;
		m_pl_sensors.erase(fiter);
	}
}

void edpl_system::pl_set_pos(uint mraa_pin, const vec3 & pos_)
{
	plmap::iterator fiter = m_pl_sensors.find(mraa_pin);
	if (fiter != m_pl_sensors.end())
		fiter->second->pos = pos_;
}

void edpl_system::pl_set_orientation(uint mraa_pin, const quat & orient_)
{
	plmap::iterator fiter = m_pl_sensors.find(mraa_pin);
	if (fiter != m_pl_sensors.end())
		fiter->second->orient = orient_;
}

void edpl_system::release()
{
	plmap::iterator fiter = m_pl_sensors.begin();
	while (fiter != m_pl_sensors.end())
	{
		delete fiter->second;
		++fiter;
	}
	m_pl_sensors.clear();
}

bool edpl_system::process(edmessage * msg)
{
	return true;
}

void edpl_system::update()
{
	plmap::iterator iter = m_pl_sensors.begin();
	while (iter != m_pl_sensors.end())
	{
		if (iter->second->meas_ready)
		{
			edmessage_handler * mh = edm.messages();
			pulsed_light_message * msg = mh->push<pulsed_light_message>();
			
			msg->distance = iter->second->timer->elapsed() / 0.010 + iter->second->cal_offset;
			msg->mraa_pin = iter->first;
			msg->orientation = iter->second->orient;
			msg->pos = iter->second->pos;
			iter->second->meas_ready = false;
		}
		++iter;
	}
}

std::string edpl_system::typestr()
{
	return TypeString();
}


edpl_system::pl_gpio::pl_gpio(uint mraa_pin,double calibrate_offset, const vec3 & poffset, const quat & orient_):
	pin(new mraa::Gpio(mraa_pin)),
	timer(new edtimer()),
	pos(poffset),
	orient(orient_),
    cal_offset(calibrate_offset),
	meas_ready(false)
{}

edpl_system::pl_gpio::~pl_gpio()
{
	delete pin;
	delete timer;
}


void edpl_system::pl_gpio::isr(void * pl)
{
	edpl_system::pl_gpio * pl_cast = static_cast<edpl_system::pl_gpio*>(pl);
	if (!pl_cast->pin->read() == 0)
	{
		pl_cast->timer->start();
	}
	else
	{
		if (pl_cast->timer->running())
		{
			pl_cast->timer->stop();
			pl_cast->meas_ready = true;
		}
	}
}
