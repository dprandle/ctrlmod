/*!
  \file   edplsystem.cpp
  \author Daniel <dprandle@dprandle-CZ-17>
  \date   Tue Jul  7 09:32:32 2015
  
  \brief  Definitions for system
  
  
*/


#include <edplsystem.h>
#include <edutility.h>
#include <iostream>
#include <edtimer.h>
#include <vector>
#include <edmessage_dispatch.h>
#include <edmessage.h>
#include <edmctrl.h>
#include <edgpio.h>
#include <string.h>

edpl_system::edpl_system():
	edsystem(),
	msgTimer(new edtimer)
{
}

edpl_system::~edpl_system()
{
	delete msgTimer;
}

void edpl_system::init()
{
	std::vector<uint32_t> gpio_pins;
    gpio_pins.push_back(GPIO_14);
	gpio_pins.push_back(GPIO_15);

	for (uint32_t i = 0; i < gpio_pins.size(); ++i)
    {
		pl_gpio * pnt = add_pl(gpio_pins[i],-30);
        if (pnt == NULL)
        {
            std::ostringstream ss;
            ss << "Error initializing GPIO pin " << gpio_pins[i];
            cprint(ss.str());
        }
    }
	msgTimer->set_callback(new edpl_callback(get_pl(GPIO_14), get_pl(GPIO_15)));
    msgTimer->set_callback_delay(0.2);
	msgTimer->set_callback_mode(edtimer::continous_shot);
	msgTimer->start();
}

pl_gpio * edpl_system::add_pl(uint32_t pin_num,double c_offset, const vec3 & pos_offset, const quat & orient_offset)
{
	if (pl_pin_taken(pin_num))
	{
		std::ostringstream oss;
		oss << "Cannot initialize pin " << pin_num << " as its already initialized";
		cprint(oss.str());
		return NULL;
	}

	pl_gpio * pl = new pl_gpio(pin_num, c_offset, pos_offset, orient_offset);
	if (pl->pin == NULL)
	{
		std::ostringstream oss;
		oss << "Unable to initialize mraa pin " << pin_num;
		cprint(oss.str());
		delete pl;
		return NULL;
	}

	int res = pl->pin->set_direction(gpio_dir_in);
	if (res == -1)
	{
		std::ostringstream ss;
		gpio_error_state state = pl->pin->get_and_clear_error();
		ss << "Error initializing pin " << pin_num << "\nErrno: " << strerror(state.errno_code) << "\nGPIO Error: " << edgpio::error_string(state.gp_code);
		cprint(ss.str());
		delete pl;
		return nullptr;
	}

    res = pl->pin->set_isr(gpio_edge_both, pl_gpio::isr, pl);
	if (res == -1)
	{
		std::ostringstream ss;
		gpio_error_state state = pl->pin->get_and_clear_error();
		ss << "Error setting pin " << pin_num << " isr\nErrno: " << strerror(state.errno_code) << "\nGPIO Error: " << edgpio::error_string(state.gp_code);
		cprint(ss.str());
		delete pl;
		return nullptr;
	}

	m_pl_sensors[pin_num] = pl;
	std::ostringstream ss;
	ss << "Successfully initialized pin " << pin_num;
	cprint(ss.str());
	return pl;
}

pl_gpio * edpl_system::get_pl(uint32_t pin_num)
{
	plmap::iterator fiter = m_pl_sensors.find(pin_num);
	if (fiter != m_pl_sensors.end())
		return fiter->second;
	return NULL;
}

void edpl_system::pl_set_cal_offset(uint32_t pin_num, double offset)
{
	pl_gpio * pl = get_pl(pin_num);
	if (pl !=NULL)
		pl->cal_offset = offset;
}

bool edpl_system::pl_pin_taken(uint32_t pin_num)
{
	plmap::iterator fiter = m_pl_sensors.find(pin_num);
	return (fiter != m_pl_sensors.end());	
}

void edpl_system::rm_pl(uint32_t pin_num)
{
	plmap::iterator fiter = m_pl_sensors.find(pin_num);
	if (fiter != m_pl_sensors.end())
	{
		delete fiter->second;
		m_pl_sensors.erase(fiter);
	}
}

void edpl_system::pl_set_pos(uint32_t pin_num, const vec3 & pos_)
{
	plmap::iterator fiter = m_pl_sensors.find(pin_num);
	if (fiter != m_pl_sensors.end())
		fiter->second->pos = pos_;
}

void edpl_system::pl_set_orientation(uint32_t pin_num, const quat & orient_)
{
	plmap::iterator fiter = m_pl_sensors.find(pin_num);
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
	msgTimer->update();
    plmap::iterator iter = m_pl_sensors.begin();
    while (iter != m_pl_sensors.end())
    {
        iter->second->pin->update();
        ++iter;
    }
}

std::string edpl_system::typestr()
{
	return TypeString();
}


pl_gpio::pl_gpio(uint32_t pin_num_,double calibrate_offset, const vec3 & poffset, const quat & orient_):
	pin(new edgpio(pin_num_)),
	timer(new edtimer()),
	pos(poffset),
	orient(orient_),
    cal_offset(calibrate_offset),
	meas_ready(false),
	sum_dist(0.0),
	meas_count(0)
{}

pl_gpio::~pl_gpio()
{
	delete pin;
	delete timer;
}

void pl_gpio::isr(void * pl, int pin_edge)
{
    pl_gpio * pl_cast = static_cast<pl_gpio*>(pl);
    if (pin_edge)
    {
        pl_cast->timer->start();
    }
    else
    {
        pl_cast->timer->stop();
        double meas = pl_cast->timer->elapsed() * 100000.0 + pl_cast->cal_offset;
        if (meas > 0 && meas < 5000)
        {
            pl_cast->sum_dist += meas;
            ++pl_cast->meas_count;
        }
    }
}

void edpl_callback::exec()
{
	pulsed_light_message * msg = edm.message_dispatch()->push<pulsed_light_message>();
	if (pl_ceil->meas_count > 0)
		msg->distance1 = pl_ceil->sum_dist/pl_ceil->meas_count;
	if (pl_floor->meas_count > 0)
		msg->distance2 = pl_floor->sum_dist/pl_floor->meas_count;
	pl_ceil->sum_dist = 0;
	pl_ceil->meas_count = 0;
	pl_floor->sum_dist = 0;
	pl_floor->meas_count = 0;
	msg->mraa_pin1 = pl_ceil->pin->pin_num();
	msg->mraa_pin2 = pl_floor->pin->pin_num();
	copy_buf((uint8_t*)pl_ceil->pos.data, (uint8_t*)msg->pos1, 4);
	copy_buf((uint8_t*)pl_floor->pos.data, (uint8_t*)msg->pos2, 4);
	copy_buf((uint8_t*)pl_ceil->orient.data, (uint8_t*)msg->orientation1, 4);
	copy_buf((uint8_t*)pl_floor->orient.data, (uint8_t*)msg->orientation2, 4);
    cprint("Sending distance measurement...");
    cprint("Distance 1: " + std::to_string(msg->distance1*0.0328084) + " ft");
    cprint("Distance 2: " + std::to_string(msg->distance2*0.0328084) + " ft");
}
