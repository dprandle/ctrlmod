/*!
  \file   edplsystem.h
  \author Daniel <dprandle@dprandle-CZ-17>
  \date   Tue Jul  7 09:19:49 2015
  
  \brief  System responsible for creating messages with laser distances
  
  
*/


#ifndef EDPLSYSTEM_H
#define EDPLSYSTEM_H

#define GPIO_14 36
#define GPIO_15 48
#define GPIO_48 33
#define GPIO_49 47

#include <edglobal.h>
#include <edsystem.h>
#include <mraa/gpio.hpp>
#include <edcallback.h>
#include <nsmath.h>
#include <map>

class edpl_system : public edsystem
{
	struct pl_gpio
	{
		pl_gpio(uint mraa_pin,double calibrate_offset=0.0, const vec3 & poffset=vec3(), const quat & orient_=quat());
		~pl_gpio();
		
		mraa::Gpio * pin;
		edtimer * timer;
		vec3 pos;
		quat orient;
		double cal_offset;
		bool meas_ready;
		static void isr(void *);
	};

	typedef std::map<uint, pl_gpio*> plmap;
	
  public:
    edpl_system();
    virtual ~edpl_system();

	pl_gpio * add_pl(uint mraa_pin,double c_offset=0.0, const vec3 & pos_offset=vec3(), const quat & orient_offset=quat());
	pl_gpio * get_pl(uint mraa_pin);
	void rm_pl(uint mraa_pin);
	bool pl_pin_taken(uint mraa_pin);
	void pl_set_pos(uint mraa_pin, const vec3 & pos_);
	void pl_set_orientation(uint mraa_pin, const quat & orient_);
	void pl_set_cal_offset(uint mraa_pin, double offset);
	
    virtual void init();
	virtual void release();
    virtual bool process(edmessage * msg);
    virtual void update();
    virtual std::string typestr();

   	static std::string TypeString() {return "edpl_system";}

  private:
	
	plmap m_pl_sensors;
};

#endif
