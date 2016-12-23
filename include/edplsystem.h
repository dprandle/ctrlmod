/*!
  \file   edplsystem.h
  \author Daniel <dprandle@dprandle-CZ-17>
  \date   Tue Jul  7 09:19:49 2015
  
  \brief  System responsible for creating messages with laser distances
  
  
*/


#ifndef EDPLSYSTEM_H
#define EDPLSYSTEM_H

#include <edglobal.h>
#include <edsystem.h>
#include <edcallback.h>
#include <nsmath.h>
#include <map>

class edgpio;

struct pl_gpio
{
pl_gpio(uint32_t pin_num_,double calibrate_offset=0.0, const vec3 & poffset=vec3(), const quat & orient_=quat());
~pl_gpio();
		
edgpio * pin;
edtimer * timer;
vec3 pos;
quat orient;
double cal_offset;
bool meas_ready;
double sum_dist;
uint32_t meas_count;
static void isr(void *, int);
};

class edpl_system : public edsystem
{
  public:
	
	typedef std::map<uint32_t, pl_gpio*> plmap;
	
    edpl_system();
    virtual ~edpl_system();

	pl_gpio * add_pl(uint32_t pin_num_,double c_offset=0.0, const vec3 & pos_offset=vec3(), const quat & orient_offset=quat());
	pl_gpio * get_pl(uint32_t pin_num_);
	void rm_pl(uint32_t pin_num_);
	bool pl_pin_taken(uint32_t pin_num_);
	void pl_set_pos(uint32_t pin_num_, const vec3 & pos_);
	void pl_set_orientation(uint32_t pin_num_, const quat & orient_);
	void pl_set_cal_offset(uint32_t pin_num_, double offset);
	
    virtual void init();
	virtual void release();
    virtual bool process(edmessage * msg);
    virtual void update();
    virtual std::string typestr();

   	static std::string TypeString() {return "edpl_system";}

  private:
	
	plmap m_pl_sensors;
	edtimer * msgTimer;
};

struct edpl_callback : public edtimer_callback
{
	edpl_callback(pl_gpio * ceil, pl_gpio * floor):
		pl_ceil(ceil),
		pl_floor(floor)
	{}
	
	void exec();
	pl_gpio * pl_ceil;
	pl_gpio * pl_floor;
};

#endif
