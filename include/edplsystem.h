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

class edtimer;

class edpl_system : public edsystem
{
  public:
    edpl_system();
    virtual ~edpl_system();

    virtual void init();
    virtual void process(edmessage * msg);
    virtual void update();
    virtual std::string typestr();

   	static std::string TypeString() {return "edpl_system";}

	uchar lt;
	int val;
	
  private:
	mraa::Gpio * m_gpio0;
	mraa::Gpio * m_gpio1;
	mraa::Gpio * m_gpio2;
	mraa::Gpio * m_gpio3;
	edtimer * m_timer;
};

void gpio14_isr(void *);

#endif
