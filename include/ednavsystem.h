/*!
  \file   ednavsystem.h
  \author Daniel <dprandle@dprandle-CZ-17>
  \date   Fri Jul 10 10:34:08 2015
  
  \brief  Navigation system header file
  
  
*/

#ifndef EDNAVSYSTEM_H
#define EDNAVSYSTEM_H
#include <edsystem.h>

struct edmessage;
struct pulsed_light_message;

class ednav_system : public edsystem
{
  public:
    ednav_system();
    virtual ~ednav_system();
	
    virtual void init();
	virtual void release();
    virtual bool process(edmessage * msg);
    virtual void update();
	
    virtual std::string typestr() {return TypeString();}
   	static std::string TypeString() {return "ednav_system";}

  private:
	bool _process_pulse_light(pulsed_light_message * msg);
};

#endif
