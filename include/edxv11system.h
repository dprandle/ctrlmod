#ifndef EDXV11SYSTEM_H
#define EDXV11SYSTEM_H
#include <edsystem.h>
#include <mraa/uart.hpp>

class edxv11_system : public edsystem
{
  public:

    edxv11_system();

    virtual ~edxv11_system();

    void update();

    void process(edmessage * msg);
    
  private:

    
};


#endif
