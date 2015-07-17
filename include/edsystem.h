#ifndef EDSYSTEM_H
#define EDSYSTEM_H
#include <string>

struct edmessage;

class edsystem
{
  public:
    edsystem() {}
    virtual ~edsystem() {}

    virtual void init() = 0;
	virtual void release() = 0;
    virtual bool process(edmessage * msg)=0;
    virtual void update() = 0;
    virtual std::string typestr() = 0;
  private:	
};


#endif
