#ifndef EDSYSTEM_H
#define EDSYSTEM_H
#include <string>

struct edmessage;

class edsystem
{
  public:

	edsystem() {}

	virtual ~edsystem() {}

    virtual void init() {}

    virtual void release() {}

    virtual bool process(edmessage * msg) = 0;

    virtual void update() {}

    virtual std::string typestr() = 0;
};


#endif
