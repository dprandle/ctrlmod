#ifndef EDSYSTEM_H
#define EDSYSTEM_H

struct edmessage;

class edsystem
{
  public:
    edsystem();
    virtual ~edsystem();
    
    virtual void process(edmessage * msg);
    virtual void update() = 0;
    virtual void typstr() = 0;
  private:	
};


#endif
