/*!
  \file   edmctrl.h
  \author Daniel <dprandle@dprandle-CZ-17>
  \date   Fri Jul 10 09:20:01 2015
  
  \brief  Header file for master controller
  
  
*/


#ifndef MCTRL_H
#define MCTRL_H

#include <string>
#include <map>

#define edm edmctrl::inst()

class edsystem;
class edtimer;
class edmessage_dispatch;

typedef std::map<std::string,edsystem*> sysmap;

class edmctrl
{
  public:
    edmctrl();
    virtual ~edmctrl();
    
    template<class T>
    T * add_sys()
    {
        T * sys = new T();
        std::pair<sysmap::iterator,bool> ret = m_systems.insert(std::pair<std::string,edsystem*>(sys->typestr(), sys));
        if (!ret.second)
        {
            delete sys;
            return NULL;
        }
        return sys;
    }

    static edmctrl & inst();

    bool running();

	void init();
	
    void release();

	edmessage_dispatch * message_dispatch();

    void start();

	void stop();

	edtimer * sys_timer();

    void update();
    
    template<class T>
    void rm_sys()
    {
        rm_sys(T::TypeString());
    }

    void rm_sys(const std::string & sysname);

    template<class T>
    T * sys()
    {
        return static_cast<T>(sys(T::TypeString()));
    }

    edsystem * sys(const std::string & sysname);

	static void quit(void);
    
  private:
    bool m_running;
    sysmap m_systems;
	edtimer * m_systimer;
	edmessage_dispatch * m_msghandler;
};


#endif
