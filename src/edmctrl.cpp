/*!
  \file   edmctrl.cpp
  \author Daniel <dprandle@dprandle-CZ-17>
  \date   Fri Jul 10 09:19:32 2015
  
  \brief  Master control file for the edison
  
  
*/

#include <sstream>
#include <edutility.h>
#include <iostream>
#include <edmctrl.h>
#include <edsystem.h>
#include <string>
#include <vector>
#include <edtimer.h>
#include <edmessage_dispatch.h>

edmctrl::edmctrl():
	m_running(false),
	m_systimer(new edtimer()),
	m_msghandler(new edmessage_dispatch())
{
	
}

edmctrl::~edmctrl()
{
	delete m_systimer;
	delete m_msghandler;
    sysmap::iterator sysiter = m_systems.begin();
    while (sysiter != m_systems.end())
    {
        delete sysiter->second;
        ++sysiter;
    }
}

void edmctrl::init()
{
	log_message("Initializing edison control engine");
    sysmap::iterator sysiter = m_systems.begin();
    while (sysiter != m_systems.end())
    {
		std::ostringstream ss;
		ss << "Initializing system " << sysiter->first;
		log_message(ss.str());
        sysiter->second->init();
        ++sysiter;
    }
}

edmctrl & edmctrl::inst()
{
    static edmctrl controller;
    return controller;
}

bool edmctrl::running()
{
    return m_running;
}

void edmctrl::release()
{
    //log_message("Releasing edison control engine");
	sysmap::iterator sysiter = m_systems.begin();
    while (sysiter != m_systems.end())
    {
        log_message("Releasing system " + sysiter->first);
		sysiter->second->release();
        ++sysiter;
    }
}

void edmctrl::update()
{
	m_systimer->update();
    sysmap::iterator sysiter = m_systems.begin();
    while (sysiter != m_systems.end())
    {
        sysiter->second->update();
		m_msghandler->process_all(sysiter->second);
        ++sysiter;
    }
}

edmessage_dispatch * edmctrl::message_dispatch()
{
	return m_msghandler;
}

edtimer * edmctrl::sys_timer()
{
	return m_systimer;
}

edsystem * edmctrl::sys(const std::string & sysname)
{
    sysmap::iterator iter = m_systems.find(sysname);
    if (iter != m_systems.end())
        return iter->second;
    return NULL;
}

void edmctrl::rm_sys(const std::string & sysname)
{
    sysmap::iterator iter = m_systems.find(sysname);
    if (iter != m_systems.end())
    {
        delete iter->second;
        m_systems.erase(iter);
    }
}

void edmctrl::start()
{
	log_message("Starting edison control engine");
	m_running = true;
	m_systimer->start();
}

void edmctrl::stop()
{
	
	m_systimer->stop();
    //std::ostringstream oss;
    //oss << "Stopping edison control engine\nExecution time: " << m_systimer->elapsed() << " ms";
    //log_message(oss.str());
	m_running = false;
}

void edmctrl::quit(void)
{
	edm.stop();
	edm.release();
}
