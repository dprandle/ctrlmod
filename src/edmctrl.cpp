#include <iostream>
#include "edmctrl.h"
#include "edsystem.h"
#include <string>
#include <vector>

edmctrl::edmctrl(): m_running(false)
{
	std::string ls;
	std::vector<int> s;
}

edmctrl::~edmctrl()
{}

void edmctrl::start()
{
    sysmap::iterator sysiter = m_systems.begin();
    while (sysiter != m_systems.end())
    {
		std::cout << "Initializing system " << sysiter->first << std::endl;
        sysiter->second->init();
        ++sysiter;
    }
    m_running = true;
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

void edmctrl::shutdown()
{
    m_running = false;
	std::cout << "Shutting down" << std::endl;
}

void edmctrl::update()
{
    sysmap::iterator sysiter = m_systems.begin();
    while (sysiter != m_systems.end())
    {
        sysiter->second->update();
        ++sysiter;
    }
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
