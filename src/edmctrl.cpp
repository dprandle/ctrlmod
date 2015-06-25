#include <edmctrl.h>

edmctrl::edmctrl()
{}

edmctrl::~edmctrl()
{}

edsystem * edmctrl::sys(const std::string & sysname)
{
    auto iter = m_systems.find(sysname);
    if (iter != m_systems.end())
        return iter->second;
    return NULL;
}

void edmctrl::rm_sys(const std::string & sysname)
{
    auto iter = m_systems.find(sysname);
    if (iter != m_systems.end())
    {
        delete iter->second;
        m_systems.erase(iter);
    }
}
