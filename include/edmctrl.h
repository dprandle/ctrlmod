#ifndef MCTRL_H
#define MCTRL_H
#include <string>
#include <map>

class edsystem;

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
        auto ret = m_systems.emplace(sys->typestr(), sys);
        if (!ret.second)
        {
            delete sys;
            return NULL;
        }
        return sys;
    }

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
    
  private:
    sysmap m_systems;
};


#endif
