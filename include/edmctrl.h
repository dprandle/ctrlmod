#ifndef MCTRL_H
#define MCTRL_H
#include <string>
#include <map>

#define edm edmctrl::inst()

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

    void shutdown();

    void start();

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
    
  private:
    bool m_running;
    sysmap m_systems;
};


#endif
