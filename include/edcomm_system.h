#ifndef EDCOMM_SYSTEM_H
#define EDCOMM_SYSTEM_H

#include <edglobal.h>
#include <edsystem.h>

struct rplidar_scan_message;

struct Command
{
    Command();

    union
    {
        struct
        {
            uint hash_id;
            uint cmd_data;
        };
        char data[8];
    };
};

class edcomm_system : public edsystem
{
  public:

	edcomm_system();

	virtual ~edcomm_system();
	
    virtual void init();

	virtual void release();

	virtual bool process(edmessage * msg);

	virtual void update();

	virtual std::string typestr() {return TypeString();}
	
	static std::string TypeString() {return "edcomm_system";}

  private:
    void _handle_byte(char byte);
	void _sendScan(rplidar_scan_message * scanmessage);
    void _do_command();
	int m_socket_fd;
    Command m_cur_cmd;
    uchar m_cur_index;
};


#endif
