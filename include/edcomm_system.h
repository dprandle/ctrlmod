#ifndef EDCOMM_SYSTEM_H
#define EDCOMM_SYSTEM_H

#include <edglobal.h>
#include <edsystem.h>
#include <vector>

#define SOCKET_BUFF_SIZE 

struct rplidar_scan_message;
struct pulsed_light_message;
class edsocket;

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

	usint port();

	void set_port(usint port_);

	virtual void update();

	uint recvFromClients(char * data, uint max_size);

	void sendToClients(char * data, uint size);

	virtual std::string typestr() {return TypeString();}
	
	static std::string TypeString() {return "edcomm_system";}

  private:
	typedef std::vector<edsocket*> ClientArray;
	
    void _handle_byte(char byte);
	void _sendScan(rplidar_scan_message * scanmessage);
    void _do_command();
	void _clean_closed_connections();

	ClientArray m_clients;
	
	int m_server_fd;
	usint m_port;
    Command m_cur_cmd;
    uint m_cur_index;
};


#endif
