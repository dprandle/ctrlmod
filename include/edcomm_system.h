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
            uint32_t hash_id;
            uint32_t cmd_data;
        };
        uint8_t data[8];
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

	uint16_t port();

	void set_port(uint16_t port_);

	virtual void update();

	uint32_t recvFromClients(uint8_t * data, uint32_t max_size);

	void sendToClients(uint8_t * data, uint32_t size);

	virtual std::string typestr() {return TypeString();}
	
	static std::string TypeString() {return "edcomm_system";}

  private:
	typedef std::vector<edsocket*> ClientArray;
	
    void _handle_byte(uint8_t byte);
	void _sendScan(rplidar_scan_message * scanmessage);
    void _do_command();
	void _clean_closed_connections();

	ClientArray m_clients;
	
	int32_t m_server_fd;
	uint16_t m_port;
    Command m_cur_cmd;
    uint32_t m_cur_index;
};


#endif
