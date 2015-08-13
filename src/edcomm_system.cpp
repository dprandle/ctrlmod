#include <edtimer.h>
#include <unistd.h>
#include <edmessage.h>
#include <edmsghandler.h>
#include <edmctrl.h>
#include <edcomm_system.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <edutility.h>
#include <edglobal.h>
#include <string.h>
#include <errno.h>
#include <edsocket.h>
#include <sstream>

Command::Command()
{
    zero_buf(data,8);
}
edcomm_system::edcomm_system():
    m_server_fd(0),
	m_port(0),
    m_cur_cmd(),
    m_cur_index(0)
{}

edcomm_system::~edcomm_system()
{}
	
void edcomm_system::init()
{
	sockaddr_in server;
    edm.messages()->register_listener<rplidar_health_message>(this);
    edm.messages()->register_listener<rplidar_info_message>(this);
    edm.messages()->register_listener<rplidar_firmware_message>(this);
    edm.messages()->register_listener<rplidar_scan_message>(this);
	edm.messages()->register_listener<pulsed_light_message>(this);

    m_server_fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);//
	if (m_server_fd < 0)
		log_message("Could not create server");

    server.sin_addr.s_addr = INADDR_ANY;
	server.sin_family = AF_INET;
    server.sin_port = htons(m_port);

	if (bind(m_server_fd, (struct sockaddr *) &server, sizeof(server)) < 0) 
		log_message("Could not bind server");

	listen(m_server_fd, 5);
    log_message("Listening on port " + std::to_string(m_port));
}

uint16_t edcomm_system::port()
{
	return m_port;
}

void edcomm_system::set_port(uint16_t port_)
{
	m_port = port_;
}

void edcomm_system::release()
{
	while (m_clients.begin() != m_clients.end())
	{
		delete m_clients.back();
		m_clients.pop_back();
	}
	close(m_server_fd);
}

bool edcomm_system::process(edmessage * msg)
{
    rplidar_info_message * imsg = NULL;
    rplidar_error_message * emsg = NULL;
    rplidar_health_message * hmsg = NULL;
    rplidar_firmware_message * fmsg = NULL;
    rplidar_scan_message * smsg = NULL;
	pulsed_light_message * plmsg = NULL;
    uint32_t hashid;
    data_packet * dp = NULL;

    if ( (smsg = dynamic_cast<rplidar_scan_message*>(msg)) )
    {
		// special case - sending variable amount of data for each scan so handle that
		// in other function
        hashid = hash_id(complete_scan_data_packet::Type());
        sendToClients((uint8_t*)&hashid, sizeof(uint32_t)); // send the hash id first though
		_sendScan(smsg);
		return true;
    }
    else if ( (imsg = dynamic_cast<rplidar_info_message*>(msg)) )
    {
        hashid = hash_id(info_data_packet::Type());
        dp = &imsg->device_info;
    }
    else if ( (emsg = dynamic_cast<rplidar_error_message*>(msg)) )
    {
        //hashid = hash_id(emsg->device_info.type());
    }
    else if ( (hmsg = dynamic_cast<rplidar_health_message*>(msg)) )
    {
        hashid = hash_id(health_data_packet::Type());
        dp = &hmsg->device_health;
    }
    else if ( (fmsg = dynamic_cast<rplidar_firmware_message*>(msg)) )
    {
        hashid = hash_id(firmware_data_packet::Type());
        dp = &fmsg->device_firmware;
    }
    else if ( (plmsg = dynamic_cast<pulsed_light_message*>(msg)))
	{
		hashid = hash_id(pulsed_light_message::Type());
		sendToClients((uint8_t*)&hashid, sizeof(uint32_t));
		sendToClients(plmsg->data, plmsg->size());
		return true;
	}
	else
        return false;

    sendToClients((uint8_t*)&hashid, sizeof(uint32_t));
    sendToClients(dp->dataptr(), dp->size());
    return true;
}

uint32_t edcomm_system::recvFromClients(uint8_t * data, uint32_t max_size)
{
	uint32_t total = 0;
	for (uint32_t i = 0; i < m_clients.size(); ++i)
		total += m_clients[i]->read(data+total, max_size-total);
	return total;
}

void edcomm_system::sendToClients(uint8_t * data, uint32_t size)
{
	for (uint32_t i = 0; i < m_clients.size(); ++i)
		m_clients[i]->write(data, size);
}

void edcomm_system::update()
{
	sockaddr_in client_addr;
	socklen_t client_len = sizeof(client_addr);
	int32_t sockfd = accept(m_server_fd, (struct sockaddr *) &client_addr, &client_len);
	if (sockfd != -1)
	{
		edsocket * new_client = new edsocket(sockfd);
		new_client->start();
		m_clients.push_back(new_client);
		log_message("Recieved connection from " + std::string(inet_ntoa(client_addr.sin_addr)) + ":" + std::to_string(ntohs(client_addr.sin_port)));
	}

	// Check for closed connections and remove them if there are any
	_clean_closed_connections();
	
	
    static uint8_t buf[256];
    int32_t cnt = recvFromClients(buf, 256);
    for (int32_t i = 0; i < cnt; ++i)
        _handle_byte(buf[i]);
}

void edcomm_system::_clean_closed_connections()
{
	ClientArray::iterator iter = m_clients.begin();
	while (iter != m_clients.end())
	{
		if (!(*iter)->running())
		{
			sockaddr_in cl_addr;
			socklen_t cl_len = sizeof(cl_addr);
			getpeername((*iter)->fd(), (sockaddr *)&cl_addr, &cl_len);
            std::string client_ip = std::string(inet_ntoa(cl_addr.sin_addr)) + ":" + std::to_string(ntohs(cl_addr.sin_port));

            edthreaded_fd::Error er = (*iter)->error();
            std::string errno_message = strerror(er._errno);

			switch(er.err_val)
			{
			  case(edthreaded_fd::ConnectionClosed):
                  log_message("Connection closed with " + client_ip);
				  break;
			  case (edthreaded_fd::DataOverwrite):
				  log_message("Socket internal buffer overwritten with new data before previous data was sent" + client_ip + "\nError: " + errno_message);
				  break;
			  case (edthreaded_fd::InvalidRead):
				  log_message("Socket invalid read from " + client_ip + "\nError: " + errno_message);
				  break;
			  case (edthreaded_fd::InvalidWrite):
				  log_message("Socket invalid write to " + client_ip + "\nError: " + errno_message);
				  break;
			  case (edthreaded_fd::ThreadCreation):
				  log_message("Error in thread creation for connection with " + client_ip);
				  break;
			  default:
				  log_message("No internal error but socket thread not running with " + client_ip);
				  break;
			}
			delete (*iter);
			iter = m_clients.erase(iter);
		}
		else
			++iter;
	}
}

void edcomm_system::_handle_byte(uint8_t byte)
{
    m_cur_cmd.data[m_cur_index] = byte;
    ++m_cur_index;
    if (m_cur_index == 8)
    {
        _do_command();
        m_cur_index = 0;
        zero_buf(m_cur_cmd.data, 8);
    }
}

void edcomm_system::_do_command()
{
    uint32_t rphash = hash_id(rplidar_request::Type());
    if (m_cur_cmd.hash_id == rphash)
    {
        rplidar_request::req_type rt = static_cast<rplidar_request::req_type>(m_cur_cmd.cmd_data);
        rplidar_request * req = edm.messages()->push<rplidar_request>();
        if (req != NULL)
            req->r_type = rt;
    }
}

void edcomm_system::_sendScan(rplidar_scan_message * scanmessage)
{
	static uint32_t scanid = 0;
	std::vector<uint32_t> tosend;
	tosend.reserve(720);
	for (uint32_t i = 0; i < 360; ++i)
	{
		scan_data_packet * curpacket = &scanmessage->scan_data.data[i];

		uint32_t angle = ((uint32_t)(curpacket->angle6to0_C) >> 1) & 0x7F;
		angle |= ((uint32_t)(curpacket->angle14to7) << 7) & 0x7F80;

		uint32_t distance = (uint32_t)(curpacket->distance7to0) & 0xFF;
		distance |= ((uint32_t)(curpacket->distance15to8) << 8) & 0xFF00;

        if (distance != 0)
        {
            tosend.push_back(angle);
            tosend.push_back(distance);
        }
	}
	uint32_t packetsize = tosend.size();
    sendToClients((uint8_t*)&packetsize, sizeof(uint32_t));
    sendToClients((uint8_t*)&tosend[0], tosend.size()*sizeof(uint32_t));
	++scanid;
}
