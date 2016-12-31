#include <edtimer.h>
#include <unistd.h>
#include <edmessage.h>
#include <edmessage_dispatch.h>
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
    zero_buf(data,COMMAND_BYTE_SIZE);
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
    edm.message_dispatch()->register_listener<rplidar_health_message>(this);
    edm.message_dispatch()->register_listener<rplidar_info_message>(this);
    edm.message_dispatch()->register_listener<rplidar_firmware_message>(this);
    edm.message_dispatch()->register_listener<rplidar_scan_message>(this);
    edm.message_dispatch()->register_listener<pulsed_light_message>(this);
    edm.message_dispatch()->register_listener<nav_message>(this);

    m_server_fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);//
	if (m_server_fd < 0)
    {
        cprint("edcomm_system::init Could not create server");
        int err = errno;
        cprint("Error: " + std::string(strerror(err)));
    }

    int optval = 1;
    setsockopt(m_server_fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(int));

    server.sin_addr.s_addr = INADDR_ANY;
	server.sin_family = AF_INET;
    server.sin_port = htons(m_port);

    if (bind(m_server_fd, (struct sockaddr *) &server, sizeof(server)) < 0)
    {
        cprint("edcomm_system::init Could not bind server");
        int err = errno;
        cprint("Error: " + std::string(strerror(err)));
    }

	listen(m_server_fd, 5);
    cprint("edcomm_system::init Listening on port " + std::to_string(m_port));
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
    uint32_t hashid;
    data_packet * dp = NULL;

    if ( (msg->type() == "lplidar_scan_message") )
    {
        rplidar_scan_message * smsg = static_cast<rplidar_scan_message*>(msg);
        hashid = hash_id(complete_scan_data_packet::Type());
        sendToClients((uint8_t*)&hashid, sizeof(uint32_t)); // send the hash id first though
        _sendScan(smsg);
        return true;
    }
    else if (msg->type() == "rplidar_info_message")
    {
        rplidar_info_message * imsg = static_cast<rplidar_info_message*>(msg);
        hashid = hash_id(info_data_packet::Type());
        dp = &imsg->device_info;
    }
    else if (msg->type() == "rplidar_error_message")
    {
        // do nothing
        return true;
    }
    else if (msg->type() == "rplidar_health_message")
    {
        rplidar_health_message * hmsg = static_cast<rplidar_health_message*>(msg);
        hashid = hash_id(health_data_packet::Type());
        dp = &hmsg->device_health;
    }
    else if (msg->type() == "rplidar_firmware_message")
    {
        rplidar_firmware_message * fmsg = static_cast<rplidar_firmware_message*>(msg);
        hashid = hash_id(firmware_data_packet::Type());
        dp = &fmsg->device_firmware;
    }
    else if (msg->type() == "pulsed_light_message")
    {
        pulsed_light_message * plmsg = static_cast<pulsed_light_message*>(msg);
        hashid = hash_id(pulsed_light_message::Type());
        sendToClients((uint8_t*)&hashid, sizeof(uint32_t));
        sendToClients(plmsg->data, plmsg->size());
        return true;
    }
    else if (msg->type() == "nav_message")
    {
        nav_message * navmsg = static_cast<nav_message*>(msg);
        hashid = hash_id(nav_message::Type());
        sendToClients((uint8_t*)&hashid, sizeof(uint32_t));
        sendToClients(navmsg->data, navmsg->size());
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
        if (!new_client->start())
        {
            cprint("edcomm_system::update Received connection but could not start socket - should see deletion next");
        }
        m_clients.push_back(new_client);
        cprint("edcomm_system::update Server recieved connection from " + std::string(inet_ntoa(client_addr.sin_addr)) + ":" + std::to_string(ntohs(client_addr.sin_port)));

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
                  cprint("Connection closed with " + client_ip);
                  break;
              case (edthreaded_fd::DataOverwrite):
                  cprint("Socket internal buffer overwritten with new data before previous data was sent" + client_ip + "\nError: " + errno_message);
                  break;
              case (edthreaded_fd::InvalidRead):
                  cprint("Socket invalid read from " + client_ip + "\nError: " + errno_message);
                  break;
              case (edthreaded_fd::InvalidWrite):
                  cprint("Socket invalid write to " + client_ip + "\nError: " + errno_message);
                  break;
              case (edthreaded_fd::ThreadCreation):
                  cprint("Error in thread creation for connection with " + client_ip);
                  break;
              default:
                  cprint("No internal error but socket thread not running with " + client_ip);
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
    if (m_cur_index == COMMAND_BYTE_SIZE)
    {
        _do_command();
        m_cur_index = 0;
        zero_buf(m_cur_cmd.data, 8);
    }
}

void edcomm_system::_do_command()
{
    uint32_t rphash = hash_id(rplidar_request::Type());
    uint32_t nav_sys_command = hash_id(nav_system_request::Type());

    if (m_cur_cmd.hash_id == rphash)
    {
        rplidar_request::req_type rt = static_cast<rplidar_request::req_type>(m_cur_cmd.cmd_data);
        cprint("Sending rplidar request type: " + std::to_string(rt));
        rplidar_request * req = edm.message_dispatch()->push<rplidar_request>();
        if (req != NULL)
            req->r_type = rt;
    }
    else if (m_cur_cmd.hash_id == nav_sys_command)
    {
        nav_system_request * rmsg = edm.message_dispatch()->push<nav_system_request>();
        if (rmsg != NULL)
        {
            rmsg->pid.set(m_cur_cmd.cmd_data_d,m_cur_cmd.cmd_data_d2, m_cur_cmd.cmd_data_d3);
            rmsg->ramp_limit = m_cur_cmd.cmd_data_d4;
            rmsg->complex_der = ((m_cur_cmd.cmd_data & 0x01) == 0x01);
            rmsg->anti_reset_winding = ((m_cur_cmd.cmd_data & 0x10) == 0x10);
            rmsg->threshold_dropout = ((m_cur_cmd.cmd_data & 0x0100) == 0x0100);
            rmsg->bias_vec.set(m_cur_cmd.cmd_data_d6, m_cur_cmd.cmd_data_d5);
            rmsg->g_factor = m_cur_cmd.cmd_data_d7;
            rmsg->bias_threshold_dist = m_cur_cmd.cmd_data_d8;
        }
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
