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

Command::Command()
{
    zero_buf(data,8);
}
edcomm_system::edcomm_system():
    m_socket_fd(0),
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

    m_socket_fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
	if (m_socket_fd == -1)
		log_message("Could not create socket");

	log_message("Created socket");
    server.sin_addr.s_addr = inet_addr("172.20.203.149");
	server.sin_family = AF_INET;
    server.sin_port = htons(2345);

    if (connect(m_socket_fd, (struct sockaddr*)&server, sizeof(server)) < 0)
    {
        char * ret = strerror(errno);
        std::string err(ret);
        log_message("Conection error with ip: 172.20.203.149 Error: " + err);
        return;
    }
    log_message("Connected to 172.20.203.149");
}

void edcomm_system::release()
{
    close(m_socket_fd);
}

bool edcomm_system::process(edmessage * msg)
{
    rplidar_info_message * imsg;
    rplidar_error_message * emsg;
    rplidar_health_message * hmsg;
    rplidar_firmware_message * fmsg;
    rplidar_scan_message * smsg;
    uint hashid;
    data_packet * dp = NULL;

    if ( (smsg = dynamic_cast<rplidar_scan_message*>(msg)) )
    {
		// special case - sending variable amount of data for each scan so handle that
		// in other function
        hashid = hash_id(smsg->scan_data.type());
        write(m_socket_fd, (char*)&hashid, sizeof(uint)); // send the hash id first though
		_sendScan(smsg);
		return true;
    }
    else if ( (imsg = dynamic_cast<rplidar_info_message*>(msg)) )
    {
        hashid = hash_id(imsg->device_info.type());
        dp = &imsg->device_info;
    }
    else if ( (emsg = dynamic_cast<rplidar_error_message*>(msg)) )
    {
        //hashid = hash_id(emsg->device_info.type());
    }
    else if ( (hmsg = dynamic_cast<rplidar_health_message*>(msg)) )
    {
        hashid = hash_id(hmsg->device_health.type());
        dp = &hmsg->device_health;
    }
    else if ( (fmsg = dynamic_cast<rplidar_firmware_message*>(msg)) )
    {
        hashid = hash_id(fmsg->device_firmware.type());
        dp = &fmsg->device_firmware;
    }
    else
        return false;

    write(m_socket_fd, (char*)&hashid, sizeof(uint));
    write(m_socket_fd, dp->dataptr(), dp->size());
    return true;
}

void edcomm_system::update()
{
    static char buf[32];
    int cnt = read(m_socket_fd, buf, 32);
    for (int i = 0; i < cnt; ++i)
        _handle_byte(buf[i]);
}

void edcomm_system::_handle_byte(char byte)
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
    uint rphash = hash_id(rplidar_request::Type());
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
	std::ostringstream ss;
	std::vector<uint32_t> tosend;
	tosend.reserve(360);
	for (uint i = 0; i < 360; ++i)
	{
		scan_data_packet * curpacket = &scanmessage->scan_data.data[i];
	    uint32_t angle = ((uint32_t)(curpacket->angle6to0_C) >> 1) & 0x7F;
		angle |= ((uint32_t)(curpacket->angle14to7) << 7) & 0x7F80;

		uint32_t distance = (uint32_t)(curpacket->distance7to0) & 0xFF;
		distance |= ((unsigned int)(curpacket->distance15to8) << 8) & 0xFF00;

        if (distance != 0)
        {
            tosend.push_back(angle);
            tosend.push_back(distance);
			double ang = angle/64.0, dist = distance/4.0;
			ss << "\nAngle: " << ang;
			ss << "\nDistance: " << dist;
        }
	}
	log_message(ss.str());
	uint32_t packetsize = tosend.size();
    write(m_socket_fd, (char*)&packetsize, sizeof(uint32_t));
    write(m_socket_fd, tosend.data(), tosend.size()*sizeof(uint32_t));
}
