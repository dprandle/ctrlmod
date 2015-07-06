#include <iostream>
#include "edxv11system.h"
#include "edutility.h"
#include <edmctrl.h>
#include <mraa/uart.hpp>
#include <sstream>

xv11packet::xv11packet()
{
	clear();
}

void xv11packet::clear()
{
	for (uint i = 0; i < 22; ++i)
		mem[i] = 0x00;
}

xv11scan::xv11scan()
{
	clear();
}

void xv11scan::clear()
{
	timestamp = "";
	for (uchar i = 0; i < PACKET_COUNT; ++i)
		packets[i].clear();
}

edxv11_system::edxv11_system(): m_rec_index(0),
								m_rec_pckt(false),
								m_current_scan_ind(0),
								m_current_packet_ind(0),
								m_current_data_ind(0),
								m_uart(NULL)
{
	zero_buf(m_rcvdbuf, RX_BUF_SIZE);
	m_scans.resize(MAX_SCANS);
	for (uchar i = 0; i < MAX_SCANS; i++)
	{
		m_scans[i] = new xv11scan();
	}
}

edxv11_system::~edxv11_system()
{
	for (uchar i = 0; i < MAX_SCANS; i++)
		delete m_scans[i];
	
	m_scans.clear();
}

void edxv11_system::init()
{
	try
	{
		m_uart = new mraa::Uart(0);
	}
	catch (std::exception & e)
	{
		std::cout << "Incorrectly configured UART" << std::endl;
		return;
	}
	std::cout << "Successfully initialized uart" << std::endl;
	if (m_uart->setBaudRate(XV_BAUD) != MRAA_SUCCESS)
	{
		std::cout << "Could not set baud rate correctly" << std::endl;
		delete m_uart;
		m_uart = NULL;
		return;
	}
	else
	{
		std::cout << "Baud rate set to " << XV_BAUD << std::endl;
	}
	if (m_uart->setMode(8, MRAA_UART_PARITY_NONE, 1) != MRAA_SUCCESS)
	{
		std::cout << "Could not set mode correctly" << std::endl;
		delete m_uart;
		m_uart = NULL;
		return;
	}
	if (m_uart->setFlowcontrol(false, false) != MRAA_SUCCESS)
	{
		std::cout << "Could not set flow control correctly" << std::endl;
		delete m_uart;
		m_uart = NULL;
		return;
	}
}


void edxv11_system::update()
{
	if (m_uart->dataAvailable()) // Check if data is available - return immediately 
	{
		char byte;
		uint cnt = m_uart->read(&byte,1);

		if (cnt == -1)
		{
			std::cout << "Error reading uart" << std::endl;
			return;
		}

		m_rcvdbuf[m_rec_index] = byte;

		// If we are in scan
		if (m_rec_pckt)
		{
			xv11packet * pckt = &m_scans[m_current_scan_ind]->packets[m_current_packet_ind];
			pckt->mem[m_current_data_ind] = byte;
			++m_current_data_ind;
			if (m_current_data_ind == 22)
			{
				m_current_data_ind = 0;
				m_rec_pckt = false;
				
				++m_current_packet_ind;
				if (m_current_packet_ind == char(PACKET_COUNT)) // Recieved entire scan !
				{
					std::cout << "Recieved Scan!" << std::endl;
					m_scans[m_current_scan_ind]->timestamp = timestamp(); // Timestamp finished scan
					
					m_current_packet_ind = 0;
					
					std::ostringstream ss;
					ss << "Received scan " << m_scans[m_current_scan_ind]->timestamp << std::endl;
					ss << "start: " << to_hex(pckt->start) << std::endl;
					ss << "packet index: " << to_hex(pckt->packet_index) << std::endl;
					ss << "packet speed: " << to_hex(pckt->mem[2]) << " " << to_hex(pckt->mem[3]) << std::endl;
					ss << "packe data: ";
					for (uchar i = 4; i < 22; ++i)
						ss << to_hex(pckt->mem[i]) << " ";
					ss << std::endl;
					log_message(ss.str(),"status.log",false);
					++m_current_scan_ind;
					if (m_current_scan_ind == MAX_SCANS)
						m_current_scan_ind = 0;
				}
			}
		}
		else
		{
			if (byte == char(PACKET_START))
			{
				m_rec_pckt = true;
				m_scans[m_current_scan_ind]->packets[m_current_packet_ind].mem[m_current_data_ind] = byte;
				++m_current_data_ind;
			}
		}
		
		++m_rec_index;
	}
}

void edxv11_system::process(edmessage * msg)
{
    
}
