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


void edxv11_system::readByte(char byte)
{
	
	if (m_rec_index == 255)
	{
		std::string hexscan = "";

		for (uint i = 0; i < 256; ++i)
			hexscan += to_hex(m_rcvdbuf[i]) + " ";

		log_message(hexscan + "\n\n\n");
	}
	m_rcvdbuf[m_rec_index] = byte;
		
	// if (m_rec_pckt)
	// {
	// 	std::ostringstream ss;

	// 	if (m_current_data_ind == 1)
	// 	{
	// 		ss << "Packet index: " << uint(uchar(byte)) << "\n";
	// 		log_message(ss.str(), "status.log", false);
	// 	}

	// 	++m_current_data_ind;
	// 	if (m_current_data_ind == 22)
	// 	{
	// 		m_current_data_ind = 0;
	// 		m_rec_pckt = false;
	// 	}
	// }
	// else if (byte == char(PACKET_START))
	// {
	// 	m_rec_pckt = true;
	// 	++m_current_data_ind;
	// }
		
	++m_rec_index;
	
}

void edxv11_system::update()
{
	if (m_uart->dataAvailable()) // Check if data is available - return immediately 
	{
		char buffer[64];
		zero_buf(buffer, 64);
		int cnt = m_uart->read(buffer, 64);

		if (cnt == -1)
		{
			std::cout << "Error reading uart" << std::endl;
			return;
		}
		std::cout << "Read " << cnt << " bytes\n";
		for (int i = 0; i < cnt; ++i)
			readByte(buffer[i]);
	}
}

void edxv11_system::process(edmessage * msg)
{
    
}
