#include <iostream>
#include "edxv11system.h"
#include "edutility.h"
#include <edmctrl.h>
#include <mraa/uart.hpp>

xv11scan::xv11scan()
{
	clear();
}

void xv11scan::clear()
{
	speed = 0;
	for (uint i = 0; i < PING_COUNT; ++i)
		pings[i] = 0;
}

edxv11_system::edxv11_system(): m_rec_index(0),
								m_rec_scan(false),
								m_current_scan_ind(0),
								m_current_ping_ind(0),
								m_uart(NULL)
{
	_clrbuf();
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
	m_uart->setBaudRate(XV_BAUD);
	m_uart->setMode(8, MRAA_UART_PARITY_NONE, 1);
}


void edxv11_system::update()
{
	static uchar recvd_speed_bytes = 0;
	static uchar ping_index = 0;
	std::cout << "Updating" << std::endl;
	if (m_uart->dataAvailable(0)) // Check if data is available - return immediately 
	{
		uchar cnt = m_uart->read(m_rcvdbuf,1); // Only ever read 1 byte at a time

		if (cnt > 1)
			log_message("Reading more than one byte from the buffer - shouldn't happen");
		
		// If not receiving scan then we should be checking for header bytes
		if (!m_rec_scan)
		{
			std::cout << "Recieved byte: " << m_rcvdbuf[m_rec_index];
			if (_checkHeader())
				m_rec_scan = true;
		}
		else
		{
			xv11scan * currentScan = m_scans[m_current_scan_ind];
			// Store the speed bytes - 16 bit value lsB received first msB received second
			if (recvd_speed_bytes != 2)
			{
				currentScan->speed |= (m_rcvdbuf[m_rec_index] << (8*recvd_speed_bytes));
				++recvd_speed_bytes;
			}
			else
			{
				// Already received speed bytes now we can revieve each ping
				// break down vars for debug purposes
				uint32_t bt = (uint32_t)m_rcvdbuf[m_rec_index];
				bt = (bt << (8*ping_index)); // Should now be in correct place
				currentScan->pings[m_current_ping_ind] |= bt;
				++ping_index;
				
				if (ping_index == 4) // We have received all four ping bytes - switch to next ping in scan
				{
					ping_index = 0;
					++m_current_ping_ind;
				}

				if (m_current_ping_ind == PING_COUNT) // we have finished this scan - increase scan index
				{
					recvd_speed_bytes = 0; // also reset received speed bytes
					m_rec_scan = false; // no longer receiving scan
					m_current_ping_ind = 0; // reset ping index
					++m_current_scan_ind;

					// If we have taken max amount of scans then reset the index to 0
					if (m_current_scan_ind == MAX_SCANS)
					{
						m_current_scan_ind = 0;
						log_message("20 scans have been received");
					}
				}
			}
		}

		m_rec_index += cnt;
		if (m_rec_index >= 16)
			m_rec_index = 0;
	}
}

void edxv11_system::process(edmessage * msg)
{
    
}

void edxv11_system::_clrbuf()
{
	for (uchar i = 0; i < 16; i++) {
		m_rcvdbuf[i] = 0;
	}
}

bool edxv11_system::_checkHeader()
{
	uchar ci = m_rec_index - 1;
	for (uchar i = 0; i < 4; i++)
	{
		ci = ci - i;
		if (ci >= 16)
			ci = 15;
		if (m_rcvdbuf[ci] != (RECV_FLAG << (8*i)))
			return false;
	}

}
