#include <eduart.h>
#include <iostream>
#include <edmsghandler.h>
#include <edrplidar_system.h>
#include <edutility.h>
#include <edmctrl.h>
#include <mraa/uart.hpp>
#include <edtimer.h>
#include <unistd.h>
#include <termios.h>
#include <mraa/mraa_internal_types.h>

edrplidar_system::edrplidar_system():
	m_wait_timer(new edtimer()),
	m_timeout_timer(new edtimer()),
	m_rec_index(0),
	m_scan_index(0),
	m_rec_descript(false),
	m_rec_start_scan(false),
	m_current_type(None),
	m_uart(new eduart())
{
	m_data_packets.resize(3);
	m_desc_packets.resize(3);
	m_data_packets[Scan] = new scan_data_packet();
	m_desc_packets[Scan] = new scan_descriptor();
	m_data_packets[Info] = new info_data_packet();
	m_desc_packets[Info] = new device_info_descriptor();
	m_data_packets[Health] = new health_data_packet();
	m_desc_packets[Health] = new device_health_descriptor();
	m_current_scan.resize(360);
}

edrplidar_system::~edrplidar_system()
{
	delete m_uart;
	delete m_wait_timer;
	delete m_timeout_timer;
	while (m_desc_packets.begin() != m_desc_packets.end())
	{
		delete m_desc_packets.back();
		m_desc_packets.pop_back();
	}
	while (m_data_packets.begin() != m_data_packets.end())
	{
		delete m_data_packets.back();
		m_data_packets.pop_back();
	}
}

void edrplidar_system::init()
{
	edm.messages()->register_listener<rplidar_request>(this);
	m_uart->set_device_path("/dev/ttyMFD1");
	m_uart->set_format(eduart::d8, eduart::None, eduart::One);
	m_uart->set_baud(eduart::b115200);
	m_uart->init();

	m_wait_timer->set_callback(new wait_ready_callback());
	m_wait_timer->set_callback_mode(edtimer::single_shot);
}

void edrplidar_system::release()
{
	m_uart->release();
}


void edrplidar_system::update()
{
	static char readBuf[256];
	m_timeout_timer->update();
	m_wait_timer->update();
	
	if (m_timeout_timer->running() && m_timeout_timer->elapsed() > 4000)
	{
		std::cout << "Timeout for last request sending reset request" << std::endl;
		reset();
		return;
	}

	int size = m_uart->read(readBuf, 256);
	if (size > 0)
	{
		m_timeout_timer->stop();
		for (uint i = 0; i < size; ++i)
		{
			std::cout << "Incoming byte: " << to_hex(readBuf[i]) << std::endl;
			_handle_byte(readBuf[i]);
		}
	}

	edm.messages()->process_all(this);
}

bool edrplidar_system::process(edmessage * msg)
{
	if (m_wait_timer->running())
		return false;

	rplidar_request * rmsg = dynamic_cast<rplidar_request*>(msg);
 	if (rmsg != NULL)
	{
		switch (rmsg->r_type)
		{
		  case (rplidar_request::StartScan):
			  return startScan();
		  case (rplidar_request::ForceScan):
			  return forceScan();
		  case (rplidar_request::StopScan):
			  return stopScan();
		  case (rplidar_request::Reset):
			  return reset();
		  case (rplidar_request::InfoReq):
			  return requestInfo();
		  case (rplidar_request::HealthReq):
			  return requestHealth();
		}
	}
}


bool edrplidar_system::startScan()
{
	if (m_current_type != None)
	{
		log_message("Cannot start new scan: device is busy");
		return false;
	}
	
	std::cout << "Start scan command issued" << std::endl;
	start_scan_request sr;
	m_uart->write((char*)(&sr), sizeof(start_scan_request));

	// set descriptor to scan response
	m_current_type = Scan;
	
	m_wait_timer->set_callback_delay(1.0);
	m_wait_timer->start();
	m_timeout_timer->start();
	return true;
}

bool edrplidar_system::forceScan()
{
	if (m_current_type != None)
	{
		log_message("Cannot start new scan: device is busy");
		return false;
	}
	
	std::cout << "Force scan command issued" << std::endl;
	force_scan_request sr;
	m_uart->write((char*)(&sr), sizeof(force_scan_request));

	// set descriptor to scan response
	m_current_type = Scan;
	
	m_wait_timer->set_callback_delay(1.0);
	m_wait_timer->start();
	m_timeout_timer->start();
	return true;
}

bool edrplidar_system::stopScan()
{
	std::cout << "Stop scan command issued" << std::endl;
	_reset_state();
	stop_scan_request sr;
	m_uart->write((char*)(&sr), sizeof(stop_scan_request));
	m_wait_timer->set_callback_delay(1.0);
	m_wait_timer->start();
	return true;
}

bool edrplidar_system::reset()
{
	std::cout << "Reset command issued" << std::endl;
	_reset_state();
	m_timeout_timer->stop();
	reset_request rr;
	m_uart->write((char*)(&rr), sizeof(reset_request));
	m_wait_timer->set_callback_delay(30.0);
	m_wait_timer->start();
	return true;
}

bool edrplidar_system::requestInfo()
{
	if (m_current_type != None)
	{
		log_message("Cannot request device information: device is busy");
		return false;
	}
	
	std::cout << "Request device information command issued" << std::endl;
	device_info_request sr;
	m_uart->write((char*)(&sr), sizeof(device_info_request));

	// set descriptor to info response
	m_current_type = Info;
	
	m_wait_timer->set_callback_delay(1.0);
	m_wait_timer->start();
	m_timeout_timer->start();
	return true;
}
	
bool edrplidar_system::requestHealth()
{
	if (m_current_type != None)
	{
		log_message("Cannot request device health: device is busy");
		return false;
	}
	
	std::cout << "Request device health command issued" << std::endl;
	device_health_request sr;
	m_uart->write((char*)&sr, sizeof(device_health_request));

	// set descriptor to info response
	m_current_type = Health;

	m_wait_timer->set_callback_delay(1.0);
	m_wait_timer->start();
	m_timeout_timer->start();
	return true;
}

void edrplidar_system::_handle_byte(char byte)
{
	if (m_current_type == None)
		return;

	if (!m_rec_descript) // receiving descriptor packet
	{
		if ( (*m_desc_packets[m_current_type])[m_rec_index] != byte)
		{
			log_message("_handle_byte Data error: Incoming byte did not match descriptor");
			std::cout << "Desciptor byte: " << to_hex((*m_desc_packets[m_current_type])[m_rec_index]) << " at index: " << m_rec_index << std::endl;

			std::cout << "Printing desc data" << std::endl;
			std::cout << "Size: " << m_desc_packets[m_current_type]->size() << " Type: " << m_current_type << std::endl;
			for (uint i = 0; i < m_desc_packets[m_current_type]->size()-1; ++i)
				std::cout << "data[" << i << "] = " << to_hex((*m_desc_packets[m_current_type])[i]) << std::endl;
			_reset_state();
			return;
		}
		++m_rec_index;

		if (m_rec_index == m_desc_packets[m_current_type]->size())
		{
			log_message("Descriptor packet received for " + m_desc_packets[m_current_type]->type());
			m_rec_index = 0;
			m_rec_descript = true;
			m_rec_start_scan = false;
		}
	}
	else // receiving data
	{	
		(*m_data_packets[m_current_type])[m_rec_index] = byte;
		++m_rec_index;

		if (m_rec_index == m_data_packets[m_current_type]->size())
		{
			
			log_message("Data packet received for " + m_data_packets[m_current_type]->type());
			m_rec_index = 0;

			switch(m_current_type)
			{
			  case (Health):
			  {
				  rplidar_health_message * msg = edm.messages()->push<rplidar_health_message>();
				  if (msg != NULL)
					  msg->device_health = *(dynamic_cast<health_data_packet*>(m_data_packets[m_current_type]));
				  _reset_state();
				  return;
			  }
			  case (Info):
			  {
				  rplidar_info_message * msg = edm.messages()->push<rplidar_info_message>();
				  if (msg != NULL)
					  msg->device_info = *(dynamic_cast<info_data_packet*>(m_data_packets[m_current_type]));
				  _reset_state();
				  return;
			  }
			  case (Scan):
			  {
				  if (!m_rec_start_scan)
				  {
					  m_rec_start_scan = _check_packet_for_scan_start();
					  if (m_rec_start_scan)
					  {
						  m_current_scan[m_scan_index] = *(dynamic_cast<scan_data_packet*>(m_data_packets[m_current_type]));
						  ++m_scan_index;
					  }
				  }
				  else
				  {
					  m_current_scan[m_scan_index] = *(dynamic_cast<scan_data_packet*>(m_data_packets[m_current_type]));
					  ++m_scan_index;
					  
					  if (m_scan_index == 360)
					  {
						  m_scan_index = 0;
						  m_rec_start_scan = false;
					  }
				  }
				  return;
			  }
			}
		}	
	}
}

void edrplidar_system::_reset_state()
{
	m_current_type = None;
	m_rec_index = 0;
	m_rec_descript = false;
	m_rec_start_scan = false;
	m_scan_index = 0;
}

bool edrplidar_system::_check_packet_for_scan_start()
{
	char first_byte = (*m_data_packets[m_current_type])[0];
	return ( (first_byte & 0x01) == 0x01);
}

void wait_ready_callback::exec()
{
	timer->stop();
}
