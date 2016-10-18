#include <eduart.h>
#include <iostream>
#include <edmessage_dispatch.h>
#include <edrplidar_system.h>
#include <edutility.h>
#include <edmctrl.h>
//#include <mraa/uart.hpp>
#include <edtimer.h>
#include <unistd.h>
#include <termios.h>
//#include <mraa/mraa_internal_types.h>

edrplidar_system::edrplidar_system():
    m_uart(new eduart(eduart::Uart1)),
    m_wait_timer(new edtimer()),
    m_timeout_timer(new edtimer()),
    m_error_timer(new edtimer()),
    m_rec_index(0),
    m_scan_index(0),
    m_rec_descript(false),
    m_rec_start_scan(false),
    m_current_type(None)
{
    m_data_packets.resize(4);
    m_desc_packets.resize(3);
    m_data_packets[Scan] = new scan_data_packet();
    m_desc_packets[Scan] = new scan_descriptor();
    m_data_packets[Info] = new info_data_packet();
    m_desc_packets[Info] = new device_info_descriptor();
    m_data_packets[Health] = new health_data_packet();
    m_desc_packets[Health] = new device_health_descriptor();
    m_data_packets[Reset] = new firmware_data_packet();
    m_current_scan.resize(360);
    firm_message = "RP LIDAR System.\r\nFirmware Ver 1.15, HW Ver 0\r\nMode: 0\r\n";
}

edrplidar_system::~edrplidar_system()
{
    delete m_uart;
    delete m_wait_timer;
    delete m_timeout_timer;
    delete m_error_timer;
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
    edm.message_dispatch()->register_listener<rplidar_request>(this);
    m_uart->set_format(eduart::d8, eduart::None, eduart::One);
    m_uart->set_baud(eduart::b115200);

	if (!m_uart->start())
		cprint("Error starting uart");
	
    m_wait_timer->set_callback(new wait_ready_callback());
    m_wait_timer->set_callback_mode(edtimer::single_shot);
    m_error_timer->set_callback(new wait_ready_callback());
    m_error_timer->set_callback_mode(edtimer::single_shot);
    m_error_timer->set_callback_delay(5000);
}

void edrplidar_system::release()
{
}

void edrplidar_system::update()
{
    static uint8_t readBuf[256];
    m_timeout_timer->update();
    m_wait_timer->update();
    m_error_timer->update();

    if (m_timeout_timer->running() && m_timeout_timer->elapsed() > 4000)
    {
        cprint("Timeout for last request sending reset request");
        reset();
        return;
    }

    if (m_current_type == Reset && !m_error_timer->running())
    {
        log_message("Could not detect firmware packet.. resetting state");
        _reset_state();
    }

    int32_t size = m_uart->read(readBuf, 256);
    if (size > 0)
    {		
        m_timeout_timer->stop();
        for (int32_t i = 0; i < size; ++i)
            _handle_byte(readBuf[i]);
    }
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
    return false;
}


bool edrplidar_system::startScan()
{
    if (m_current_type != None)
        return false;

    cprint("Start scan command issued");
    // set descriptor to scan response
    m_current_type = Scan;

    m_wait_timer->set_callback_delay(1.0);
    m_wait_timer->start();
    m_timeout_timer->start();
    m_error_timer->start();
    start_scan_request sr;
	m_uart->write(sr.data,2, scan_descriptor::Size());
    return true;
}

bool edrplidar_system::forceScan()
{
    if (m_current_type != None)
        return false;

    cprint("Force scan command issued");
    // set descriptor to scan response
    m_current_type = Scan;

    m_wait_timer->set_callback_delay(1.0);
    m_wait_timer->start();
    m_timeout_timer->start();
    m_error_timer->start();
	force_scan_request sr;
	m_uart->write(sr.data, 2, scan_descriptor::Size());
    return true;
}

bool edrplidar_system::stopScan()
{
    cprint("Stop scan command issued");
    _reset_state();
    m_wait_timer->set_callback_delay(50.0);
    m_wait_timer->start();
    stop_scan_request sr;
	m_uart->write(sr.data,2);
    return true;
}

bool edrplidar_system::reset()
{
    cprint("Reset command issued");
    _reset_state();
    m_timeout_timer->stop();
    reset_request rr;
    m_wait_timer->set_callback_delay(20.0);
    m_wait_timer->start();
    m_current_type = Reset;
    m_rec_descript = true; // No descriptor for reset
	m_uart->write(rr.data, 2, firmware_data_packet::Size());
    m_error_timer->start();
    return true;
}

bool edrplidar_system::requestInfo()
{
    if (m_current_type != None)
        return true;

    cprint("Request device information command issued");
    // set descriptor to info response
    m_current_type = Info;
    m_wait_timer->set_callback_delay(1.0);
    m_wait_timer->start();
    m_timeout_timer->start();
    m_error_timer->start();
    device_info_request sr;
	m_uart->write(sr.data,2, device_info_descriptor::Size() + info_data_packet::Size());
    return true;
}

bool edrplidar_system::requestHealth()
{
    if (m_current_type != None)
        return true;

    cprint("Request device health command issued");
    // set descriptor to info response
    m_current_type = Health;
    m_wait_timer->set_callback_delay(1.0);
    m_wait_timer->start();
    m_timeout_timer->start();
    m_error_timer->start();
    device_health_request sr;
	m_uart->write(sr.data,2, device_health_descriptor::Size() + health_data_packet::Size());
    return true;
}

void edrplidar_system::_handle_byte(uint8_t byte)
{	
    if (m_current_type == None)
        return;

    if (!m_rec_descript) // receiving descriptor packet
    {
        if ( (*m_desc_packets[m_current_type])[m_rec_index] != byte)
        {
            m_rec_index = 0;
            if (!m_error_timer->running())
            {
                rplidar_error_message * msg = edm.message_dispatch()->push<rplidar_error_message>();
                std::string message = "Error in communicating with rplidar - packet descriptor not received";
                if (msg != NULL)
                    copy_buf((uint8_t*)message.c_str(), msg->message, message.size());
                _reset_state();
            }
        }
        ++m_rec_index;

        if (m_rec_index == m_desc_packets[m_current_type]->size())
        {
            log_message("Descriptor packet received for " + m_desc_packets[m_current_type]->type());
            m_rec_index = 0;
            m_rec_descript = true;
            m_rec_start_scan = false;
            m_error_timer->stop();
        }
    }
    else // receiving data
    {
        if (m_current_type == Reset)
        {
            if (byte != firm_message[m_rec_index])
                m_rec_index = 0; // start over
            if (!m_error_timer->running())
            {
                rplidar_error_message * msg = edm.message_dispatch()->push<rplidar_error_message>();
                std::string message = "Error in communicating with rplidar - firmware packet not found after reset";
                if (msg != NULL)
                    copy_buf((uint8_t*)message.c_str(), msg->message, message.size());
                _reset_state();
            }
        }

        (*m_data_packets[m_current_type])[m_rec_index] = byte;
        ++m_rec_index;

        if (m_rec_index == m_data_packets[m_current_type]->size())
        {
            m_rec_index = 0;

            switch(m_current_type)
            {
            case (Health):
            {
                rplidar_health_message * msg = edm.message_dispatch()->push<rplidar_health_message>();
                if (msg != NULL)
                {
                    copy_buf(m_data_packets[m_current_type]->dataptr(),
                             msg->device_health.data,
                             m_data_packets[m_current_type]->size());
                }
                _reset_state();
                return;
            }
            case (Info):
            {
                rplidar_info_message * msg = edm.message_dispatch()->push<rplidar_info_message>();
                if (msg != NULL)
                {
                    copy_buf(m_data_packets[m_current_type]->dataptr(),
                             msg->device_info.data,
                             m_data_packets[m_current_type]->size());
                }
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
                        copy_buf(m_data_packets[m_current_type]->dataptr(),
                                 m_current_scan[m_scan_index].data,
                                 m_current_scan[m_scan_index].size());
                        ++m_scan_index;
                    }
                }
                else
                {
                    copy_buf(m_data_packets[m_current_type]->dataptr(),
                             m_current_scan[m_scan_index].data,
                             m_current_scan[m_scan_index].size());
                    ++m_scan_index;

                    if (m_scan_index == 360)
                    {
                        rplidar_scan_message * msg = edm.message_dispatch()->push<rplidar_scan_message>();
                        if (msg != NULL)
                        {
                            msg->millis_timestamp = edm.sys_timer()->elapsed();
                            for(uint32_t i = 0; i < 360; ++i)
                            {
                                copy_buf(m_current_scan[i].data,
                                         msg->scan_data.data[i].data,
                                         msg->scan_data.data[i].size());
                            }
                        }
                        m_scan_index = 0;
                        m_rec_start_scan = false;
                    }
                }
                return;
            }
            case (Reset):
            {
                rplidar_firmware_message * msg = edm.message_dispatch()->push<rplidar_firmware_message>();
                if (msg != NULL)
                {
                    copy_buf(m_data_packets[m_current_type]->dataptr(),
                             msg->device_firmware.data,
                             m_data_packets[m_current_type]->size());
                }
                _reset_state();
                return;
            }
            case (None):
            {
                log_message("Shouldnt be here AGRAG!");
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
    m_error_timer->stop();
    m_scan_index = 0;
}

bool edrplidar_system::_check_packet_for_scan_start()
{
    uint8_t first_byte = (*m_data_packets[m_current_type])[0];
    return ( (first_byte & 0x01) == 0x01);
}
