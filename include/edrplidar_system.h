#ifndef EDLPLIDAR_SYSTEM_H
#define EDLPLIDAR_SYSTEM_H

#include <edsystem.h>
#include <vector>
#include <edutility.h>
#include <edrplidar_packets.h>
#include <edcallback.h>
#include <mraa/uart.h>
#include <edmessage.h>

#define XV_BAUD 115200


class edtimer;
class eduart;

class edrplidar_system : public edsystem
{
public:

    enum ExchangeType
    {
        Scan,
        Info,
        Health,
        Reset,
        None
    };

    edrplidar_system();
    ~edrplidar_system();

    void init();

    void release();
    
    bool process(edmessage * msg);

    void update();

    std::string typestr() {return TypeString();}

    static std::string TypeString() {return "edrplidar_system";}

protected:

    bool startScan();

    bool forceScan();

    bool stopScan();

    bool reset();

    bool requestInfo();

    bool requestHealth();

private:

    void _handle_byte(char byte);
    void _reset_state();
    bool _check_packet_for_scan_start();

    eduart * m_uart;
    edtimer * m_wait_timer;
    edtimer * m_timeout_timer;
    edtimer * m_error_timer;

    uint m_rec_index;
    uint m_scan_index;
    bool m_rec_descript;
    bool m_rec_start_scan;

    ExchangeType m_current_type;

    std::vector<descriptor_packet*> m_desc_packets;
    std::vector<data_packet*> m_data_packets;
    std::vector<scan_data_packet> m_current_scan;
    std::string firm_message;
};

#endif
