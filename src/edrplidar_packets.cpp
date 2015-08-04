#include <edrplidar_packets.h>
#include <sstream>
#include <iomanip>

scan_data_packet::scan_data_packet()
{
    for (unsigned int i = 0; i < size(); ++i)
        data[i] = 0;
}

health_data_packet::health_data_packet()
{
    for (unsigned int i = 0; i < size(); ++i)
        data[i] = 0;
}

info_data_packet::info_data_packet()
{
    for (unsigned int i = 0; i < size(); ++i)
        data[i] = 0;
}

firmware_data_packet::firmware_data_packet()
{
    for (unsigned int i = 0; i < size(); ++i)
        data[i] = 0;
}

complete_scan_data_packet::complete_scan_data_packet()
{
    // do nothing
}

std::string complete_scan_data_packet::toString()
{
    std::string str;
    str.reserve(360*100);
    for (unsigned int i = 0; i < 360; ++i)
        str += data[i].toString();
    return str;
}

std::string scan_data_packet::toString()
{
    std::ostringstream ss;
    unsigned short int angle = ((unsigned short int)(angle6to0_C) >> 1) & 0x7F;
    angle |= ((unsigned int)(angle14to7) << 7) & 0x7F80;

    unsigned short int distance = (unsigned short int)(distance7to0) & 0xFF;
    distance |= ((unsigned int)(distance15to8) << 8) & 0xFF00;

    unsigned short int ang = angle/64, dist = distance/4;
    ss << "\nAngle: " << ang;
    ss << "\nDistance: " << dist << "\n";
    return ss.str();
}

std::string health_data_packet::toString()
{
    std::ostringstream ss;
    ss << "Device Health\n";
    ss << "Status: ";
    if (status == 0x00)
            ss << "Good\n";
    else if (status == 0x01)
            ss << "Warning\n";
    else
            ss << "Error\n";

    short int error_code = (((short int)(error_code15to8)) << 8);
    error_code |= error_code7to0;

    ss << "Error Code: ";
    if (error_code == 0)
            ss << "None";
    else
            ss << error_code;

    return ss.str();
}

std::string info_data_packet::toString()
{    
    std::ostringstream ss;
    ss << "Device Information\n";
    ss << "Model ID: " << int(model) << "\n";
    ss << "Firware Version: " << int(firmware_major) << "." << int(firmware_minor) << "\n";
    ss << "Hardware Version: " << int(hardware) << "\n";
    ss << "Serial Number: ";
    for (int i = 15; i >= 0; --i)
        ss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>((unsigned char)(serialnumber[i]));
    return ss.str();
}

std::string firmware_data_packet::toString()
{
    std::string message = "Device Firmware\n";
    message.resize(size());
    for (unsigned int i = 0; i < size(); ++i)
            message[i] = (*this)[i];
    return message;
}
