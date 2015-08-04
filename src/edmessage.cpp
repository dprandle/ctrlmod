#include <edmessage.h>
#include <edutility.h>

rplidar_error_message::rplidar_error_message()
{
    zero_buf(message, 100);
}
