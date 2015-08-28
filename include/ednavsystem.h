/*!
  \file   ednavsystem.h
  \author Daniel <dprandle@dprandle-CZ-17>
  \date   Fri Jul 10 10:34:08 2015
  
  \brief  Navigation system header file
  
  
*/

#ifndef EDNAVSYSTEM_H
#define EDNAVSYSTEM_H

#define ARDUINO_ADDRESS 0x04

#define ALT_DIF_MIN -100		/*!< cm */
#define ALT_DIF_MAX 100			/*!< cm */
#define LIDAR_DIST_DIFF_MIN -5000 /*!< cm */
#define LIDAR_DIST_DIFF_MAX 5000  /*!< cm */
#define YAW_ANGLE_DIFF_MIN -45
#define YAW_ANGLE_DIFF_MAX 45
#define THROTTLE_MIN -500		/*!< scaled min */
#define THROTTLE_MAX 500
#define PITCH_MIN -500
#define PITCH_MAX 500
#define ROLL_MIN -500
#define ROLL_MAX 500
#define YAW_MIN -500
#define YAW_MAX 500
#define G_CONST 5000000

#include <edsystem.h>
#include <nsmath.h>
#include <edpid_controller.h>

namespace mraa
{
struct I2c;
}

struct edmessage;
struct pulsed_light_message;
struct rplidar_scan_message;
struct nav_message;
class edtimer;
class edi2c;

class ednav_system : public edsystem
{
	friend struct instruction_callback;
  public:
    ednav_system();
    virtual ~ednav_system();
	
    virtual void init();
	virtual void release();
    virtual bool process(edmessage * msg);
    virtual void update();

	double interval();
	void set_interval(double ms);

    virtual std::string typestr() {return TypeString();}
   	static std::string TypeString() {return "ednav_system";}

  private:
	bool _process_pulse_light(pulsed_light_message * msg);
	bool _process_nav_message(nav_message * nmsg);
	bool _process_scan(rplidar_scan_message * msg);
	
	edtimer * m_nav_timer;
	edi2c * m_i2c;
	
	edpid_controller<vec4> m_nav_pid;
	
	vec2 m_resultant_vec;
	double m_angle_to_last_vec;
	
	int32_t m_scaled_throttle;
	double m_pl_dist_to_center;

	double m_G_mult;
	double m_bias_threshold_dist;
	vec2 m_bias_vec;
	bool m_threshold_dropout;
};

#include <edcallback.h>
struct instruction_callback : public edtimer_callback
{
	instruction_callback(ednav_system * system):
		m_nav_sys(system)
	{}
	
	void exec();
	ednav_system * m_nav_sys;
};
#endif
