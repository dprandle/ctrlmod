/*!
  \file   ednavsystem.cpp
  \author Daniel <dprandle@dprandle-CZ-17>
  \date   Fri Jul 10 10:44:40 2015
  
  \brief  Definition file for navigation system  
*/

#include <edutility.h>
#include <ednavsystem.h>
#include <edmctrl.h>
#include <edmessage_dispatch.h>
#include <edmessage.h>
#include <edtimer.h>
#include <edi2c.h>

ednav_system::ednav_system():
	m_nav_timer(new edtimer()),
	m_i2c(NULL),
	m_G_mult(1.0),
	m_bias_threshold_dist(0.0),
	m_threshold_dropout(false)
{}

ednav_system::~ednav_system()
{
	delete m_nav_timer;
}

void ednav_system::init()
{
	edmessage_dispatch * mh = edm.message_dispatch();
	mh->register_listener<pulsed_light_message>(this);
	mh->register_listener<rplidar_scan_message>(this);
	mh->register_listener<nav_system_request>(this);
	mh->register_listener<nav_message>(this);

	m_i2c = new edi2c();
	m_i2c->set_target_address(ARDUINO_ADDRESS);
	if (!m_i2c->start())
	 	log_message("ednav_system::init - Could not start i2c");
	else
		log_message("ednav_system::init - Successfully initialized i2c");

	
	m_nav_pid.set_gain(vec3(1.0,0.0,0.0));	
	m_nav_pid.set_range(
		vec4(LIDAR_DIST_DIFF_MIN, LIDAR_DIST_DIFF_MIN, YAW_ANGLE_DIFF_MIN, ALT_DIF_MIN),
		vec4(LIDAR_DIST_DIFF_MAX, LIDAR_DIST_DIFF_MAX, YAW_ANGLE_DIFF_MAX, ALT_DIF_MAX));
	
	// Set up timer for 20 Hz transmission
	m_nav_timer->set_callback_mode(edtimer::continous_shot);
	m_nav_timer->set_callback_delay(100.0); // 50 ms = 20 Hz
	m_nav_timer->set_callback(new instruction_callback(this));
	m_nav_timer->start();
}

void ednav_system::release()
{
	delete m_i2c;
	m_i2c = NULL;
}

bool ednav_system::process(edmessage * msg)
{
	rplidar_scan_message * rmsg = dynamic_cast<rplidar_scan_message*>(msg);
	pulsed_light_message * pmsg = dynamic_cast<pulsed_light_message*>(msg);
	nav_system_request * navmsg = dynamic_cast<nav_system_request*>(msg);
	nav_message * nmsg = dynamic_cast<nav_message*>(msg);

	if (pmsg != NULL)
		return _process_pulse_light(pmsg);

	if (rmsg != NULL)
		return _process_scan(rmsg);

	if (navmsg != NULL)
	{
		m_nav_pid.set_gain(navmsg->pid);
		m_nav_pid.set_ramp_limit(navmsg->ramp_limit);
		m_nav_pid.enable_anti_reset_windup(navmsg->anti_reset_winding);
		m_nav_pid.enable_complex_derivitive(navmsg->complex_der);
		m_G_mult = navmsg->g_factor;
		m_bias_vec = navmsg->bias_vec;
		m_bias_threshold_dist = navmsg->bias_threshold_dist;
		m_threshold_dropout = navmsg->threshold_dropout;
	}

	if (nmsg != NULL)
		return _process_nav_message(nmsg);
	
    return true;
}

void ednav_system::update()
{
	m_nav_timer->update();
}

double ednav_system::interval()
{
	return m_nav_timer->callback_delay();
}

void ednav_system::set_interval(double ms)
{
	m_nav_timer->set_callback_delay(ms);
}

bool ednav_system::_process_pulse_light(pulsed_light_message * msg)
{
	m_pl_dist_to_center = (msg->distance2 - msg->distance1) / 2.0;
	return true;
}

bool ednav_system::_process_scan(rplidar_scan_message * msg)
{
	vec2 tmp = m_resultant_vec;
	vec2 to_add;
	double shortest_dist = 10000;
	
	m_resultant_vec = 0;
	for (uint32_t i = 0; i < 360; ++i)
	{
		scan_data_packet * curpacket = &msg->scan_data.data[i];

		uint32_t angle = ((uint32_t)(curpacket->angle6to0_C) >> 1) & 0x7F;
		angle |= ((uint32_t)(curpacket->angle14to7) << 7) & 0x7F80;

		uint32_t distance = (uint32_t)(curpacket->distance7to0) & 0xFF;
		distance |= ((uint32_t)(curpacket->distance15to8) << 8) & 0xFF00;

		double ang_t = double(angle) / 64.0;
		double dist_t = double(distance) / 4.0;
		
        if (dist_t != 0)
        {
			if (dist_t < shortest_dist)
				shortest_dist = dist_t;
				
			to_add.setFromPolar(dist_t, ang_t).normalize();
			m_resultant_vec += m_G_mult * G_CONST * to_add / (dist_t * dist_t);
        }
	}
	
	if (shortest_dist > m_bias_threshold_dist)
	{
		if (m_threshold_dropout)
			m_resultant_vec = 0;
		else
			m_resultant_vec += m_bias_vec;
	}

	m_angle_to_last_vec = m_resultant_vec.angleTo(tmp);
	return true;
}

bool ednav_system::_process_nav_message(nav_message * nmsg)
{
	m_i2c->write_word(nmsg->pitch);
	m_i2c->write_word(nmsg->roll);
	m_i2c->write_word(nmsg->yaw);
	m_i2c->write_word(nmsg->throttle);
	return true;
}

void instruction_callback::exec()
{
	double dt = m_nav_sys->m_nav_timer->callback_delay() / 1000.0;

	vec4 current_input(m_nav_sys->m_resultant_vec, m_nav_sys->m_angle_to_last_vec, m_nav_sys->m_pl_dist_to_center);
	
	vec4 delta = m_nav_sys->m_nav_pid.loop(current_input, dt);
	
	nav_message * nmsg = edm.message_dispatch()->push<nav_message>();
	if (nmsg != NULL)
	{
		nmsg->pitch = int16_t(delta.x * (PITCH_MAX - PITCH_MIN) / (LIDAR_DIST_DIFF_MAX - LIDAR_DIST_DIFF_MIN));
		nmsg->roll = int16_t(delta.y * (ROLL_MAX - ROLL_MIN) / (LIDAR_DIST_DIFF_MAX - LIDAR_DIST_DIFF_MIN));
		nmsg->yaw = int16_t(delta.z * (YAW_MAX - YAW_MIN) / (YAW_ANGLE_DIFF_MAX - YAW_ANGLE_DIFF_MIN));		
		nmsg->throttle = int16_t(delta.w * (THROTTLE_MAX - THROTTLE_MIN) / (ALT_DIF_MAX - ALT_DIF_MIN));
		nmsg->rvec_raw[0] = m_nav_sys->m_resultant_vec.y;
		nmsg->rvec_raw[1] = m_nav_sys->m_resultant_vec.x;
		nmsg->rvec_corrected[0] = delta.y;
		nmsg->rvec_corrected[1] = delta.x;
	}
}
