#include <edutility.h>
#include <edtimer.h>
#include <iostream>
#include <string>
#include <edcallback.h>

edtimer::edtimer():
    m_running(false),
	m_elapsed_ms(0),
	m_dt_ms(0),
	m_cumilitive_time_ms(0),
	m_cbdelay_ms(0),
	m_cb(NULL),
    m_cmode(no_shot)
{}

edtimer::~edtimer()
{
	if (m_cb != NULL)
		delete m_cb;
}

void edtimer::start()
{
	m_running = true;
	m_start_time = HighResClock::now();
	m_previous_frame_time = m_start_time;
	m_last_cbexec_time = m_start_time;
	
	m_elapsed_ms = 0;
	m_dt_ms = 0;
	m_cumilitive_time_ms = 0;
}

void edtimer::cont()
{
	m_running = true;
	m_start_time = HighResClock::now();
	m_previous_frame_time = m_start_time;
	m_last_cbexec_time = m_start_time;
	
	m_cumilitive_time_ms += m_elapsed_ms;
	m_elapsed_ms = 0;
	m_dt_ms = 0;
}

edtimer_callback * edtimer::callback()
{
	return m_cb;
}

edtimer::cb_mode edtimer::callback_mode()
{
	return m_cmode;
}

void edtimer::set_callback(edtimer_callback * cb)
{
	if (m_cb != NULL)
		delete m_cb;
	m_cb = cb;
	if (m_cb != NULL)
		m_cb->timer = this;
}

double edtimer::callback_delay()
{
	return m_cbdelay_ms;
}

void edtimer::set_callback_mode(cb_mode mode)
{
	m_cmode = mode;
	m_last_cbexec_time = m_previous_frame_time;
}

void edtimer::set_callback_delay(double ms)
{
	m_cbdelay_ms = ms;
}

void edtimer::update()
{
	if (!m_running)
		return;

	HighResClock::time_point current = HighResClock::now();
	m_dt_ms = double((current - m_previous_frame_time).count()) / 1000000.0;
	m_elapsed_ms = double((current - m_start_time).count()) / 1000000.0;

	double cb_elapsed_ms = double((current - m_last_cbexec_time).count()) / 1000000.0;
	if (((m_cmode == single_shot) && (m_last_cbexec_time == m_start_time) && (m_elapsed_ms >= m_cbdelay_ms)) ||
		((m_cmode == continous_shot) && (cb_elapsed_ms >= m_cbdelay_ms)))
	{
		m_last_cbexec_time = current;

		if (m_cb == NULL)
			log_message("No callback set for timer yet callback mode is enabled");
		else
			m_cb->exec(); // This must come after - stop calls update()
	}
	m_previous_frame_time = current;
}

void edtimer::stop()
{
	update();
	m_running = false;
}

bool edtimer::running()
{
	return m_running;
}

double edtimer::dt()
{
	return m_dt_ms;
}

double edtimer::elapsed()
{
	return m_elapsed_ms;	
}
