#include <edutility.h>
#include <edtimer.h>
#include <iostream>
#include <string>
#include <edcallback.h>

edtimer::edtimer():
	m_running(false),
	prev_time(0),
	ms_last(0),
	ms(0),
	m_cbdelay(0),
	m_cb(NULL),
	last_exec(0),
	m_cmode(no_shot)
{}

edtimer::~edtimer()
{
	if (m_cb != NULL)
		delete m_cb;
}

void edtimer::start()
{
	gettimeofday(&t, NULL);
	m_running = true;
	ms = 0;
	ms_last = 0;
	last_exec = 0;
	prev_time = 0;
}

void edtimer::cont()
{
	prev_time += ms;
	ms = 0;
	ms_last = 0;
	gettimeofday(&t, NULL);
	m_running = true;
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
	return m_cbdelay;
}

void edtimer::set_callback_mode(cb_mode mode)
{
	m_cmode = mode;
	last_exec = ms;
}

void edtimer::set_callback_delay(double ms)
{
	m_cbdelay = ms;
}

void edtimer::update()
{
	if (!m_running)
		return;
	
	timeval e_tm;
	long s, us;
	
	gettimeofday(&e_tm, NULL);

	s = e_tm.tv_sec - t.tv_sec;
	us = e_tm.tv_usec - t.tv_usec;

	ms_last = ms;
	ms = s*1000 + double(us)/1000.0;
	if (((m_cmode == single_shot) && (last_exec == 0.0) && (ms >= m_cbdelay)) ||
		((m_cmode == continous_shot) && (ms - last_exec >= m_cbdelay)))
	{
		if (m_cb == NULL)
		{
			std::string message = "No callback set for timer yet callback mode is enabled";
			log_message(message);
			return;
		}
		last_exec = ms;
		std::cout << "ms: " << ms << "last_exec: " << last_exec << std::endl;
		m_cb->exec(); // This must come after - stop calls update()
	}
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
	return ms - ms_last;
}

double edtimer::elapsed()
{
	return prev_time + ms;
}
