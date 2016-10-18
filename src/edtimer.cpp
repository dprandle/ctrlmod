#include <edutility.h>
#include <edtimer.h>
#include <iostream>
#include <string>
#include <edcallback.h>
#include <edutility.h>

edtimer::edtimer():
    m_running(false),
	m_start(),
	m_prev(),
	m_cur(),
	m_last_cb(),
	m_cb(nullptr),
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
	clock_gettime(CLOCK_MONOTONIC, &m_start);
	m_prev = m_start;
	m_last_cb = m_start;
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
	return m_cb_delay;
}

void edtimer::set_callback_mode(cb_mode mode)
{
	m_cmode = mode;
	m_last_cb = m_prev;
}

void edtimer::set_callback_delay(double secs)
{
	m_cb_delay = secs;
}

void edtimer::update()
{
	if (!m_running)
		return;

	m_prev = m_cur;
	clock_gettime(CLOCK_MONOTONIC, &m_cur);

	double elpsed = elapsed();
	double cb_elapsed = elapsed_since_callback();
	if (((m_cmode == single_shot) && (m_last_cb.tv_nsec == m_start.tv_nsec && m_last_cb.tv_sec == m_last_cb.tv_sec) && (elpsed >= m_cb_delay)) ||
		((m_cmode == continous_shot) && (cb_elapsed >= m_cb_delay)))
	{
		m_last_cb = m_cur;
		if (m_cb == nullptr)
			cprint("No callback set for timer yet callback mode is enabled");
		else
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
	return double(m_cur.tv_sec - m_prev.tv_sec + (m_cur.tv_nsec - m_prev.tv_nsec) / 1000000000.0);
}

double edtimer::elapsed()
{
	return double(m_cur.tv_sec - m_start.tv_sec + (m_cur.tv_nsec - m_start.tv_nsec) / 1000000000.0);
}

double edtimer::elapsed_since_callback()
{
	return double(m_cur.tv_sec - m_last_cb.tv_sec + (m_cur.tv_nsec - m_last_cb.tv_nsec) / 1000000000.0);
}
