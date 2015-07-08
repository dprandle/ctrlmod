#include <edtimer.h>

edtimer::edtimer():
	m_running(false),
	prev_time(0),
	ms_last(0),
	ms(0)
{}

edtimer::~edtimer()
{}

void edtimer::start()
{
	gettimeofday(&t, NULL);
	m_running = true;
	ms = 0;
	ms_last = 0;
	prev_time = 0;
}

void edtimer::cont()
{
	prev_time += ms;
	ms = 0;
	ms_last = 0;
	gettimeofday(&t, NULL);
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
