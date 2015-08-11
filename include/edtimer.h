#ifndef EDTIMER_H
#define EDTIMER_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <chrono>

struct edtimer_callback;

//! class edtimer 
/*! 
  This class keeps track of time allowing you to start, stop, and continue the timer.
  It also has a "dt" functionality to allow you to see how much time has elapsed since
  the last update call. This could be useful for various things.

  You can also set up the timer to execute a callback every so often (every so many milliseconds) or you can
  set it to execute once after some delay.
 */
class edtimer
{
  public:
	
	enum cb_mode {
		no_shot,
		single_shot,
		continous_shot
	};
	
	edtimer();
	~edtimer();
	
	void start();

	void update();

	edtimer_callback * callback();

	cb_mode callback_mode();

	double callback_delay();

	void cont();

	void stop();

	void set_callback(edtimer_callback * cb);

	void set_callback_mode(cb_mode mode);

	void set_callback_delay(double ms);
	
	double dt();

	bool running();

	double elapsed();

  private:
	typedef std::chrono::high_resolution_clock HighResClock;

	HighResClock m_clk;
	bool m_running;
	
	HighResClock::time_point m_start_time;
	HighResClock::time_point m_previous_frame_time;
	HighResClock::time_point m_last_cbexec_time;

	double m_elapsed_ms;
	double m_dt_ms;
	double m_cumilitive_time_ms;
	double m_cbdelay_ms;

	edtimer_callback * m_cb;
	cb_mode m_cmode;
};


#endif
