#ifndef EDTIMER_H
#define EDTIMER_H

#include <time.h>
#include <inttypes.h>

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

	void stop();

	void set_callback(edtimer_callback * cb);

	void set_callback_mode(cb_mode mode);

	void set_callback_delay(double ms);
	
	double dt();

	bool running();

	double elapsed();

	double elapsed_since_callback();

  private:
	bool m_running;

	timespec m_start;
	timespec m_prev;
	timespec m_cur;
	timespec m_last_cb;

	double m_cb_delay;
	edtimer_callback * m_cb;
	cb_mode m_cmode;
};


#endif
