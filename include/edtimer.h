#ifndef EDTIMER_H
#define EDTIMER_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

class edtimer
{
  public:
	edtimer();
	~edtimer();

	void start();
	void update();
	void cont();
	void stop();

	double dt();
	bool running();
	double elapsed();

  private:
	bool m_running;
	double prev_time;
	double ms_last;
	double ms;
	timeval t;
};


#endif
