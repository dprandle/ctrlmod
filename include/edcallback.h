#ifndef EDCALLBACK_H
#define EDCALLBACK_H

struct edcallback
{
	virtual ~edcallback() {}
	virtual void exec()=0;
};

class edtimer;

struct edtimer_callback : public edcallback
{
	edtimer * timer;
};

struct wait_ready_callback : public edtimer_callback
{
	void exec();
};

#endif
