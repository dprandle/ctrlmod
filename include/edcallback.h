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

#endif
