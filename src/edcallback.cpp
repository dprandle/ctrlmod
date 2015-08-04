#include <edcallback.h>
#include <edtimer.h>

void wait_ready_callback::exec()
{
	timer->stop();
}
