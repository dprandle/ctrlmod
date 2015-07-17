#include <edrplidar_system.h>
#include <edmctrl.h>
#include <edplsystem.h>
#include <stdlib.h>
#include <signal.h>
#include <ednavsystem.h>
#include <edmsghandler.h>
#include <edmessage.h>

void handle_ctrlc(int sig)
{
	edmctrl::quit();
	exit(1);
}

int main()
{
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = handle_ctrlc;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);
	
    edm.add_sys<edrplidar_system>();
	edm.add_sys<edpl_system>();
	edm.add_sys<ednav_system>();
    edm.init();
	edm.start();

	rplidar_request * msg1 = edm.messages()->push<rplidar_request>();
	msg1->r_type = rplidar_request::Reset;

	rplidar_request * msg = edm.messages()->push<rplidar_request>();
	msg->r_type = rplidar_request::InfoReq;

    while (edm.running())
    {
		edm.update();
    }
	
    return 0;
}
