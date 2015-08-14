#include <edrplidar_system.h>
#include <edmctrl.h>
#include <edplsystem.h>
#include <stdlib.h>
#include <signal.h>
#include <ednavsystem.h>
#include <edmessage_dispatch.h>
#include <edlogging_system.h>
#include <edmessage.h>
#include <edtimer.h>
#include <edcallback.h>
#include <edcomm_system.h>
#include <edimu_system.h>

void handle_ctrlc(int32_t sig)
{
	edmctrl::quit();
	exit(1);
}

int32_t main(int32_t argc, char * argv[])
{
	int32_t port = 0;
	for (int32_t i = 0; i < argc; ++i)
	{
		std::string curarg(argv[i]);
		if (curarg.find("-port:") == 0)
			port = std::stoi(curarg.substr(6));
	}
	
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = handle_ctrlc;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);
	
    //edm.add_sys<edrplidar_system>();
    //edm.add_sys<edpl_system>();
    //edm.add_sys<ednav_system>();
    //edm.add_sys<edlogging_system>();
    //edm.add_sys<edcomm_system>()->set_port(port);
	edm.add_sys<edimu_system>();
	
	edm.start();
    edm.init();

    while (edm.running())
		edm.update();
	
    return 0;
}
