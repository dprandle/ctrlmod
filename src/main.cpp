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

#include <sys/types.h>
#include <unistd.h>

void sig_exit_handler(int signum)
{
    std::string msg("Cought signal ");
    switch (signum)
    {
    case(SIGTERM):
        msg += "sigterm";
        break;
    case(SIGINT):
        msg += "sigint";
        break;
    case(SIGKILL):
        msg += "sigkill";
        break;
    default:
        log_message("Cought signal that shouldnt have caught");
        return;
    }
    log_message(msg);
    int pid = getpid();
    log_message("Stopping ctrlmod with pid " + std::to_string(pid) + " at " + timestamp());
    edm.quit();
    cprint_flush();
    exit(0);
}

int32_t main(int32_t argc, char * argv[])
{
    int pid = getpid();
    log_message("Starting ctrlmod with pid " + std::to_string(pid) + " at " + timestamp());
    signal(SIGTERM, sig_exit_handler);
    signal(SIGINT, sig_exit_handler);
    signal(SIGKILL, sig_exit_handler);

    int32_t port = 0;
	for (int32_t i = 0; i < argc; ++i)
	{
		std::string curarg(argv[i]);
		if (curarg.find("-port:") == 0)
			port = std::stoi(curarg.substr(6));
	}
	
    edm.add_sys<ednav_system>();
    edm.add_sys<edrplidar_system>();
    edm.add_sys<edpl_system>();
    edm.add_sys<edlogging_system>();
    edm.add_sys<edcomm_system>()->set_port(port);
	
	edm.start();
    edm.init();

    rplidar_request * req = edm.message_dispatch()->push<rplidar_request>();
    req->r_type = rplidar_request::HealthReq;

    req = edm.message_dispatch()->push<rplidar_request>();
    req->r_type = rplidar_request::StartScan;

    while (edm.running())
		edm.update();

    pid = getpid();
    log_message("Stopping ctrlmod with pid " + std::to_string(pid) + " at " + timestamp());
	
    return 0;
}
