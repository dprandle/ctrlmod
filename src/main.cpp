#include <iostream>
#include <stdio.h>
#include <edxv11system.h>
#include <edutility.h>
#include <exception>
#include <edmctrl.h>
#include <edplsystem.h>

int main()
{
    edm.add_sys<edxv11_system>();
	edm.add_sys<edpl_system>();
	
    edm.start();
    while (edm.running())
    {
        edm.update();
    }
    return 0;
}
