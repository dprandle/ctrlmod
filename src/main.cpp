#include <iostream>
#include <stdio.h>
#include <edxv11system.h>
#include <edutility.h>
#include <exception>
#include <edmctrl.h>

int main()
{
    edm.add_sys<edxv11_system>();
    edm.start();
	int x;
	std::cin >> x;
    while (edm.running())
    {
        //edm.update();
    }
    return 0;
}
