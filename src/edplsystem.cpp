/*!
  \file   edplsystem.cpp
  \author Daniel <dprandle@dprandle-CZ-17>
  \date   Tue Jul  7 09:32:32 2015
  
  \brief  Definitions for system
  
  
*/


#include <edplsystem.h>
#include <edutility.h>
#include <mraa.hpp>
#include <iostream>
#include <edtimer.h>

edpl_system::edpl_system():
	m_gpio0(NULL),
	m_gpio1(NULL),
	m_gpio2(NULL),
	m_gpio3(NULL),
	m_timer(new edtimer())
{
	lt = 0x00;
	val = 0x00;
}

edpl_system::~edpl_system()
{
	m_gpio0->write(0);
	delete m_gpio0;
	delete m_gpio1;
	delete m_gpio2;
	delete m_gpio3;
	delete m_timer;
}

void edpl_system::init()
{
	m_gpio0 = new mraa::Gpio(GPIO_14);
	if (m_gpio0 == NULL)
		log_message("Unable to initialize gpio pin 0");

	m_gpio1 = new mraa::Gpio(GPIO_15);
	if (m_gpio1 == NULL)
		log_message("Unable to initialize gpio pin 1");

	m_gpio2 = new mraa::Gpio(GPIO_48);
	if (m_gpio2 == NULL)
		log_message("Unable to initialize gpio pin 2");
 
	m_gpio3 = new mraa::Gpio(GPIO_49);
	if (m_gpio3 == NULL)
		log_message("Unable to initialize gpio pin 3");

	mraa_result_t res;
	
	res = m_gpio0->dir(mraa::DIR_OUT);
	if (res != MRAA_SUCCESS)
		mraa::printError(res);
	res = m_gpio1->dir(mraa::DIR_IN);
	if (res != MRAA_SUCCESS)
		mraa::printError(res);

	m_gpio1->isr(mraa::EDGE_RISING,gpio14_isr,this);
	m_gpio0->write(val);
	// res = m_gpio2->dir(mraa::DIR_OUT);
	// if (res != MRAA_SUCCESS)
	// 	mraa::printError(res);
	// res = m_gpio3->dir(mraa::DIR_OUT);
	// if (res != MRAA_SUCCESS)
	// 	mraa::printError(res);
	m_timer->start();
}

void edpl_system::process(edmessage * msg)
{
	
}

void edpl_system::update()
{
	m_timer->update();
	if (lt == 0x01)
	{
		if (m_timer->running())
		{
			m_timer->stop();
			std::cout << "Elapsed time: " << m_timer->elapsed() << " ms" << std::endl;
		}
		else
		{
			m_timer->start();
			std::cout << "Starting timer";
		}
	}
}

std::string edpl_system::typestr()
{
	return TypeString();
}

void gpio14_isr(void * edplsys)
{
	edpl_system * sys = static_cast<edpl_system*>(edplsys);
	sys->lt = 0x01;
}
