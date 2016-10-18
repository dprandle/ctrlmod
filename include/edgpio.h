/*!
  \file   edgpio.h
  \author Daniel <dprandle@dprandle-CZ-17>
  \date   Tue Jul  7 09:19:49 2015
*/


#ifndef EDGPIO_H
#define EDGPIO_H

#define GPIO_14 14
#define GPIO_15 15
#define GPIO_48 48
#define GPIO_49 49

#include <pthread.h>
#include <atomic>
#include <string>

enum gpio_dir {
	gpio_dir_out,
	gpio_dir_in,
	gpio_dir_out_high,
	gpio_dir_out_low
};

enum gpio_output_mode
{
	gpio_strong,
	gpio_pullup,
	gpio_pulldown,
	gpio_hiz
};

enum gpio_isr_edge
{
	gpio_edge_none,
	gpio_edge_both,
	gpio_edge_rising,
	gpio_edge_falling
};

enum gpio_error_code
{
	gpio_no_error,
	gpio_thread_start_error=1,
	gpio_thread_join_error=2,
	gpio_unexport_error=4,
	gpio_export_error=8,
	gpio_direction_error=16,
	gpio_edge_error=32,
	gpio_pin_error=64
};

struct gpio_error_state
{
	gpio_error_state():
		gp_code(0),
		errno_code(0)
	{}
	
	int gp_code;
	int errno_code;
};

class edgpio
{
  public:

	edgpio(int pin);
	~edgpio();
	
	int set_direction(gpio_dir dir);
	int direction();

	int set_isr(gpio_isr_edge edge, void (*func)(void *), void * param);

	int read_pin();

	int write_pin(int val);
	
	void update();

	int pin_num();

	gpio_error_state get_and_clear_error();

	static std::string error_string(int gp_err);

  private:

	static void * _thread_exec(void * param);
	void _exec();
	
	void (*m_fnc)(void *);
	void * m_fnc_param;
	int m_pin;
	gpio_error_state m_err;
	std::atomic_flag m_run_isr;
	std::atomic_flag m_thread_running;
	pthread_t m_isr_thread;
	
};

#endif
