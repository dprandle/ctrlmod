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

#define MAX_PWM_MEASUREMENTS 128

#include <pthread.h>
#include <atomic>
#include <string>

#define FILE_BUF_SZ 40

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

struct pwm_measurement
{
    int8_t cur_edge; // This will be 0 for rising and 1 for falling - useful mainly for when the edge mode is both
    int8_t prev_edge; // What period are we measuring - falling to falling, rising to rising, or both
    double seconds; // the time from the last edge to this one
};

class edgpio
{
  public:

	edgpio(int pin);
	~edgpio();
	
	int set_direction(gpio_dir dir);
	int direction();

    int set_isr(gpio_isr_edge edge, void (*func)(void *, pwm_measurement), void * param);

	int read_pin();

	int write_pin(int val);
	
	void update();

	int pin_num();

	gpio_error_state get_and_clear_error();

	static std::string error_string(int gp_err);

  private:

	static void * _thread_exec(void * param);
	void _exec();
	
    void (*m_fnc)(void *, pwm_measurement);
	void * m_fnc_param;
	gpio_error_state m_err;

    std::atomic_int m_pin;
    std::atomic_flag m_thread_running = ATOMIC_FLAG_INIT;
	pthread_t m_isr_thread;

    // only used within separate thread... no mutex necessary
    int8_t m_prev_edge;
    int8_t m_cur_edge;

    pwm_measurement m_tmp_measurements[MAX_PWM_MEASUREMENTS];

    pthread_mutex_t distance_meas_lock;
    uint32_t m_cur_meas_index;
    pwm_measurement m_pwm_measurements[MAX_PWM_MEASUREMENTS];
	
};

#endif
