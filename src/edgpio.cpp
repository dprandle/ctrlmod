#include <edgpio.h>
#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <poll.h>
#include <edutility.h>
#include <edtimer.h>

edgpio::edgpio(int pin):
	m_fnc(nullptr),
	m_fnc_param(nullptr),
	m_err(),
    m_pin(pin),
	m_thread_running(),
    m_isr_thread(),
    m_prev_edge(0),
    m_cur_edge(0),
    m_cur_meas_index(0)
{
    pthread_mutex_init(&distance_meas_lock, nullptr);
    char buffer[8];
    int pind = m_pin;
    int n = sprintf (buffer, "%d", pind);
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd == -1)
    {
        m_err.errno_code = errno;
        m_err.gp_code |= gpio_export_error;
    }
    else
    {
        write(fd, buffer, n);
        close(fd);
    }
}

edgpio::~edgpio()
{
    pthread_mutex_destroy(&distance_meas_lock);

    // Close the isr thread if needed
    if (m_thread_running.test_and_set())
    {
        m_thread_running.clear();
        m_err.errno_code = pthread_join(m_isr_thread, nullptr);
        if (m_err.errno_code)
        {
            // error joining - set error code and return
            m_err.gp_code |= gpio_thread_join_error;
        }

    }

    char buffer[8];
    int pin = m_pin;
    int n = sprintf (buffer, "%d", pin);
    int fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (fd == -1)
    {
        m_err.errno_code = errno;
        m_err.gp_code |= gpio_unexport_error;
    }
    else
    {
        write(fd, buffer, n);
        close(fd);
    }
}

gpio_error_state edgpio::get_and_clear_error()
{
	gpio_error_state tmp = m_err;
	m_err.errno_code = 0;
	m_err.gp_code = gpio_no_error;
	return tmp;
}
	
int edgpio::set_direction(gpio_dir dir)
{
    char buf[FILE_BUF_SZ];
    int pin = m_pin;
    sprintf(buf, "/sys/class/gpio/gpio%d/direction", pin);
	int fd = open(buf, O_WRONLY);
	if (fd == -1)
	{
		m_err.errno_code = errno;
		m_err.gp_code |= gpio_direction_error;
		return -1;
	}
	else
	{
		int tmp = 0;
		switch (dir)
		{
		  case(gpio_dir_out):
			  tmp = write(fd, "out", 3);
			  break;
		  case (gpio_dir_in):
			  tmp = write(fd, "in", 2);
			  break;
		  case (gpio_dir_out_high):
			  tmp = write(fd, "high", 4);
			  break;
		  case (gpio_dir_out_low):
			  tmp = write(fd, "low", 3);
			  break;
		}
		close(fd);
		if (tmp == -1)
		{
			m_err.errno_code = errno;
			m_err.gp_code |= gpio_direction_error;
			return -1;
		}
	}
	return 0;
}

int edgpio::direction()
{
    char buf[FILE_BUF_SZ];
	char r_buf[5];
    int pin = m_pin;
    sprintf(buf, "/sys/class/gpio/gpio%d/direction", pin);
	int fd = open(buf, O_RDONLY);
	if (fd == -1)
	{
		m_err.errno_code = errno;
		m_err.gp_code |= gpio_direction_error;
	}
	else
	{
		int bytes = read(fd, r_buf, 5);
		close(fd);
		if (bytes == -1)
		{
			m_err.errno_code = errno;
			m_err.gp_code |= gpio_direction_error;
		}
		if (strncmp(r_buf, "in", 2) == 0) // for clarity == 0
			return gpio_dir_in;
		if (strncmp(r_buf, "out", 3) == 0) // for clarity == 0
			return gpio_dir_out;
	}
	return -1;
}

int edgpio::set_isr(gpio_isr_edge edge, void (*func)(void *, pwm_measurement), void * param)
{
	// Close the isr thread if needed - no matter what after this the thread should be dead
	if (m_thread_running.test_and_set())
	{
		m_thread_running.clear();
		m_err.errno_code = pthread_join(m_isr_thread, nullptr);
		if (m_err.errno_code)
		{
			// error joining - set error code and return
			m_err.gp_code |= gpio_thread_join_error;
			return -1;
		}
	}
	m_thread_running.clear(); // need to set false now because of previous if test

	if (edge == gpio_edge_none)
	{
		m_fnc = func;
		m_fnc_param = param;
		return 0;
	}

	if (func == nullptr) // no isr if func is null
		edge = gpio_edge_none;
	
    char buf[FILE_BUF_SZ];
    int pin = m_pin;
    sprintf(buf, "/sys/class/gpio/gpio%d/edge", pin);
	int fd = open(buf, O_WRONLY);
	if (fd == -1)
	{
		m_err.errno_code = errno;
		m_err.gp_code |= gpio_edge_error;
		return -1;
	}
	else
	{
		int tmp = 0;
        m_fnc = func;
        m_fnc_param = param;
		switch (edge)
		{
		  case(gpio_edge_none): // basically disable isr and don't start the thread!
			  tmp = write(fd, "none", 4);
			  if (tmp == -1)
			  {
				  m_err.errno_code = errno;
				  m_err.gp_code = gpio_edge_error;
				  close(fd);
				  return -1;
			  }
			  close(fd);
			  return 0;
          case (gpio_edge_both): // trigger isr for both edges
			  tmp = write(fd, "both", 4);
			  break;
          case (gpio_edge_rising): // trigger isr for rising edges only
			  tmp = write(fd, "rising", 6);
			  break;
          case (gpio_edge_falling): // trigger isr for falling edges only
			  tmp = write(fd, "falling", 7);
			  break;
		}
		close(fd);
		if (tmp == -1)
		{
			m_err.errno_code = errno;
			m_err.gp_code |= gpio_edge_error;
			return -1;
		}
	}

	// start thread if isr is not null and set the edge mode to create interrupt
	if (func != nullptr)
	{
        m_thread_running.test_and_set();
		m_err.errno_code = pthread_create(&m_isr_thread, nullptr, edgpio::_thread_exec, (void*)this);
		if (m_err.errno_code)
		{
			// error occured
			m_err.gp_code |= gpio_thread_start_error;
            m_thread_running.clear();
			return -1;
		}
	}
	return 0;
}

void edgpio::update()
{
    int count = 0;
    pthread_mutex_lock(&distance_meas_lock);
    while (m_cur_meas_index > 0)
    {
        m_tmp_measurements[count] = m_pwm_measurements[m_cur_meas_index-1];
        --m_cur_meas_index;
        ++count;
    }
    pthread_mutex_unlock(&distance_meas_lock);

    for (int i = 0; i < count; ++i)
    {
        if (m_fnc != nullptr)
            m_fnc(m_fnc_param, m_tmp_measurements[i]);
        else
            log_message("edgpio::update - No isr found - make sure one has been assigned");
    }
}

int edgpio::pin_num()
{
	return m_pin;
}

void * edgpio::_thread_exec(void * param)
{
	edgpio * _this = static_cast<edgpio*>(param);
    int pin = _this->m_pin;
    log_message("edgpio::_thread_exec Starting gpio thread on pin " + std::to_string(pin));
	_this->_exec();
	return nullptr;
}

int edgpio::read_pin()
{
    char buf[FILE_BUF_SZ];
	char r_val;
    int pin = m_pin;
    sprintf(buf, "/sys/class/gpio/gpio%d/value", pin);
	int fd = open(buf, O_RDONLY);
	if (fd == -1)
	{
		m_err.errno_code = errno;
		m_err.gp_code |= gpio_pin_error;
		return -1;
	}
	else
	{
		int bytes = read(fd, &r_val, 1);
		close(fd);
		if (bytes == -1)
		{
			m_err.errno_code = errno;
			m_err.gp_code |= gpio_pin_error;
			return -1;
		}
		return int(r_val - '0');
	}
}

int edgpio::write_pin(int val)
{
	if (direction() == gpio_dir_in)
		return 0;
	
    char buf[FILE_BUF_SZ];
	char w_val = char('0' + val);
    int pin = m_pin;
    sprintf(buf, "/sys/class/gpio/gpio%d/value", pin);
	int fd = open(buf, O_WRONLY);
	if (fd == -1)
	{
		m_err.errno_code = errno;
		m_err.gp_code |= gpio_pin_error;
		return -1;
	}
	else
	{
		int bytes = write(fd, &w_val, 1);
		close(fd);
		if (bytes == -1)
		{
			m_err.errno_code = errno;
			m_err.gp_code |= gpio_pin_error;
			return -1;
		}
	}
	return 0;
}


void edgpio::_exec()
{
    char buf[FILE_BUF_SZ];
    char r_val;
    int pin = m_pin;
    sprintf(buf, "/sys/class/gpio/gpio%d/value", pin);
    pollfd polldes;
    polldes.fd = open(buf, O_RDONLY);
    polldes.events = POLLPRI | POLLERR;

    if (polldes.fd == -1)
    {
        int err = errno;
        log_message("Error: " + std::to_string(err));
        log_message("edgpio::_exec Error starting thread as fd is invalid");
        m_thread_running.clear();
    }

    edtimer tm;
    tm.start();
    double prev_time[2] = {};
    double cur_time[2] = {};
    while (m_thread_running.test_and_set())
    {
        if (poll(&polldes, 1, -1) == 1 && polldes.revents & POLLPRI)
        {
            // Stop the pwm timer
            tm.stop();

            // Get the current value of pwm stuff
            lseek(polldes.fd, 0, SEEK_SET);
            read(polldes.fd, &r_val, 1);
            //lseek(polldes.fd, 0, SEEK_SET);
            m_prev_edge = m_cur_edge;
            m_cur_edge = r_val - '0';

            cur_time[m_cur_edge] = tm.elapsed() * 100000;

            if (cur_time[m_cur_edge] <= prev_time[m_cur_edge] + 5.0 && cur_time[m_cur_edge] >= prev_time[m_cur_edge] - 5.0)
            {
                // Lock the pwm_measurements mutex and save a measurement for reading from the other thread
                pthread_mutex_lock(&distance_meas_lock);
                pwm_measurement & cur_m = m_pwm_measurements[m_cur_meas_index];
                cur_m.cur_edge = m_cur_edge;
                cur_m.prev_edge = m_prev_edge;
                cur_m.seconds = tm.elapsed();
                ++m_cur_meas_index;
                pthread_mutex_unlock(&distance_meas_lock);
            }
            prev_time[m_cur_edge] = cur_time[m_cur_edge];

            // Start the pwm timer again
            tm.start();
        }
    }
    close(polldes.fd);
    log_message("edgpio::_exec Ending gpio thread on pin " + std::to_string(pin));
}

std::string edgpio::error_string(int gp_err)
{
	if (gp_err == 0)
		return std::string("No error");
	std::string ret;
	if ( (gp_err & gpio_thread_start_error) == gpio_thread_start_error)
		ret += "GPIO Thread Start Error";
	else if ( (gp_err & gpio_thread_join_error) == gpio_thread_join_error)
		ret += " | GPIO Thread Join Error";
	else if ( (gp_err & gpio_unexport_error) == gpio_unexport_error)
		ret += " | GPIO Unexport Error";
	else if ( (gp_err & gpio_export_error) == gpio_export_error)
		ret += " | GPIO Export Error";
	else if ( (gp_err & gpio_direction_error) == gpio_direction_error)
		ret += " | GPIO Direction Error";
	else if ( (gp_err & gpio_edge_error) == gpio_edge_error)
		ret += " | GPIO Edge Error";
	else if ( (gp_err & gpio_pin_error) == gpio_pin_error)
		ret += " | GPIO Pin Error";
	else
		ret = "Unexpected error - gp_err not zero yet value not recognized";
	if (ret[0] == ' ')
		ret.erase(ret.begin(), ret.begin() + 3); // get rid of " | " if its at the start of string
	return ret;
}
