#ifndef EDPID_CONTROLLER_H
#define EDPID_CONTROLLER_H

#include <nsmath.h>

template<class T>
class edpid_controller
{
  public:

	struct output_range
	{
		output_range(const T & min_=0, const T & max_=0):
			min(min_),
			max(max_)
		{}
		
		T min;
		T max;
	};
	
	edpid_controller();

	~edpid_controller();

	void enable_complex_derivitive(bool enable);

	void enable_anti_reset_windup(bool enable);

	bool anti_reset_windup();

	bool complex_derivitive();

	const vec3 & gain();

	double offset();

	const output_range & range();

	double ramp_limit();
	
	void set_gain(const vec3 & pid_);
	
	void set_gain(double P, double I, double D);

	void set_gain_P(double P);
	
	void set_gain_I(double I);
	
	void set_gain_D(double D);
	
	void set_offset(double offset_);

	void set_ramp_limit(double percent);

	void set_range(const T & min, const T & max);
	
	void set_target(const T & target_);

	T loop(const T & input, double dt);
	
	const T & target();
		
  private:
	vec3 m_gain;
	double m_offset;

	bool m_complex_deriv;
	bool m_anti_reset_windup;

	output_range m_range;
	T m_target;
	T m_prev_error[3];
	T m_integral_err;
	
	double m_ramp_limit;
};

template<class T>
edpid_controller<T>::edpid_controller():
	m_offset(0.0),
	m_complex_deriv(true),
	m_anti_reset_windup(true),
	m_target(0),
	m_integral_err(0),
	m_ramp_limit(1)
{
	for (uint8_t i = 0; i < 3; ++i)
		m_prev_error[i] = 0;
}

template<class T>
edpid_controller<T>::~edpid_controller()
{}

template<class T>
double edpid_controller<T>::ramp_limit()
{
	return m_ramp_limit * 100.0;
}

template<class T>
void edpid_controller<T>::set_ramp_limit(double percent)
{
	if (percent > 100)
		percent = 100;
	if (percent < 0)
		percent = 0;
	
	m_ramp_limit = percent / 100.0;
}

template<class T>
void edpid_controller<T>::enable_complex_derivitive(bool enable)
{
	m_complex_deriv = enable;
}

template<class T>
void edpid_controller<T>::enable_anti_reset_windup(bool enable)
{
	m_anti_reset_windup = enable;
}

template<class T>
bool edpid_controller<T>::anti_reset_windup()
{
	return m_anti_reset_windup;
}

template<class T>
bool edpid_controller<T>::complex_derivitive()
{
	return m_complex_deriv;
}

template<class T>
void edpid_controller<T>::set_gain(const vec3 & pid_)
{
	m_gain = pid_;
}

template<class T>
const typename edpid_controller<T>::output_range & edpid_controller<T>::range()
{
	return m_range;
}

template<class T>
void edpid_controller<T>::set_range(const T & min, const T & max)
{
	m_range.max = max;
	m_range.min = min;
}

template<class T>
void edpid_controller<T>::set_gain(double P, double I, double D)
{
	m_gain.set(P,I,D);
}

template<class T>
void edpid_controller<T>::set_gain_P(double P)
{
	m_gain.P = P;
}

template<class T>
void edpid_controller<T>::set_gain_I(double I)
{
	m_gain.I = I;
}

template<class T>
void edpid_controller<T>::set_gain_D(double D)
{
	m_gain.D = D;
}

template<class T>
void edpid_controller<T>::set_offset(double offset_)
{
	m_offset = offset_;
}

template<class T>
double edpid_controller<T>::offset()
{
	return m_offset;
}

template<class T>
const vec3 & edpid_controller<T>::gain()
{
	return m_gain;
}

template<class T>
void edpid_controller<T>::set_target(const T & target_)
{
	m_target = target_;
}

template<class T>
const T & edpid_controller<T>::target()
{
	return m_target;
}

template<class T>
T edpid_controller<T>::loop(const T & input, double dt)
{
	T current_error = m_target - input;

	// Limit the error if an output range is set (default ramping is one, or the entire range)
	if (m_range.min != m_range.max)
	{
		T max_err = (m_range.max - m_range.min) * m_ramp_limit;
		if (current_error > max_err)
			current_error = max_err;
	}

	// Integral portion
	m_integral_err += current_error * dt;

	// Derivitive portion
	T deriv;
	if (m_complex_deriv)
		deriv = (current_error + 3.0 * (m_prev_error[2] - m_prev_error[1]) - m_prev_error[0]) / 6.0;
	else
		deriv = (current_error - m_prev_error[2]) / dt;

	// Shift errors to the left and set last error
	for (uint8_t i = 0; i < 2; ++i)
		m_prev_error[i] = m_prev_error[i+1];
	m_prev_error[2] = current_error;

	T ret = (m_gain.P * current_error + m_gain.I * m_integral_err + m_gain.D * deriv);

	// limit the output if a valid range is set
	// also, if anti-reset-windup is set then undo the earlier summation to the integral term
	if (m_range.min != m_range.max)
	{
		if (ret > m_range.max)
		{
			ret = m_range.max;
			if (m_anti_reset_windup)
				m_integral_err -= current_error * dt;
		}
		if (ret < m_range.min)
		{
			ret = m_range.min;
			if (m_anti_reset_windup)
				m_integral_err -= current_error * dt;
		}
	}

	return ret;
}

#endif
