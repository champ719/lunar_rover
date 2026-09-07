#pragma once

#include "mathetics.hpp"
#include <array>


struct pid_cfg_s
{
	const float _kp;
	const float _ki;
	const float _kd;
	const float _deadzone;
	const float _integral_limit;
	const float _output_limit;
};

//template <typename T>
//concept pid = (std::constructible_from<T, const pid_cfg_t&>
//	|| std::constructible_from<T, const pid_cfg_t&, const pid_cfg_t&>)
//	&& requires(T t)
//	{
//		t.output;
//		t.Update(float{}, float{});
//		t.Clear();
//	};

struct pid_s
{
	float kp;
	float ki;
	float kd;
	float deadzone;
	float error;
	float error_prev;
	float integral_limit;
	float integral;
	float output_limit;
	float output;
	explicit pid_s(const pid_cfg_s& _pid_cfg)
		: kp(_pid_cfg._kp), ki(_pid_cfg._ki), kd(_pid_cfg._kd), deadzone(_pid_cfg._deadzone), error(0), error_prev(0),
		integral_limit(_pid_cfg._integral_limit), integral(0), output_limit(_pid_cfg._output_limit), output(0)
	{}
	void Update(const float _target, const float _feedback)
	{
		error_prev = error;
		error = _target - _feedback;
		if (error < deadzone && error > -deadzone) return;
		integral = Limit(integral + error, integral_limit, -integral_limit);
		output = (error - error_prev) * kd + integral * ki + error * kp;
		output = Limit(output, output_limit, -output_limit);
	}
	void Clear()
	{
		error = 0;
		error_prev = 0;
		integral = 0;
		output = 0;
	}
};

struct pid_diff_ahead_s
{
	float kp;
	float ki;
	float kd;
	float deadzone;
	float error;
	float feedback_prev;
	float integral_limit;
	float integral;
	float output_limit;
	float output;
	explicit pid_diff_ahead_s(const pid_cfg_s& _pid_cfg)
		: kp(_pid_cfg._kp), ki(_pid_cfg._ki), kd(_pid_cfg._kd), deadzone(_pid_cfg._deadzone), error(0), feedback_prev(0),
		integral_limit(_pid_cfg._integral_limit), integral(0), output_limit(_pid_cfg._output_limit), output(0)
	{}
	void Update(const float _target, const float _feedback)
	{
		error = _target - _feedback;
		if (error < deadzone && error > -deadzone)
		{
			feedback_prev = _feedback;  // doubt its necessity
			return;
		}
		const float derivative = (_feedback - feedback_prev) * kd;
		feedback_prev = _feedback;
		integral = Limit(integral + error, integral_limit, -integral_limit);
		output = Limit(error * kp + integral * ki - derivative, output_limit, -output_limit);
	}
	void Clear()
	{
		error = 0;
		feedback_prev = 0;
		integral = 0;
		output = 0;
	}
};
