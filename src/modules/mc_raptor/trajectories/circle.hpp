#pragma once

#include "trajectory.hpp"

#include <math.h>

struct CircleParameters {
	float radius = 1.0f;
	float speed = 1.0f;
	float ramp_duration = 3.0f;
};

inline Setpoint circle(float time, const CircleParameters &params)
{
	const float t = fmaxf(time, 0.0f);
	const float radius = fmaxf(params.radius, 0.001f);
	const float speed_sign = params.speed >= 0.0f ? 1.0f : -1.0f;
	const float speed_abs = fabsf(params.speed);

	float speed_abs_cmd = speed_abs;
	float s = 0.0f;

	if (params.ramp_duration > 0.0f) {
		if (t < params.ramp_duration) {
			speed_abs_cmd = speed_abs * t / params.ramp_duration;
			s = 0.5f * speed_abs * t * t / params.ramp_duration;

		} else {
			s = 0.5f * speed_abs * params.ramp_duration + speed_abs * (t - params.ramp_duration);
		}

	} else {
		s = speed_abs * t;
	}

	const float theta = speed_sign * s / radius;

	Setpoint setpoint{};
	setpoint.position[0] = radius * (cosf(theta) - 1.0f);
	setpoint.position[1] = radius * sinf(theta);
	setpoint.position[2] = 0.0f;

	setpoint.linear_velocity[0] = -sinf(theta) * speed_sign * speed_abs_cmd;
	setpoint.linear_velocity[1] = cosf(theta) * speed_sign * speed_abs_cmd;
	setpoint.linear_velocity[2] = 0.0f;

	if (speed_abs_cmd > 1e-6f) {
		setpoint.yaw = atan2f(setpoint.linear_velocity[1], setpoint.linear_velocity[0]);

	} else {
		setpoint.yaw = 0.0f;
	}

	setpoint.yaw_rate = speed_sign * speed_abs_cmd / radius;

	return setpoint;
}
