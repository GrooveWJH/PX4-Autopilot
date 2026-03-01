#include "plugin_exports.hpp"

#include "../../trajectories/circle.hpp"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

namespace mc_raptor_intref
{

namespace
{

bool parse_float_arg(const char *arg, float &value)
{
	char *endptr = nullptr;
	value = strtof(arg, &endptr);
	return endptr != arg && *endptr == '\0' && PX4_ISFINITE(value);
}

bool parse_validate_circle(int argc, char *argv[], TrajectoryCommand &out, char *error, size_t error_len)
{
	if (argc != 3) {
		snprintf(error, error_len, "Usage: mc_raptor intref set circle <radius_m> <speed_mps> <ramp_s>");
		return false;
	}

	TrajectoryCommand command {};
	command.id = TRAJECTORY_ID_CIRCLE;
	set_command_name(command, "circle");
	command.arg_count = 3;

	for (int i = 0; i < 3; ++i) {
		if (!parse_float_arg(argv[i], command.args[i])) {
			snprintf(error, error_len, "invalid circle argument %d: %s", i + 1, argv[i]);
			return false;
		}
	}

	const float radius = command.args[0];
	const float speed = command.args[1];
	const float ramp_duration = command.args[2];
	const float speed_abs = fabsf(speed);
	constexpr float MIN_RADIUS = 0.1f;
	constexpr float MIN_SPEED_ABS = 0.05f;
	constexpr float MAX_SPEED_ABS = 12.0f;
	constexpr float MAX_CENTRIPETAL_ACCEL = 6.0f;

	if (radius < MIN_RADIUS) {
		snprintf(error, error_len, "radius must be >= %.2f m", (double)MIN_RADIUS);
		return false;
	}

	if (speed_abs < MIN_SPEED_ABS) {
		snprintf(error, error_len, "abs(speed) must be >= %.2f m/s", (double)MIN_SPEED_ABS);
		return false;
	}

	if (speed_abs > MAX_SPEED_ABS) {
		snprintf(error, error_len, "abs(speed) must be <= %.2f m/s", (double)MAX_SPEED_ABS);
		return false;
	}

	if (ramp_duration < 0.0f) {
		snprintf(error, error_len, "ramp duration must be >= 0 s");
		return false;
	}

	const float centripetal_accel = (speed * speed) / radius;

	if (centripetal_accel > MAX_CENTRIPETAL_ACCEL) {
		snprintf(error, error_len, "speed^2/radius must be <= %.2f m/s^2", (double)MAX_CENTRIPETAL_ACCEL);
		return false;
	}

	out = command;
	return true;
}

Setpoint evaluate_circle(float time_s, const TrajectoryCommand &command)
{
	CircleParameters params {};
	params.radius = command.args[0];
	params.speed = command.args[1];
	params.ramp_duration = command.args[2];
	return circle(time_s, params);
}

} // namespace

const TrajectoryPluginDescriptor g_circle_plugin {
	TRAJECTORY_ID_CIRCLE,
	"circle",
	"Horizontal circle; starts with forward-aligned tangent yaw",
	"mc_raptor intref set circle <radius_m> <speed_mps> <ramp_s>",
	parse_validate_circle,
	evaluate_circle
};

} // namespace mc_raptor_intref
