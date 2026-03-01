#include "plugin_exports.hpp"

#include "../../trajectories/lissajous.hpp"

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

bool parse_validate_lissajous(int argc, char *argv[], TrajectoryCommand &out, char *error, size_t error_len)
{
	if (argc != 8) {
		snprintf(error, error_len,
			 "Usage: mc_raptor intref set lissajous <A> <B> <C> <fa> <fb> <fc> <duration> <ramp>");
		return false;
	}

	TrajectoryCommand command {};
	command.id = TRAJECTORY_ID_LISSAJOUS;
	set_command_name(command, "lissajous");
	command.arg_count = 8;

	for (int i = 0; i < 8; ++i) {
		if (!parse_float_arg(argv[i], command.args[i])) {
			snprintf(error, error_len, "invalid lissajous argument %d: %s", i + 1, argv[i]);
			return false;
		}
	}

	if (command.args[6] <= 0.0f) {
		snprintf(error, error_len, "duration must be > 0");
		return false;
	}

	if (command.args[7] < 0.0f) {
		snprintf(error, error_len, "ramp must be >= 0");
		return false;
	}

	out = command;
	return true;
}

Setpoint evaluate_lissajous(float time_s, const TrajectoryCommand &command)
{
	LissajousParameters params {};
	params.A = command.args[0];
	params.B = command.args[1];
	params.C = command.args[2];
	params.a = command.args[3];
	params.b = command.args[4];
	params.c = command.args[5];
	params.duration = command.args[6];
	params.ramp_duration = command.args[7];
	return lissajous(time_s, params);
}

} // namespace

const TrajectoryPluginDescriptor g_lissajous_plugin {
	TRAJECTORY_ID_LISSAJOUS,
	"lissajous",
	"Lissajous internal trajectory",
	"mc_raptor intref set lissajous <A> <B> <C> <fa> <fb> <fc> <duration> <ramp>",
	parse_validate_lissajous,
	evaluate_lissajous
};

} // namespace mc_raptor_intref
