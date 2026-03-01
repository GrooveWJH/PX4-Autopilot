#pragma once

#include "../trajectories/trajectory.hpp"

#include <px4_platform_common/defines.h>

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

namespace mc_raptor_intref
{

static constexpr uint8_t TRAJECTORY_ID_NONE = 0;
static constexpr uint8_t TRAJECTORY_ID_LISSAJOUS = 1;
static constexpr uint8_t TRAJECTORY_ID_CIRCLE = 2;

static constexpr int MAX_TRAJECTORY_ARGS = 8;
static constexpr int ACTIVE_TRAJECTORY_NAME_LEN = 24;

enum class InternalReferenceConfigured : uint8_t {
	NONE = 0,
	LISSAJOUS = 1,
	CIRCLE = 2
};

enum class ReferenceMode : uint8_t {
	EXTREF = 0,
	INTREF = 1,
	HOLD = 2
};

enum class ReferenceSource : uint8_t {
	EXTERNAL = 0,
	INTERNAL_LISSAJOUS = 1,
	INTERNAL_CIRCLE = 2,
	HOLD = 3,
	INTREF_CUSTOM = 4
};

struct TrajectoryCommand {
	uint8_t id = TRAJECTORY_ID_NONE;
	char name[ACTIVE_TRAJECTORY_NAME_LEN] {};
	uint8_t arg_count = 0;
	float args[MAX_TRAJECTORY_ARGS] {};
};

using ParseValidateFn = bool (*)(int argc, char *argv[], TrajectoryCommand &out, char *error, size_t error_len);
using EvaluateFn = Setpoint (*)(float time_s, const TrajectoryCommand &command);

struct TrajectoryPluginDescriptor {
	uint8_t id;
	const char *name;
	const char *description;
	const char *usage;
	ParseValidateFn parse_validate;
	EvaluateFn evaluate;
};

inline void set_command_name(TrajectoryCommand &command, const char *name)
{
	if (name == nullptr) {
		command.name[0] = '\0';
		return;
	}

	strncpy(command.name, name, sizeof(command.name) - 1);
	command.name[sizeof(command.name) - 1] = '\0';
}

inline bool same_command(const TrajectoryCommand &a, const TrajectoryCommand &b)
{
	if (a.id != b.id || a.arg_count != b.arg_count) {
		return false;
	}

	if (strncmp(a.name, b.name, sizeof(a.name)) != 0) {
		return false;
	}

	for (uint8_t i = 0; i < a.arg_count; ++i) {
		if (fabsf(a.args[i] - b.args[i]) > 1e-6f) {
			return false;
		}
	}

	return true;
}

inline const char *configured_mode_name(InternalReferenceConfigured configured)
{
	switch (configured) {
	case InternalReferenceConfigured::NONE:
		return "None";

	case InternalReferenceConfigured::LISSAJOUS:
		return "Lissajous";

	case InternalReferenceConfigured::CIRCLE:
		return "Circle";

	default:
		return "Unknown";
	}
}

inline const char *reference_mode_name(ReferenceMode reference_mode)
{
	switch (reference_mode) {
	case ReferenceMode::EXTREF:
		return "ExtRef";

	case ReferenceMode::INTREF:
		return "IntRef";

	case ReferenceMode::HOLD:
		return "Hold";

	default:
		return "Unknown";
	}
}

inline const char *reference_source_name(ReferenceSource source)
{
	switch (source) {
	case ReferenceSource::EXTERNAL:
		return "External";

	case ReferenceSource::INTERNAL_LISSAJOUS:
		return "InternalLissajous";

	case ReferenceSource::INTERNAL_CIRCLE:
		return "InternalCircle";

	case ReferenceSource::HOLD:
		return "Hold";

	case ReferenceSource::INTREF_CUSTOM:
		return "InternalCustom";

	default:
		return "Unknown";
	}
}

} // namespace mc_raptor_intref
