#pragma once

#include "intref_registry.hpp"

#include <drivers/drv_hrt.h>
#include <uORB/topics/trajectory_setpoint.h>

#include <math.h>

namespace mc_raptor_intref
{

struct IntRefStatusSnapshot {
	InternalReferenceConfigured configured_mode = InternalReferenceConfigured::NONE;
	ReferenceMode reference_mode = ReferenceMode::EXTREF;
	ReferenceSource reference_source = ReferenceSource::EXTERNAL;
	uint8_t active_trajectory_id = TRAJECTORY_ID_NONE;
	char active_trajectory_name[ACTIVE_TRAJECTORY_NAME_LEN] {};
	bool hold_active = false;
	bool hold_pending_capture = false;
	float hold_position[3] {0.0f, 0.0f, 0.0f};
	float hold_yaw = 0.0f;
	bool transition_active = false;
	float transition_progress = 0.0f;
	float transition_remaining_s = 0.0f;
};

struct IntRefStepInput {
	hrt_abstime now = 0;
	bool vehicle_active = false;
	bool just_activated = false;
	float position[3] {0.0f, 0.0f, 0.0f};
	float linear_velocity[3] {0.0f, 0.0f, 0.0f};
	float attitude_q[4] {1.0f, 0.0f, 0.0f, 0.0f};
};

struct IntRefStepResult {
	ReferenceSource reference_source = ReferenceSource::EXTERNAL;
	bool produced_setpoint = false;
	trajectory_setpoint_s setpoint {};
	bool internal_reference_valid = false;
	float internal_reference_position[3] {NAN, NAN, NAN};
	float internal_reference_linear_velocity[3] {NAN, NAN, NAN};
	bool plugin_error = false;
	char plugin_error_message[96] {};
};

class IntRefRuntimeManager
{
public:
	IntRefRuntimeManager();

	void reset();

	void setConfiguredMode(InternalReferenceConfigured mode);
	InternalReferenceConfigured configuredMode() const;

	bool setTrajectoryCommand(const TrajectoryCommand &command, char *error, size_t error_len);
	bool setReferenceMode(ReferenceMode mode, char *error, size_t error_len);
	ReferenceMode referenceMode() const;
	bool canUseIntref() const;
	void setTransitionConfig(float transition_time_s, float max_yaw_rate_rad_s);

	IntRefStepResult step(const IntRefStepInput &input);
	IntRefStatusSnapshot statusSnapshot() const;

	TrajectoryCommand configuredLissajousCommand() const;
	TrajectoryCommand configuredCircleCommand() const;
	bool hasSelectedInternalTrajectory() const;
	TrajectoryCommand selectedInternalTrajectory() const;

private:
	const TrajectoryCommand *selectTrajectoryCommand() const;
	ReferenceSource resolveReferenceSource(const TrajectoryCommand *selected_command) const;
	void captureActivationAnchor(const IntRefStepInput &input);
	void captureHoldAnchor(const IntRefStepInput &input);
	void writeHoldSetpoint(IntRefStepResult &result) const;
	void updateStatusSnapshot();
	bool validateSetpoint(const Setpoint &setpoint) const;
	void activateHoldFromPluginFailure(const IntRefStepInput &input, const char *reason, IntRefStepResult &result);
	trajectory_setpoint_s measuredSetpoint(const IntRefStepInput &input) const;
	trajectory_setpoint_s blendTransitionSetpoint(const trajectory_setpoint_s &target, const IntRefStepInput &input);
	void applyYawContinuity(const IntRefStepInput &input, bool limit_yaw_rate, trajectory_setpoint_s &setpoint);
	void finalizeProducedSetpoint(const IntRefStepInput &input, bool limit_yaw_rate, IntRefStepResult &result);
	void updateTransitionDurationForTarget(const trajectory_setpoint_s &target, const IntRefStepInput &input);
	void resetTransitionState();

	InternalReferenceConfigured _configured_mode = InternalReferenceConfigured::NONE;
	ReferenceMode _reference_mode = ReferenceMode::EXTREF;
	bool _reference_mode_user_set = false;

	TrajectoryCommand _configured_lissajous_command {};
	TrajectoryCommand _configured_circle_command {};
	TrajectoryCommand _selected_internal_trajectory {};
	bool _selected_internal_trajectory_valid = false;

	bool _reanchor_pending = true;
	bool _activation_anchor_valid = false;
	TrajectoryCommand _last_anchor_command {};
	bool _last_anchor_command_valid = false;
	float _activation_position[3] {0.0f, 0.0f, 0.0f};
	float _activation_orientation[4] {1.0f, 0.0f, 0.0f, 0.0f};
	hrt_abstime _activation_time = 0;

	bool _hold_pending_capture = false;
	float _hold_position[3] {0.0f, 0.0f, 0.0f};
	float _hold_yaw = 0.0f;

	float _transition_time_s = 2.0f;
	float _transition_max_yaw_rate_rad_s = 0.8f;
	bool _transition_active = false;
	hrt_abstime _transition_start_time = 0;
	float _transition_duration_s = 0.0f;
	trajectory_setpoint_s _transition_from_setpoint {};
	bool _last_output_valid = false;
	trajectory_setpoint_s _last_output_setpoint {};
	float _last_output_yaw_unwrapped = 0.0f;
	hrt_abstime _last_output_timestamp = 0;
	float _transition_progress = 0.0f;
	float _transition_remaining_s = 0.0f;

	IntRefStatusSnapshot _status_snapshot {};
};

} // namespace mc_raptor_intref
