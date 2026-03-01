#include "intref_runtime.hpp"

#include <matrix/matrix/math.hpp>

#include <math.h>
#include <stdio.h>
#include <string.h>

namespace mc_raptor_intref
{

namespace
{

TrajectoryCommand make_default_command(uint8_t id, const char *name)
{
	TrajectoryCommand command {};
	command.id = id;
	set_command_name(command, name);
	return command;
}

float yaw_from_quat(const float q[4])
{
	return atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
}

float wrap_pi(float angle)
{
	const float two_pi = 2.0f * static_cast<float>(M_PI);

	while (angle > static_cast<float>(M_PI)) {
		angle -= two_pi;
	}

	while (angle < -static_cast<float>(M_PI)) {
		angle += two_pi;
	}

	return angle;
}

float unwrap_angle_near(float reference, float target)
{
	return reference + wrap_pi(target - reference);
}

float smoothstep01(float u)
{
	return u * u * (3.0f - 2.0f * u);
}

float constrainf_scalar(float value, float min_value, float max_value)
{
	return fminf(fmaxf(value, min_value), max_value);
}

} // namespace

IntRefRuntimeManager::IntRefRuntimeManager()
{
	_configured_lissajous_command = make_default_command(TRAJECTORY_ID_LISSAJOUS, "lissajous");
	_configured_lissajous_command.arg_count = 8;
	_configured_lissajous_command.args[0] = 0.5f;
	_configured_lissajous_command.args[1] = 1.0f;
	_configured_lissajous_command.args[2] = 0.0f;
	_configured_lissajous_command.args[3] = 2.0f;
	_configured_lissajous_command.args[4] = 1.0f;
	_configured_lissajous_command.args[5] = 1.0f;
	_configured_lissajous_command.args[6] = 10.0f;
	_configured_lissajous_command.args[7] = 3.0f;

	_configured_circle_command = make_default_command(TRAJECTORY_ID_CIRCLE, "circle");
	_configured_circle_command.arg_count = 3;
	_configured_circle_command.args[0] = 1.0f;
	_configured_circle_command.args[1] = 1.0f;
	_configured_circle_command.args[2] = 3.0f;

	updateStatusSnapshot();
}

void IntRefRuntimeManager::resetTransitionState()
{
	_transition_active = false;
	_transition_start_time = 0;
	_transition_duration_s = 0.0f;
	_transition_progress = 0.0f;
	_transition_remaining_s = 0.0f;
}

void IntRefRuntimeManager::reset()
{
	_reference_mode = ReferenceMode::EXTREF;
	_reference_mode_user_set = false;
	_selected_internal_trajectory_valid = false;
	_reanchor_pending = true;
	_activation_anchor_valid = false;
	_last_anchor_command_valid = false;
	_hold_pending_capture = false;
	_hold_position[0] = 0.0f;
	_hold_position[1] = 0.0f;
	_hold_position[2] = 0.0f;
	_hold_yaw = 0.0f;
	_last_output_valid = false;
	_last_output_yaw_unwrapped = 0.0f;
	_last_output_timestamp = 0;
	resetTransitionState();
	updateStatusSnapshot();
}

void IntRefRuntimeManager::setConfiguredMode(InternalReferenceConfigured mode)
{
	if (_configured_mode != mode) {
		_configured_mode = mode;
		_reanchor_pending = true;
	}

	if (!_reference_mode_user_set) {
		_reference_mode = (_configured_mode == InternalReferenceConfigured::NONE) ? ReferenceMode::EXTREF : ReferenceMode::INTREF;
	}

	updateStatusSnapshot();
}

InternalReferenceConfigured IntRefRuntimeManager::configuredMode() const
{
	return _configured_mode;
}

void IntRefRuntimeManager::setTransitionConfig(float transition_time_s, float max_yaw_rate_rad_s)
{
	_transition_time_s = constrainf_scalar(transition_time_s, 0.0f, 10.0f);
	_transition_max_yaw_rate_rad_s = constrainf_scalar(max_yaw_rate_rad_s, 0.2f, 3.0f);
}

bool IntRefRuntimeManager::setTrajectoryCommand(const TrajectoryCommand &command, char *error, size_t error_len)
{
	const TrajectoryPluginDescriptor *plugin = TrajectoryRegistry::instance().findById(command.id);

	if (plugin == nullptr) {
		if (error != nullptr && error_len > 0) {
			snprintf(error, error_len, "unknown trajectory id: %u", (unsigned)command.id);
		}

		return false;
	}

	if (command.id == TRAJECTORY_ID_LISSAJOUS) {
		_configured_lissajous_command = command;

	} else if (command.id == TRAJECTORY_ID_CIRCLE) {
		_configured_circle_command = command;
	}

	_selected_internal_trajectory = command;
	_selected_internal_trajectory_valid = true;
	_reanchor_pending = true;
	updateStatusSnapshot();
	return true;
}

bool IntRefRuntimeManager::setReferenceMode(ReferenceMode mode, char *error, size_t error_len)
{
	if (mode == ReferenceMode::INTREF && !canUseIntref()) {
		if (error != nullptr && error_len > 0) {
			snprintf(error, error_len, "no internal trajectory available, run 'mc_raptor intref set ...' or set MC_RAPTOR_INTREF");
		}

		return false;
	}

	_reference_mode = mode;
	_reference_mode_user_set = true;

	if (_reference_mode == ReferenceMode::HOLD) {
		_hold_pending_capture = true;
		resetTransitionState();

	} else {
		_hold_pending_capture = false;
	}

	if (_reference_mode == ReferenceMode::INTREF) {
		_reanchor_pending = true;

	} else {
		resetTransitionState();
	}

	updateStatusSnapshot();
	return true;
}

ReferenceMode IntRefRuntimeManager::referenceMode() const
{
	return _reference_mode;
}

const TrajectoryCommand *IntRefRuntimeManager::selectTrajectoryCommand() const
{
	if (_selected_internal_trajectory_valid) {
		return &_selected_internal_trajectory;
	}

	switch (_configured_mode) {
	case InternalReferenceConfigured::LISSAJOUS:
		return &_configured_lissajous_command;

	case InternalReferenceConfigured::CIRCLE:
		return &_configured_circle_command;

	case InternalReferenceConfigured::NONE:
	default:
		break;
	}

	return nullptr;
}

bool IntRefRuntimeManager::canUseIntref() const
{
	return selectTrajectoryCommand() != nullptr;
}

ReferenceSource IntRefRuntimeManager::resolveReferenceSource(const TrajectoryCommand *selected_command) const
{
	switch (_reference_mode) {
	case ReferenceMode::EXTREF:
		return ReferenceSource::EXTERNAL;

	case ReferenceMode::HOLD:
		return ReferenceSource::HOLD;

	case ReferenceMode::INTREF:
		if (selected_command == nullptr) {
			return ReferenceSource::INTREF_CUSTOM;
		}

		switch (selected_command->id) {
		case TRAJECTORY_ID_LISSAJOUS:
			return ReferenceSource::INTERNAL_LISSAJOUS;

		case TRAJECTORY_ID_CIRCLE:
			return ReferenceSource::INTERNAL_CIRCLE;

		default:
			return ReferenceSource::INTREF_CUSTOM;
		}
	}

	return ReferenceSource::EXTERNAL;
}

void IntRefRuntimeManager::captureActivationAnchor(const IntRefStepInput &input)
{
	_activation_position[0] = input.position[0];
	_activation_position[1] = -input.position[1];
	_activation_position[2] = -input.position[2];
	_activation_orientation[0] = input.attitude_q[0];
	_activation_orientation[1] = input.attitude_q[1];
	_activation_orientation[2] = -input.attitude_q[2];
	_activation_orientation[3] = -input.attitude_q[3];
	_activation_time = input.now;
	_activation_anchor_valid = true;
	_reanchor_pending = false;
}

void IntRefRuntimeManager::captureHoldAnchor(const IntRefStepInput &input)
{
	_hold_position[0] = input.position[0];
	_hold_position[1] = input.position[1];
	_hold_position[2] = input.position[2];
	_hold_yaw = yaw_from_quat(input.attitude_q);
	_hold_pending_capture = false;
}

trajectory_setpoint_s IntRefRuntimeManager::measuredSetpoint(const IntRefStepInput &input) const
{
	trajectory_setpoint_s setpoint {};
	setpoint.position[0] = input.position[0];
	setpoint.position[1] = input.position[1];
	setpoint.position[2] = input.position[2];
	setpoint.velocity[0] = input.linear_velocity[0];
	setpoint.velocity[1] = input.linear_velocity[1];
	setpoint.velocity[2] = input.linear_velocity[2];
	setpoint.yaw = yaw_from_quat(input.attitude_q);
	setpoint.yawspeed = 0.0f;
	return setpoint;
}

trajectory_setpoint_s IntRefRuntimeManager::blendTransitionSetpoint(const trajectory_setpoint_s &target, const IntRefStepInput &input)
{
	if (!_transition_active || _transition_duration_s <= 0.0f) {
		return target;
	}

	const float elapsed = static_cast<float>(input.now - _transition_start_time) / 1e6f;
	const float position_transition_duration = fmaxf(_transition_time_s, 0.0f);
	const float u_position = (position_transition_duration > 0.0f)
				 ? constrainf_scalar(elapsed / position_transition_duration, 0.0f, 1.0f)
				 : 1.0f;
	const float u_yaw = (_transition_duration_s > 0.0f) ? constrainf_scalar(elapsed / _transition_duration_s, 0.0f, 1.0f) : 1.0f;
	const float a_position = smoothstep01(u_position);
	const float a_yaw = smoothstep01(u_yaw);
	trajectory_setpoint_s blended {};

	for (int i = 0; i < 3; ++i) {
		blended.position[i] = _transition_from_setpoint.position[i]
				     + a_position * (target.position[i] - _transition_from_setpoint.position[i]);
		blended.velocity[i] = _transition_from_setpoint.velocity[i]
				     + a_position * (target.velocity[i] - _transition_from_setpoint.velocity[i]);
	}

	const float unwrapped_target_yaw = unwrap_angle_near(_transition_from_setpoint.yaw, target.yaw);
	blended.yaw = _transition_from_setpoint.yaw + a_yaw * (unwrapped_target_yaw - _transition_from_setpoint.yaw);
	blended.yawspeed = _transition_from_setpoint.yawspeed + a_yaw * (target.yawspeed - _transition_from_setpoint.yawspeed);

	_transition_progress = fminf(u_position, u_yaw);
	_transition_remaining_s = fmaxf((1.0f - u_position) * position_transition_duration, (1.0f - u_yaw) * _transition_duration_s);

	if (u_position >= 1.0f && u_yaw >= 1.0f) {
		_transition_active = false;
		_transition_progress = 1.0f;
		_transition_remaining_s = 0.0f;
		// Pause trajectory time while transitioning, then start the trajectory from t=0.
		_activation_time = input.now;
	}

	return blended;
}

void IntRefRuntimeManager::applyYawContinuity(const IntRefStepInput &input, bool limit_yaw_rate, trajectory_setpoint_s &setpoint)
{
	if (!_last_output_valid) {
		_last_output_yaw_unwrapped = setpoint.yaw;
		setpoint.yaw = wrap_pi(setpoint.yaw);
		return;
	}

	const float unwrapped_yaw = unwrap_angle_near(_last_output_yaw_unwrapped, setpoint.yaw);
	float output_yaw_unwrapped = unwrapped_yaw;

	if (limit_yaw_rate && _transition_max_yaw_rate_rad_s > 0.0f
	    && _last_output_timestamp > 0 && input.now > _last_output_timestamp) {
		const float dt = static_cast<float>(input.now - _last_output_timestamp) / 1e6f;
		const float max_delta = _transition_max_yaw_rate_rad_s * dt;
		const float delta = unwrapped_yaw - _last_output_yaw_unwrapped;
		const float limited_delta = constrainf_scalar(delta, -max_delta, max_delta);
		output_yaw_unwrapped = _last_output_yaw_unwrapped + limited_delta;

		if (dt > 1e-6f) {
			setpoint.yawspeed = limited_delta / dt;
		}
	}

	_last_output_yaw_unwrapped = output_yaw_unwrapped;
	setpoint.yaw = wrap_pi(output_yaw_unwrapped);
}

void IntRefRuntimeManager::updateTransitionDurationForTarget(const trajectory_setpoint_s &target, const IntRefStepInput &input)
{
	if (!_transition_active) {
		return;
	}

	const float base_duration = fmaxf(_transition_time_s, 0.0f);
	float yaw_duration = 0.0f;

	if (_transition_max_yaw_rate_rad_s > 1e-3f) {
		const float unwrapped_target_yaw = unwrap_angle_near(_transition_from_setpoint.yaw, target.yaw);
		const float yaw_delta = fabsf(unwrapped_target_yaw - _transition_from_setpoint.yaw);
		// smoothstep peaks at 1.5x average slope, account for this to avoid yaw lag.
		yaw_duration = 1.5f * yaw_delta / _transition_max_yaw_rate_rad_s;
	}

	_transition_duration_s = fmaxf(base_duration, yaw_duration);

	if (_transition_duration_s <= 0.0f) {
		_transition_active = false;
		_transition_progress = 1.0f;
		_transition_remaining_s = 0.0f;
		_activation_time = input.now;
	}
}

void IntRefRuntimeManager::finalizeProducedSetpoint(const IntRefStepInput &input, bool limit_yaw_rate, IntRefStepResult &result)
{
	if (!result.produced_setpoint) {
		return;
	}

	applyYawContinuity(input, limit_yaw_rate, result.setpoint);
	_last_output_setpoint = result.setpoint;
	_last_output_timestamp = input.now;
	_last_output_valid = true;
}

void IntRefRuntimeManager::writeHoldSetpoint(IntRefStepResult &result) const
{
	result.produced_setpoint = true;
	result.internal_reference_valid = true;
	result.setpoint.position[0] = _hold_position[0];
	result.setpoint.position[1] = _hold_position[1];
	result.setpoint.position[2] = _hold_position[2];
	result.setpoint.velocity[0] = 0.0f;
	result.setpoint.velocity[1] = 0.0f;
	result.setpoint.velocity[2] = 0.0f;
	result.setpoint.yaw = _hold_yaw;
	result.setpoint.yawspeed = 0.0f;
	result.internal_reference_position[0] = _hold_position[0];
	result.internal_reference_position[1] = _hold_position[1];
	result.internal_reference_position[2] = _hold_position[2];
	result.internal_reference_linear_velocity[0] = 0.0f;
	result.internal_reference_linear_velocity[1] = 0.0f;
	result.internal_reference_linear_velocity[2] = 0.0f;
}

bool IntRefRuntimeManager::validateSetpoint(const Setpoint &setpoint) const
{
	return PX4_ISFINITE(setpoint.position[0])
	       && PX4_ISFINITE(setpoint.position[1])
	       && PX4_ISFINITE(setpoint.position[2])
	       && PX4_ISFINITE(setpoint.linear_velocity[0])
	       && PX4_ISFINITE(setpoint.linear_velocity[1])
	       && PX4_ISFINITE(setpoint.linear_velocity[2])
	       && PX4_ISFINITE(setpoint.yaw)
	       && PX4_ISFINITE(setpoint.yaw_rate);
}

void IntRefRuntimeManager::activateHoldFromPluginFailure(const IntRefStepInput &input, const char *reason, IntRefStepResult &result)
{
	_reference_mode = ReferenceMode::HOLD;
	_reference_mode_user_set = true;
	resetTransitionState();
	captureHoldAnchor(input);
	result.plugin_error = true;
	strncpy(result.plugin_error_message, reason, sizeof(result.plugin_error_message) - 1);
	result.plugin_error_message[sizeof(result.plugin_error_message) - 1] = '\0';
	result.reference_source = ReferenceSource::HOLD;
	writeHoldSetpoint(result);
	finalizeProducedSetpoint(input, false, result);
	updateStatusSnapshot();
}

IntRefStepResult IntRefRuntimeManager::step(const IntRefStepInput &input)
{
	IntRefStepResult result {};
	const TrajectoryCommand *selected_command = selectTrajectoryCommand();
	result.reference_source = resolveReferenceSource(selected_command);

	if (_reference_mode == ReferenceMode::EXTREF) {
		resetTransitionState();
		_last_output_valid = false;
		_last_output_yaw_unwrapped = 0.0f;
		_last_output_timestamp = 0;
		updateStatusSnapshot();
		return result;
	}

	if (_reference_mode == ReferenceMode::HOLD) {
		if (_hold_pending_capture && input.vehicle_active) {
			captureHoldAnchor(input);
		}

		if (!_hold_pending_capture) {
			writeHoldSetpoint(result);
			finalizeProducedSetpoint(input, false, result);
		}

		updateStatusSnapshot();
		return result;
	}

	if (selected_command == nullptr) {
		activateHoldFromPluginFailure(input, "no internal trajectory selected", result);
		return result;
	}

	const TrajectoryPluginDescriptor *plugin = TrajectoryRegistry::instance().findById(selected_command->id);

	if (plugin == nullptr || plugin->evaluate == nullptr) {
		activateHoldFromPluginFailure(input, "trajectory plugin not found", result);
		return result;
	}

	const bool selected_changed = !_last_anchor_command_valid || !same_command(_last_anchor_command, *selected_command);
	bool reanchored = false;

	if (input.vehicle_active && (!_activation_anchor_valid || input.just_activated || _reanchor_pending || selected_changed)) {
		captureActivationAnchor(input);
		_last_anchor_command = *selected_command;
		_last_anchor_command_valid = true;
		reanchored = true;
	}

	if (reanchored) {
		_transition_from_setpoint = measuredSetpoint(input);
		_transition_start_time = input.now;
		_transition_duration_s = 0.0f;
		_transition_active = true;
		_transition_progress = 0.0f;
		_transition_remaining_s = 0.0f;
	}

	if (!_activation_anchor_valid) {
		updateStatusSnapshot();
		return result;
	}

	const float trajectory_time_s = _transition_active ? 0.0f : static_cast<float>(input.now - _activation_time) / 1e6f;
	Setpoint setpoint = plugin->evaluate(trajectory_time_s, *selected_command);

	if (!validateSetpoint(setpoint)) {
		activateHoldFromPluginFailure(input, "trajectory setpoint invalid", result);
		return result;
	}

	auto &q = _activation_orientation;
	matrix::Quatf q_activation_frame(q[0], q[1], q[2], q[3]);
	matrix::Vector3f position_activation_frame = q_activation_frame.rotateVector(matrix::Vector3f(setpoint.position[0], setpoint.position[1],
			setpoint.position[2]));
	matrix::Vector3f linear_velocity_activation_frame = q_activation_frame.rotateVector(matrix::Vector3f(setpoint.linear_velocity[0],
			setpoint.linear_velocity[1], setpoint.linear_velocity[2]));

	trajectory_setpoint_s target_setpoint {};
	target_setpoint.position[0] = +(_activation_position[0] + position_activation_frame(0));
	target_setpoint.position[1] = -(_activation_position[1] + position_activation_frame(1));
	target_setpoint.position[2] = -(_activation_position[2] + position_activation_frame(2));
	target_setpoint.yaw = -yaw_from_quat(q) - setpoint.yaw;
	target_setpoint.velocity[0] = +linear_velocity_activation_frame(0);
	target_setpoint.velocity[1] = -linear_velocity_activation_frame(1);
	target_setpoint.velocity[2] = -linear_velocity_activation_frame(2);
	target_setpoint.yawspeed = -setpoint.yaw_rate;
	updateTransitionDurationForTarget(target_setpoint, input);

	const bool transition_active_before_blend = _transition_active;
	result.produced_setpoint = true;
	result.internal_reference_valid = true;
	result.setpoint = transition_active_before_blend ? blendTransitionSetpoint(target_setpoint, input) : target_setpoint;
	finalizeProducedSetpoint(input, transition_active_before_blend, result);

	result.internal_reference_position[0] = result.setpoint.position[0];
	result.internal_reference_position[1] = result.setpoint.position[1];
	result.internal_reference_position[2] = result.setpoint.position[2];
	result.internal_reference_linear_velocity[0] = result.setpoint.velocity[0];
	result.internal_reference_linear_velocity[1] = result.setpoint.velocity[1];
	result.internal_reference_linear_velocity[2] = result.setpoint.velocity[2];

	updateStatusSnapshot();
	return result;
}

IntRefStatusSnapshot IntRefRuntimeManager::statusSnapshot() const
{
	return _status_snapshot;
}

TrajectoryCommand IntRefRuntimeManager::configuredLissajousCommand() const
{
	return _configured_lissajous_command;
}

TrajectoryCommand IntRefRuntimeManager::configuredCircleCommand() const
{
	return _configured_circle_command;
}

bool IntRefRuntimeManager::hasSelectedInternalTrajectory() const
{
	return _selected_internal_trajectory_valid;
}

TrajectoryCommand IntRefRuntimeManager::selectedInternalTrajectory() const
{
	return _selected_internal_trajectory;
}

void IntRefRuntimeManager::updateStatusSnapshot()
{
	_status_snapshot.configured_mode = _configured_mode;
	_status_snapshot.reference_mode = _reference_mode;
	_status_snapshot.hold_active = _reference_mode == ReferenceMode::HOLD;
	_status_snapshot.hold_pending_capture = _hold_pending_capture;
	_status_snapshot.hold_position[0] = _hold_position[0];
	_status_snapshot.hold_position[1] = _hold_position[1];
	_status_snapshot.hold_position[2] = _hold_position[2];
	_status_snapshot.hold_yaw = _hold_yaw;
	_status_snapshot.transition_active = _transition_active;
	_status_snapshot.transition_progress = _transition_progress;
	_status_snapshot.transition_remaining_s = _transition_remaining_s;

	const TrajectoryCommand *selected_command = selectTrajectoryCommand();
	_status_snapshot.reference_source = resolveReferenceSource(selected_command);

	if (_reference_mode != ReferenceMode::INTREF || selected_command == nullptr) {
		_status_snapshot.active_trajectory_id = TRAJECTORY_ID_NONE;
		_status_snapshot.active_trajectory_name[0] = '\0';

	} else {
		_status_snapshot.active_trajectory_id = selected_command->id;
		strncpy(_status_snapshot.active_trajectory_name, selected_command->name,
			sizeof(_status_snapshot.active_trajectory_name) - 1);
		_status_snapshot.active_trajectory_name[sizeof(_status_snapshot.active_trajectory_name) - 1] = '\0';
	}
}

} // namespace mc_raptor_intref
