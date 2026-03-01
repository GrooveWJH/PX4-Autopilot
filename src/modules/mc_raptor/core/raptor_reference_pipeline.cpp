#include "raptor_reference_pipeline.hpp"

#include "../mc_raptor.hpp"

#include <containers/LockGuard.hpp>

void Raptor::fill_status_from_intref_snapshot(const mc_raptor_intref::IntRefStatusSnapshot &snapshot, raptor_status_s &status) const
{
	status.internal_reference_configured = static_cast<uint8_t>(snapshot.configured_mode);
	status.reference_mode = static_cast<uint8_t>(snapshot.reference_mode);
	status.reference_source = static_cast<uint8_t>(snapshot.reference_source);
	status.active_trajectory_id = snapshot.active_trajectory_id;
	strncpy(status.active_trajectory_name, snapshot.active_trajectory_name, sizeof(status.active_trajectory_name) - 1);
	status.active_trajectory_name[sizeof(status.active_trajectory_name) - 1] = '\0';
	status.hold_active = snapshot.hold_active;
}

void Raptor::update_runtime_config_from_params(raptor_status_s &status)
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update {};
		_parameter_update_sub.copy(&parameter_update);
		updateParams();
	}

	auto previous_internal_reference = internal_reference;
	const int32_t configured_mode_param = _param_mc_raptor_intref.get();

	switch (configured_mode_param) {
	case 1:
		internal_reference = mc_raptor_intref::InternalReferenceConfigured::LISSAJOUS;
		break;

	case 2:
		internal_reference = mc_raptor_intref::InternalReferenceConfigured::CIRCLE;
		break;

	case 0:
	default:
		internal_reference = mc_raptor_intref::InternalReferenceConfigured::NONE;
		break;
	}

	if (previous_internal_reference != internal_reference) {
		PX4_INFO("internal reference configured changed from %d to %d", (int)previous_internal_reference, (int)internal_reference);
	}

	{
		LockGuard intref_lock{_intref_runtime_mutex};
		_intref_runtime.setConfiguredMode(internal_reference);
		const mc_raptor_intref::IntRefStatusSnapshot snapshot = _intref_runtime.statusSnapshot();
		reference_source = snapshot.reference_source;
		fill_status_from_intref_snapshot(snapshot, status);
	}
}

void Raptor::update_external_setpoint_subscription(hrt_abstime current_time, bool next_active, bool use_external_reference,
		raptor_status_s &status)
{
	trajectory_setpoint_s temp_trajectory_setpoint {};
	status.subscription_update_trajectory_setpoint = use_external_reference && _trajectory_setpoint_sub.update(&temp_trajectory_setpoint);

	if (status.subscription_update_trajectory_setpoint) {
		if (
			PX4_ISFINITE(temp_trajectory_setpoint.position[0]) &&
			PX4_ISFINITE(temp_trajectory_setpoint.position[1]) &&
			PX4_ISFINITE(temp_trajectory_setpoint.position[2]) &&
			PX4_ISFINITE(temp_trajectory_setpoint.yaw) &&
			PX4_ISFINITE(temp_trajectory_setpoint.velocity[0]) &&
			PX4_ISFINITE(temp_trajectory_setpoint.velocity[1]) &&
			PX4_ISFINITE(temp_trajectory_setpoint.velocity[2]) &&
			PX4_ISFINITE(temp_trajectory_setpoint.yawspeed)
		) {
			if (_timestamp_last_external_trajectory_setpoint_set) {
				trajectory_setpoint_dts[trajectory_setpoint_dt_index] = current_time - _timestamp_last_external_trajectory_setpoint;
				trajectory_setpoint_dt_index++;

				if (trajectory_setpoint_dt_index == NUM_TRAJECTORY_SETPOINT_DTS) {
					if (next_active && !trajectory_setpoint_dts_full) {
						PX4_INFO("trajectory_setpoint_dts_full");
					}

					trajectory_setpoint_dts_full = true;
					trajectory_setpoint_dt_index = 0;
				}
			}

			_timestamp_last_external_trajectory_setpoint_set = true;
			_timestamp_last_external_trajectory_setpoint = current_time;
			_trajectory_setpoint = temp_trajectory_setpoint;

		} else {
			trajectory_setpoint_invalid_count++;

			if (next_active && trajectory_setpoint_invalid_count % TRAJECTORY_SETPOINT_INVALID_COUNT_WARNING_INTERVAL == 0) {
				PX4_WARN("trajectory_setpoint invalid, count: %d", (int)trajectory_setpoint_invalid_count);
			}
		}
	}

	status.timestamp_last_trajectory_setpoint = _timestamp_last_external_trajectory_setpoint_set ?
			_timestamp_last_external_trajectory_setpoint : 0;
}

void Raptor::step_internal_reference_if_needed(hrt_abstime current_time, bool next_active, raptor_status_s &status)
{
	if (reference_source == mc_raptor_intref::ReferenceSource::EXTERNAL) {
		return;
	}

	mc_raptor_intref::IntRefStepInput intref_step_input {};
	intref_step_input.now = current_time;
	intref_step_input.vehicle_active = next_active;
	intref_step_input.just_activated = !previous_active && next_active;
	intref_step_input.position[0] = position[0];
	intref_step_input.position[1] = position[1];
	intref_step_input.position[2] = position[2];
	intref_step_input.attitude_q[0] = _vehicle_attitude.q[0];
	intref_step_input.attitude_q[1] = _vehicle_attitude.q[1];
	intref_step_input.attitude_q[2] = _vehicle_attitude.q[2];
	intref_step_input.attitude_q[3] = _vehicle_attitude.q[3];

	mc_raptor_intref::IntRefStepResult intref_step_result {};
	mc_raptor_intref::IntRefStatusSnapshot intref_snapshot_post {};

	{
		LockGuard intref_lock{_intref_runtime_mutex};
		intref_step_result = _intref_runtime.step(intref_step_input);
		intref_snapshot_post = _intref_runtime.statusSnapshot();
	}

	reference_source = intref_step_result.reference_source;

	if (intref_step_result.plugin_error) {
		PX4_ERR("intref plugin error, switching to hold: %s", intref_step_result.plugin_error_message);
	}

	if (intref_step_result.produced_setpoint) {
		_trajectory_setpoint = intref_step_result.setpoint;
	}

	if (intref_step_result.internal_reference_valid) {
		status.internal_reference_position[0] = intref_step_result.internal_reference_position[0];
		status.internal_reference_position[1] = intref_step_result.internal_reference_position[1];
		status.internal_reference_position[2] = intref_step_result.internal_reference_position[2];
		status.internal_reference_linear_velocity[0] = intref_step_result.internal_reference_linear_velocity[0];
		status.internal_reference_linear_velocity[1] = intref_step_result.internal_reference_linear_velocity[1];
		status.internal_reference_linear_velocity[2] = intref_step_result.internal_reference_linear_velocity[2];
	}

	reference_source = intref_snapshot_post.reference_source;
	fill_status_from_intref_snapshot(intref_snapshot_post, status);
}

void Raptor::apply_external_stale_logic(hrt_abstime current_time, bool next_active, raptor_status_s &status)
{
	if (reference_source == mc_raptor_intref::ReferenceSource::EXTERNAL) {
		if (!_timestamp_last_external_trajectory_setpoint_set
		    || (current_time - _timestamp_last_external_trajectory_setpoint) > TRAJECTORY_SETPOINT_TIMEOUT) {
			status.trajectory_setpoint_stale = true;

			if (!_previous_external_trajectory_setpoint_stale || (!previous_active && next_active)) {
				_trajectory_setpoint.position[0] = position[0];
				_trajectory_setpoint.position[1] = position[1];
				_trajectory_setpoint.position[2] = position[2];
				auto &q = _vehicle_attitude.q;
				_trajectory_setpoint.yaw = atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
				_trajectory_setpoint.velocity[0] = 0;
				_trajectory_setpoint.velocity[1] = 0;
				_trajectory_setpoint.velocity[2] = 0;
				_trajectory_setpoint.yawspeed = 0;

				if (!_previous_external_trajectory_setpoint_stale) {
					PX4_WARN("trajectory_setpoint turned stale at: %f %f %f, yaw: %f %llu / %llu us", (double)position[0], (double)position[1],
						 (double)position[2],
						 (double)_trajectory_setpoint.yaw, (unsigned long long)(current_time - _timestamp_last_external_trajectory_setpoint),
						 (unsigned long long)(TRAJECTORY_SETPOINT_TIMEOUT));

				} else {
					PX4_WARN("trajectory_setpoint reset due to activation at: %f %f %f, yaw: %f", (double)position[0], (double)position[1], (double)position[2],
						 (double)_trajectory_setpoint.yaw);
				}
			}

			_previous_external_trajectory_setpoint_stale = true;

		} else {
			if (_previous_external_trajectory_setpoint_stale) {
				PX4_WARN("trajectory_setpoint turned non-stale at: %f %f %f", (double)position[0], (double)position[1], (double)position[2]);
				_previous_external_trajectory_setpoint_stale = false;
			}

			status.trajectory_setpoint_stale = false;
		}

	} else {
		if (_previous_external_trajectory_setpoint_stale) {
			PX4_WARN("trajectory_setpoint turned non-stale at: %f %f %f", (double)position[0], (double)position[1], (double)position[2]);
			_previous_external_trajectory_setpoint_stale = false;
		}

		status.trajectory_setpoint_stale = false;
	}
}
