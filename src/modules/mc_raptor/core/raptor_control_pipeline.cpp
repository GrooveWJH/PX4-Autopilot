#include "raptor_control_pipeline.hpp"

#include "../mc_raptor.hpp"
#include "raptor_math.hpp"

#include <rl_tools/inference/applications/l2f/operations_generic.h>

namespace
{
constexpr bool PUBLISH_NON_COMPLETE_STATUS = true;
}

void Raptor::observe(rl_tools::inference::applications::l2f::Observation<EXECUTOR_SPEC> &observation)
{
	T Rt_inv[9];

	{
		T q_target[4];
		q_target[0] = cosf(0.5f * _trajectory_setpoint.yaw);
		q_target[1] = 0;
		q_target[2] = 0;
		q_target[3] = sinf(0.5f * _trajectory_setpoint.yaw);

		T qt[4], qtc[4], qr[4];
		qt[0] = +q_target[0];
		qt[1] = +q_target[1];
		qt[2] = -q_target[2];
		qt[3] = -q_target[3];
		mc_raptor_core::math::quaternion_conjugate(qt, qtc);
		mc_raptor_core::math::quaternion_to_rotation_matrix(qtc, Rt_inv);

		qr[0] = +_vehicle_attitude.q[0];
		qr[1] = +_vehicle_attitude.q[1];
		qr[2] = -_vehicle_attitude.q[2];
		qr[3] = -_vehicle_attitude.q[3];
		T qd[4];
		mc_raptor_core::math::quaternion_multiplication(qtc, qr, qd);

		observation.orientation[0] = qd[0];
		observation.orientation[1] = qd[1];
		observation.orientation[2] = qd[2];
		observation.orientation[3] = qd[3];
	}

	{
		T p[3], pt[3];
		p[0] = +(position[0] - _trajectory_setpoint.position[0]);
		p[1] = -(position[1] - _trajectory_setpoint.position[1]);
		p[2] = -(position[2] - _trajectory_setpoint.position[2]);
		mc_raptor_core::math::rotate_vector(Rt_inv, p, pt);
		observation.position[0] = mc_raptor_core::math::clip(pt[0], max_position_error, -max_position_error);
		observation.position[1] = mc_raptor_core::math::clip(pt[1], max_position_error, -max_position_error);
		observation.position[2] = mc_raptor_core::math::clip(pt[2], max_position_error, -max_position_error);
	}

	{
		T v[3], vt[3];
		v[0] = +(linear_velocity[0] - _trajectory_setpoint.velocity[0]);
		v[1] = -(linear_velocity[1] - _trajectory_setpoint.velocity[1]);
		v[2] = -(linear_velocity[2] - _trajectory_setpoint.velocity[2]);
		mc_raptor_core::math::rotate_vector(Rt_inv, v, vt);
		observation.linear_velocity[0] = mc_raptor_core::math::clip(vt[0], max_velocity_error, -max_velocity_error);
		observation.linear_velocity[1] = mc_raptor_core::math::clip(vt[1], max_velocity_error, -max_velocity_error);
		observation.linear_velocity[2] = mc_raptor_core::math::clip(vt[2], max_velocity_error, -max_velocity_error);
	}

	observation.angular_velocity[0] = +_vehicle_angular_velocity.xyz[0];
	observation.angular_velocity[1] = -_vehicle_angular_velocity.xyz[1];
	observation.angular_velocity[2] = -_vehicle_angular_velocity.xyz[2];

	for (TI action_i = 0; action_i < EXECUTOR_CONFIG::OUTPUT_DIM; action_i++) {
		observation.previous_action[action_i] = previous_action[action_i];
	}
}

void Raptor::initialize_status_message(hrt_abstime current_time, raptor_status_s &status) const
{
	status.timestamp = current_time;
	status.timestamp_sample = current_time;
	status.exit_reason = raptor_status_s::EXIT_REASON_NONE;
	status.substep = 0;
	status.active = false;
	status.control_interval = NAN;
	status.trajectory_setpoint_dt_mean = NAN;
	status.trajectory_setpoint_dt_max = NAN;
	status.trajectory_setpoint_dt_max_since_activation = NAN;

	for (TI i = 0; i < 3; i++) {
		status.internal_reference_position[i] = NAN;
		status.internal_reference_linear_velocity[i] = NAN;
	}
}

void Raptor::update_trajectory_setpoint_timing_stats(raptor_status_s &status)
{
	if (!trajectory_setpoint_dts_full && trajectory_setpoint_dt_index == 0) {
		return;
	}

	float trajectory_setpoint_dt_mean = 0;
	float trajectory_setpoint_dt_max = 0;

	for (TI i = 0; i < (trajectory_setpoint_dts_full ? NUM_TRAJECTORY_SETPOINT_DTS : trajectory_setpoint_dt_index); i++) {
		TI index = trajectory_setpoint_dts_full ? i : trajectory_setpoint_dt_index - 1 - i;
		trajectory_setpoint_dt_mean += trajectory_setpoint_dts[index];

		if (trajectory_setpoint_dts[index] > trajectory_setpoint_dt_max) {
			trajectory_setpoint_dt_max = trajectory_setpoint_dts[index];
		}
	}

	if (trajectory_setpoint_dt_max > trajectory_setpoint_dt_max_since_reset) {
		trajectory_setpoint_dt_max_since_reset = trajectory_setpoint_dt_max;
	}

	trajectory_setpoint_dt_mean /= NUM_TRAJECTORY_SETPOINT_DTS;
	status.trajectory_setpoint_dt_mean = trajectory_setpoint_dt_mean;
	status.trajectory_setpoint_dt_max = trajectory_setpoint_dt_max;
	status.trajectory_setpoint_dt_max_since_activation = trajectory_setpoint_dt_max_since_reset;
}

bool Raptor::update_observations_and_mode_state(hrt_abstime current_time, bool &next_active, raptor_status_s &status)
{
	status.subscription_update_vehicle_status = _vehicle_status_sub.update(&_vehicle_status);

	if (status.subscription_update_vehicle_status) {
		timestamp_last_vehicle_status = current_time;
		timestamp_last_vehicle_status_set = true;
	}

	next_active = timestamp_last_vehicle_status_set && _vehicle_status.nav_state == ext_component_mode_id;

	if (!previous_active && next_active) {
		reset();
		PX4_INFO("Resetting Inference Executor (Recurrent State)");

	} else if (previous_active && !next_active) {
		PX4_INFO("inactive");
	}

	bool angular_velocity_update = false;
	status.subscription_update_angular_velocity = _vehicle_angular_velocity_sub.update(&_vehicle_angular_velocity);

	if (status.subscription_update_angular_velocity) {
		timestamp_last_angular_velocity = current_time;
		timestamp_last_angular_velocity_set = true;
		angular_velocity_update = true;
	}

	status.timestamp_last_vehicle_angular_velocity = current_time;
	status.timestamp_sample = _vehicle_angular_velocity.timestamp_sample;

	status.subscription_update_local_position = _vehicle_local_position_sub.update(&_vehicle_local_position);

	if (status.subscription_update_local_position) {
		timestamp_last_local_position = current_time;
		timestamp_last_local_position_set = true;
	}

	status.timestamp_last_vehicle_local_position = current_time;

	status.subscription_update_attitude = _vehicle_attitude_sub.update(&_vehicle_attitude);

	if (status.subscription_update_attitude) {
		timestamp_last_attitude = current_time;
		timestamp_last_attitude_set = true;
	}

	status.timestamp_last_vehicle_attitude = timestamp_last_attitude;

	const bool use_external_reference = reference_source == mc_raptor_intref::ReferenceSource::EXTERNAL;
	update_external_setpoint_subscription(current_time, next_active, use_external_reference, status);

	if (!angular_velocity_update) {
		status.exit_reason = raptor_status_s::EXIT_REASON_NO_ANGULAR_VELOCITY_UPDATE;

		if constexpr(PUBLISH_NON_COMPLETE_STATUS) {
			_raptor_status_pub.publish(status);
		}

		updateArmingCheckReply();
		return false;
	}

	if (!timestamp_last_angular_velocity_set || !timestamp_last_local_position_set || !timestamp_last_attitude_set) {
		status.exit_reason = raptor_status_s::EXIT_REASON_NOT_ALL_OBSERVATIONS_SET;
		status.vehicle_angular_velocity_stale = !timestamp_last_angular_velocity_set;
		status.vehicle_local_position_stale = !timestamp_last_local_position_set;
		status.vehicle_attitude_stale = !timestamp_last_attitude_set;

		if constexpr(PUBLISH_NON_COMPLETE_STATUS) {
			_raptor_status_pub.publish(status);
		}

		can_arm = false;
		updateArmingCheckReply();
		return false;
	}

	if ((current_time - timestamp_last_angular_velocity) > OBSERVATION_TIMEOUT_ANGULAR_VELOCITY) {
		status.exit_reason = raptor_status_s::EXIT_REASON_ANGULAR_VELOCITY_STALE;

		if constexpr(PUBLISH_NON_COMPLETE_STATUS) {
			_raptor_status_pub.publish(status);
		}

		if (!timeout_message_sent) {
			PX4_ERR("angular velocity timeout");
			timeout_message_sent = true;
		}

		can_arm = false;
		updateArmingCheckReply();
		return false;
	}

	if ((current_time - timestamp_last_local_position) > OBSERVATION_TIMEOUT_LOCAL_POSITION) {
		status.exit_reason = raptor_status_s::EXIT_REASON_LOCAL_POSITION_STALE;

		if constexpr(PUBLISH_NON_COMPLETE_STATUS) {
			_raptor_status_pub.publish(status);
		}

		if (!timeout_message_sent) {
			PX4_ERR("local position timeout");
			timeout_message_sent = true;
		}

		can_arm = false;
		updateArmingCheckReply();
		return false;
	}

	position[0] = _vehicle_local_position.x;
	position[1] = _vehicle_local_position.y;
	position[2] = _vehicle_local_position.z;
	linear_velocity[0] = _vehicle_local_position.vx;
	linear_velocity[1] = _vehicle_local_position.vy;
	linear_velocity[2] = _vehicle_local_position.vz;

	return true;
}

void Raptor::update_executor_frequency_statistics(const EXECUTOR_CONFIG::EXECUTOR_STATUS &executor_status, hrt_abstime current_time)
{
	if (executor_status.source == decltype(executor_status.source)::CONTROL) {
		if (executor_status.step_type == decltype(executor_status.step_type)::INTERMEDIATE) {
			last_intermediate_status = executor_status;
			last_intermediate_status_set = true;

		} else if (executor_status.step_type == decltype(executor_status.step_type)::NATIVE) {
			last_native_status = executor_status;
			last_native_status_set = true;
		}
	}

	if (!timestamp_last_policy_frequency_check_set
	    || (current_time - timestamp_last_policy_frequency_check) > POLICY_FREQUENCY_CHECK_INTERVAL) {
		if (timestamp_last_policy_frequency_check_set) {
			if (last_intermediate_status_set) {
				if (!last_intermediate_status.timing_bias.OK || !last_intermediate_status.timing_jitter.OK) {
					PX4_WARN("Raptor: INTERMEDIATE: BIAS %fx JITTER %fx", (double)last_intermediate_status.timing_bias.MAGNITUDE,
						 (double)last_intermediate_status.timing_jitter.MAGNITUDE);

				} else if (ENABLE_CONTROL_FREQUENCY_INFO && policy_frequency_check_counter % POLICY_FREQUENCY_INFO_INTERVAL == 0) {
					PX4_INFO("Raptor: INTERMEDIATE: BIAS %fx JITTER %fx", (double)last_intermediate_status.timing_bias.MAGNITUDE,
						 (double)last_intermediate_status.timing_jitter.MAGNITUDE);
				}
			}

			if (last_native_status_set) {
				if (!last_native_status.timing_bias.OK || !last_native_status.timing_jitter.OK) {
					PX4_WARN("Raptor: NATIVE: BIAS %fx JITTER %fx", (double)last_native_status.timing_bias.MAGNITUDE,
						 (double)last_native_status.timing_jitter.MAGNITUDE);

				} else if (ENABLE_CONTROL_FREQUENCY_INFO && policy_frequency_check_counter % POLICY_FREQUENCY_INFO_INTERVAL == 0) {
					PX4_INFO("Raptor: NATIVE: BIAS %fx JITTER %fx", (double)last_native_status.timing_bias.MAGNITUDE,
						 (double)last_native_status.timing_jitter.MAGNITUDE);
				}
			}
		}

		num_healthy_executor_statii_intermediate = 0;
		num_non_healthy_executor_statii_intermediate = 0;
		num_healthy_executor_statii_native = 0;
		num_non_healthy_executor_statii_native = 0;
		num_statii = 0;
		timestamp_last_policy_frequency_check = current_time;
		timestamp_last_policy_frequency_check_set = true;
		policy_frequency_check_counter++;
	}

	num_statii++;
	num_healthy_executor_statii_intermediate += executor_status.OK && executor_status.source == decltype(executor_status.source)::CONTROL
			&& executor_status.step_type == decltype(executor_status.step_type)::INTERMEDIATE;
	num_non_healthy_executor_statii_intermediate += (!executor_status.OK)
			&& executor_status.source == decltype(executor_status.source)::CONTROL
			&& executor_status.step_type == decltype(executor_status.step_type)::INTERMEDIATE;
	num_healthy_executor_statii_native += executor_status.OK && executor_status.source == decltype(executor_status.source)::CONTROL
			&& executor_status.step_type == decltype(executor_status.step_type)::NATIVE;
	num_non_healthy_executor_statii_native += (!executor_status.OK)
			&& executor_status.source == decltype(executor_status.source)::CONTROL
			&& executor_status.step_type == decltype(executor_status.step_type)::NATIVE;
}

void Raptor::execute_policy_and_publish(hrt_abstime current_time, bool next_active, raptor_status_s &status)
{
	perf_count(_loop_interval_policy_perf);

	rl_tools::inference::applications::l2f::Observation<EXECUTOR_SPEC> observation;
	rl_tools::inference::applications::l2f::Action<EXECUTOR_SPEC> action;
	observe(observation);
	const hrt_abstime nanoseconds = current_time * 1000;
	auto executor_status = rl_tools::control(device, executor, nanoseconds, policy, observation, action, rng);

	if (!executor_status.OK) {
		if (executor_status.TIMESTAMP_INVALID) {
			PX4_ERR("RLtools executor error: Timestamp invalid");
		}

		if (executor_status.LAST_CONTROL_TIMESTAMP_GREATER_THAN_LAST_OBSERVATION_TIMESTAMP) {
			PX4_ERR("RLtools executor error: Last control timestamp %llu greater than last observation timestamp %llu",
				(unsigned long long)executor.executor.last_control_timestamp,
				(unsigned long long)executor.executor.last_observation_timestamp);
		}
	}

	if (executor_status.source != decltype(executor_status.source)::CONTROL) {
		return;
	}

	status.active = next_active;

	raptor_input_s input_msg;
	input_msg.active = status.active;
	static_assert(raptor_input_s::ACTION_DIM == EXECUTOR_CONFIG::OUTPUT_DIM);
	input_msg.timestamp = current_time;
	input_msg.timestamp_sample = _vehicle_angular_velocity.timestamp_sample;

	for (TI dim_i = 0; dim_i < 3; dim_i++) {
		input_msg.position[dim_i] = observation.position[dim_i];
		input_msg.orientation[dim_i] = observation.orientation[dim_i];
		input_msg.linear_velocity[dim_i] = observation.linear_velocity[dim_i];
		input_msg.angular_velocity[dim_i] = observation.angular_velocity[dim_i];
	}

	input_msg.orientation[3] = observation.orientation[3];

	for (TI dim_i = 0; dim_i < EXECUTOR_CONFIG::OUTPUT_DIM; dim_i++) {
		input_msg.previous_action[dim_i] = observation.previous_action[dim_i];
	}

	_raptor_input_pub.publish(input_msg);
	_raptor_status_pub.publish(status);

	actuator_motors_s actuator_motors{};
	actuator_motors.timestamp = hrt_absolute_time();
	actuator_motors.timestamp_sample = _vehicle_angular_velocity.timestamp_sample;

	for (TI action_i = 0; action_i < actuator_motors_s::NUM_CONTROLS; action_i++) {
		if (action_i < EXECUTOR_CONFIG::OUTPUT_DIM) {
			T value = action.action[action_i];
			previous_action[action_i] = value;
			value = (value + 1) / 2;
			constexpr T training_min = 0;
			constexpr T training_max = 1.0;
			const T scaled_value = (training_max - training_min) * value + training_min;
			actuator_motors.control[action_i] = scaled_value;

		} else {
			actuator_motors.control[action_i] = NAN;
		}
	}

	if constexpr(Raptor::REMAP_FROM_CRAZYFLIE) {
		actuator_motors_s temp = actuator_motors;
		temp.control[0] = actuator_motors.control[0];
		temp.control[1] = actuator_motors.control[2];
		temp.control[2] = actuator_motors.control[3];
		temp.control[3] = actuator_motors.control[1];
		actuator_motors = temp;
	}

	if (status.active) {
		_actuator_motors_pub.publish(actuator_motors);
	}

	perf_end(_loop_perf);
	previous_active = next_active;
	update_executor_frequency_statistics(executor_status, current_time);
}
