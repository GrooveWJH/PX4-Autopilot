#include "raptor_mode_lifecycle.hpp"

#include "../mc_raptor.hpp"

void Raptor::updateArmingCheckReply()
{
	if (flightmode_state == FlightModeState::CONFIGURED && _arming_check_request_sub.updated()) {
		arming_check_request_s arming_check_request;
		_arming_check_request_sub.copy(&arming_check_request);
		arming_check_reply_s arming_check_reply{};
		arming_check_reply.timestamp = hrt_absolute_time();
		arming_check_reply.request_id = arming_check_request.request_id;
		arming_check_reply.registration_id = ext_component_arming_check_id;
		arming_check_reply.health_component_index = arming_check_reply.HEALTH_COMPONENT_INDEX_NONE;
		arming_check_reply.num_events = 0;
		arming_check_reply.can_arm_and_run = can_arm;
		arming_check_reply.mode_req_angular_velocity = true;
		arming_check_reply.mode_req_local_position = true;
		arming_check_reply.mode_req_attitude = true;
		arming_check_reply.mode_req_local_alt = true;
		arming_check_reply.mode_req_home_position = false;
		arming_check_reply.mode_req_mission = false;
		arming_check_reply.mode_req_global_position = false;
		arming_check_reply.mode_req_prevent_arming = false;
		arming_check_reply.mode_req_manual_control = false;
		_arming_check_reply_pub.publish(arming_check_reply);
	}
}

bool Raptor::handle_shutdown_request()
{
	if (!should_exit()) {
		return false;
	}

	_vehicle_angular_velocity_sub.unregisterCallback();

	if (flightmode_state >= FlightModeState::REGISTERED) {
		unregister_ext_component_s unregister_ext_component{};
		unregister_ext_component.timestamp = hrt_absolute_time();
		strncpy(unregister_ext_component.name, "RAPTOR", sizeof(unregister_ext_component.name) - 1);
		unregister_ext_component.arming_check_id = ext_component_arming_check_id;
		unregister_ext_component.mode_id = ext_component_mode_id;
		unregister_ext_component.mode_executor_id = -1;
		_unregister_ext_component_pub.publish(unregister_ext_component);
	}

	ScheduleClear();
	exit_and_cleanup(desc);
	return true;
}

void Raptor::process_mode_registration()
{
	register_ext_component_reply_s register_ext_component_reply;

	if (_register_ext_component_reply_sub.update(&register_ext_component_reply)
	    && register_ext_component_reply.request_id == Raptor::EXT_COMPONENT_REQUEST_ID
	    && register_ext_component_reply.success) {
		ext_component_arming_check_id = register_ext_component_reply.arming_check_id;
		ext_component_mode_id = register_ext_component_reply.mode_id;
		flightmode_state = FlightModeState::REGISTERED;
		PX4_INFO("Raptor mode registration successful, arming_check_id: %d, mode_id: %d",
			 ext_component_arming_check_id, ext_component_mode_id);
	}

	if (flightmode_state != FlightModeState::REGISTERED) {
		return;
	}

	vehicle_control_mode_s config_control_setpoints{};
	config_control_setpoints.timestamp = hrt_absolute_time();
	config_control_setpoints.source_id = ext_component_mode_id;
	config_control_setpoints.flag_multicopter_position_control_enabled = false;
	config_control_setpoints.flag_control_manual_enabled = false;
	config_control_setpoints.flag_control_offboard_enabled = false;
	config_control_setpoints.flag_control_position_enabled = false;
	config_control_setpoints.flag_control_climb_rate_enabled = false;
	config_control_setpoints.flag_control_allocation_enabled = false;
	config_control_setpoints.flag_control_termination_enabled = true;
	_config_control_setpoints_pub.publish(config_control_setpoints);
	flightmode_state = FlightModeState::CONFIGURED;
	PX4_INFO("Raptor mode configuration sent");
}
