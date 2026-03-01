#include "mc_raptor.hpp"
#include "intref/intref_cli.hpp"
#undef OK

#include <containers/LockGuard.hpp>

#include <rl_tools/inference/applications/l2f/operations_generic.h>

#include <stdlib.h>
#include <string.h>

// Keep policy-dependent implementation in a single translation unit to avoid
// multiple definitions from generated policy symbols.
#include "core/raptor_checkpoint_io.cpp"
#include "core/raptor_mode_lifecycle.cpp"
#include "core/raptor_reference_pipeline.cpp"
#include "core/raptor_control_pipeline.cpp"

ModuleBase::Descriptor Raptor::desc{task_spawn, custom_command, print_usage};

Raptor::Raptor(): ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	timestamp_last_angular_velocity_set = false;
	timestamp_last_local_position_set = false;
	timestamp_last_attitude_set = false;
	_timestamp_last_external_trajectory_setpoint = 0;
	_timestamp_last_external_trajectory_setpoint_set = false;
	timestamp_last_vehicle_status_set = false;
	_previous_external_trajectory_setpoint_stale = false;
	previous_active = false;
	timeout_message_sent = false;
	timestamp_last_policy_frequency_check_set = false;
	last_intermediate_status_set = false;
	last_native_status_set = false;
	policy_frequency_check_counter = 0;
	flightmode_state = FlightModeState::UNREGISTERED;
	can_arm = false;
	trajectory_setpoint_dt_index = 0;
	trajectory_setpoint_dts_full = false;
	trajectory_setpoint_invalid_count = 0;
	trajectory_setpoint_dt_max_since_reset = 0;
	internal_reference = mc_raptor_intref::InternalReferenceConfigured::NONE;
	reference_source = mc_raptor_intref::ReferenceSource::EXTERNAL;
	_intref_runtime.reset();
	const int mutex_init_result = pthread_mutex_init(&_intref_runtime_mutex, nullptr);

	if (mutex_init_result != 0) {
		PX4_ERR("failed to initialize intref runtime mutex (%d)", mutex_init_result);
		abort();
	}

	_actuator_motors_pub.advertise();
	_tune_control_pub.advertise();
}

void Raptor::reset()
{
	trajectory_setpoint_dt_index = 0;
	trajectory_setpoint_dts_full = false;
	trajectory_setpoint_invalid_count = 0;
	trajectory_setpoint_dt_max_since_reset = 0;
	_timestamp_last_external_trajectory_setpoint = 0;
	_timestamp_last_external_trajectory_setpoint_set = false;
	_previous_external_trajectory_setpoint_stale = false;

	for (TI action_i = 0; action_i < EXECUTOR_SPEC::OUTPUT_DIM; action_i++) {
		previous_action[action_i] = RESET_PREVIOUS_ACTION_VALUE;
	}

	rlt::reset(device, executor, policy, rng);
}

Raptor::~Raptor()
{
	const int mutex_destroy_result = pthread_mutex_destroy(&_intref_runtime_mutex);

	if (mutex_destroy_result != 0) {
		PX4_ERR("failed to destroy intref runtime mutex (%d)", mutex_destroy_result);
	}

	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

void Raptor::Run()
{
	if (handle_shutdown_request()) {
		return;
	}

	process_mode_registration();

	perf_count(_loop_interval_perf);
	perf_begin(_loop_perf);
	const hrt_abstime current_time = hrt_absolute_time();

	raptor_status_s status{};
	initialize_status_message(current_time, status);
	update_runtime_config_from_params(status);
	update_trajectory_setpoint_timing_stats(status);

	bool next_active = false;

	if (!update_observations_and_mode_state(current_time, next_active, status)) {
		return;
	}

	step_internal_reference_if_needed(current_time, next_active, status);

	if ((current_time - timestamp_last_attitude) > OBSERVATION_TIMEOUT_ATTITUDE) {
		status.exit_reason = raptor_status_s::EXIT_REASON_ATTITUDE_STALE;

		if constexpr(PUBLISH_NON_COMPLETE_STATUS) {
			_raptor_status_pub.publish(status);
		}

		if (!timeout_message_sent) {
			PX4_ERR("attitude timeout");
			timeout_message_sent = true;
		}

		can_arm = false;
		updateArmingCheckReply();
		return;
	}

	timeout_message_sent = false;
	can_arm = true;
	updateArmingCheckReply();

	apply_external_stale_logic(current_time, next_active, status);
	execute_policy_and_publish(current_time, next_active, status);
}

int Raptor::task_spawn(int argc, char *argv[])
{
	Raptor *instance = new Raptor();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			instance->ScheduleNow();
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int Raptor::print_status()
{
	mc_raptor_intref::IntRefStatusSnapshot snapshot {};

	{
		LockGuard intref_lock{_intref_runtime_mutex};
		snapshot = _intref_runtime.statusSnapshot();
	}

	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	perf_print_counter(_loop_interval_policy_perf);
	PX4_INFO_RAW("Checkpoint: %s\n", checkpoint_name);
	PX4_INFO_RAW("intref configured: %s (%d)\n", mc_raptor_intref::configured_mode_name(snapshot.configured_mode),
		     (int)snapshot.configured_mode);
	PX4_INFO_RAW("reference mode: %s (%d)\n", mc_raptor_intref::reference_mode_name(snapshot.reference_mode),
		     (int)snapshot.reference_mode);
	PX4_INFO_RAW("reference source: %s (%d)\n", mc_raptor_intref::reference_source_name(snapshot.reference_source),
		     (int)snapshot.reference_source);
	PX4_INFO_RAW("active trajectory: id=%u name=%s\n", (unsigned)snapshot.active_trajectory_id,
		     snapshot.active_trajectory_name[0] == '\0' ? "None" : snapshot.active_trajectory_name);
	PX4_INFO_RAW("hold: active=%s pending_capture=%s\n", snapshot.hold_active ? "true" : "false",
		     snapshot.hold_pending_capture ? "true" : "false");
	PX4_INFO_RAW("hold anchor: x=%.3f y=%.3f z=%.3f yaw=%.3f\n", (double)snapshot.hold_position[0],
		     (double)snapshot.hold_position[1], (double)snapshot.hold_position[2], (double)snapshot.hold_yaw);
	PX4_INFO_RAW("transition: active=%s progress=%.3f remaining=%.3fs\n",
		     snapshot.transition_active ? "true" : "false",
		     (double)snapshot.transition_progress, (double)snapshot.transition_remaining_s);
	if (snapshot.circle_center_valid) {
		PX4_INFO_RAW("circle center: x=%.3f y=%.3f z=%.3f\n",
			     (double)snapshot.circle_center_position[0],
			     (double)snapshot.circle_center_position[1],
			     (double)snapshot.circle_center_position[2]);
	}
	PX4_INFO_RAW("setpoint: pos=(%.3f %.3f %.3f) vel=(%.3f %.3f %.3f) yaw=%.3f yawspeed=%.3f\n",
		     (double)_trajectory_setpoint.position[0], (double)_trajectory_setpoint.position[1], (double)_trajectory_setpoint.position[2],
		     (double)_trajectory_setpoint.velocity[0], (double)_trajectory_setpoint.velocity[1], (double)_trajectory_setpoint.velocity[2],
		     (double)_trajectory_setpoint.yaw, (double)_trajectory_setpoint.yawspeed);
	return 0;
}

int Raptor::custom_command(int argc, char *argv[])
{
	if (argc >= 1 && strcmp(argv[0], "help") == 0) {
		return print_usage(nullptr);
	}

	if (argc >= 1 && strcmp(argv[0], "mode") == 0) {
		Raptor *instance = get_instance<Raptor>(desc);

		if (instance == nullptr) {
			PX4_ERR("mc_raptor is not running");
			return PX4_ERROR;
		}

		LockGuard intref_lock{instance->_intref_runtime_mutex};
		return mc_raptor_intref::handle_mode_command(instance->intref_runtime(), argc, argv);
	}

	if (argc >= 1 && strcmp(argv[0], "intref") == 0) {
		Raptor *instance = get_instance<Raptor>(desc);

		if (instance == nullptr) {
			PX4_ERR("mc_raptor is not running");
			return PX4_ERROR;
		}

		LockGuard intref_lock{instance->_intref_runtime_mutex};
		return mc_raptor_intref::handle_intref_command(instance->intref_runtime(), argc, argv);
	}

	return print_usage("unknown command");
}

int Raptor::print_usage(const char *reason)
{
	if (reason) {
		PX4_INFO_RAW("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
RAPTOR Policy Flight Mode

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_raptor", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("mode", "Reference source mode control");
	mc_raptor_intref::print_mode_usage();
	PRINT_MODULE_USAGE_COMMAND_DESCR("intref", "Internal reference trajectory control");
	mc_raptor_intref::print_intref_usage();
	PRINT_MODULE_USAGE_COMMAND_DESCR("help", "Print this help");

	return 0;
}

extern "C" __EXPORT int mc_raptor_main(int argc, char *argv[])
{
	return ModuleBase::main(Raptor::desc, argc, argv);
}
