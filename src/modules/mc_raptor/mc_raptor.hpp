#pragma once

#include "intref/intref_runtime.hpp"
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <pthread.h>
#include <stdio.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/register_ext_component_request.h>
#include <uORB/topics/register_ext_component_reply.h>
#include <uORB/topics/unregister_ext_component.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/raptor_status.h>
#include <uORB/topics/raptor_input.h>
#include <uORB/topics/tune_control.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/arming_check_request.h>
#include <uORB/topics/arming_check_reply.h>
#undef OK

#ifdef __PX4_POSIX
#include <rl_tools/operations/cpu.h>
#else
#include <rl_tools/operations/arm.h>
#endif

#include "core/raptor_executor_config.hpp"
namespace rlt = rl_tools;
// #define MC_RAPTOR_EMBED_POLICY // embed policy into firmware instead of loading from sd card.

using namespace time_literals;

class Raptor : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;
	Raptor();
	~Raptor() override;
	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);
	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);
	bool init();
	int print_status() override;
	mc_raptor_intref::IntRefRuntimeManager &intref_runtime()
	{
		return _intref_runtime;
	}
	const mc_raptor_intref::IntRefRuntimeManager &intref_runtime() const
	{
		return _intref_runtime;
	}

private:
#ifdef __PX4_POSIX
	using DEVICE = rlt::devices::DefaultCPU;
#else
	using DEV_SPEC = rlt::devices::DefaultARMSpecification;
	using DEVICE = rlt::devices::arm::OPT<DEV_SPEC>;
#endif
	using TI = typename DEVICE::index_t;
	using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
	using T = float;
	static constexpr uint64_t EXT_COMPONENT_REQUEST_ID = 1337;
	DEVICE device;
	RNG rng;
	hrt_abstime init_time;
	static constexpr TI OBSERVATION_TIMEOUT_ANGULAR_VELOCITY = 10 * 1000;
	static constexpr TI OBSERVATION_TIMEOUT_LOCAL_POSITION = 100 * 1000;
	static constexpr TI OBSERVATION_TIMEOUT_ATTITUDE = 50 * 1000;
	static constexpr TI TRAJECTORY_SETPOINT_TIMEOUT = 200 * 1000;
	static constexpr T RESET_PREVIOUS_ACTION_VALUE = 0; // -1 to 1
	static constexpr bool ENABLE_CONTROL_FREQUENCY_INFO = false;
	T max_position_error = 0.5;
	T max_velocity_error = 1.0;
	void Run() override;
	decltype(register_ext_component_reply_s::mode_id) ext_component_mode_id;
	decltype(register_ext_component_reply_s::arming_check_id) ext_component_arming_check_id;
	enum class FlightModeState : TI {
		UNREGISTERED = 0,
		REGISTERED = 1,
		CONFIGURED = 2
	};
	FlightModeState flightmode_state = FlightModeState::UNREGISTERED;
	bool can_arm = false;
	void updateArmingCheckReply();
	vehicle_local_position_s _vehicle_local_position{};
	vehicle_angular_velocity_s _vehicle_angular_velocity{};
	vehicle_attitude_s _vehicle_attitude{};
	vehicle_status_s _vehicle_status{};
	trajectory_setpoint_s _trajectory_setpoint{};
	hrt_abstime timestamp_last_local_position, timestamp_last_angular_velocity, timestamp_last_attitude, _timestamp_last_external_trajectory_setpoint,
		    timestamp_last_vehicle_status;
	bool timestamp_last_local_position_set = false, timestamp_last_angular_velocity_set = false, timestamp_last_attitude_set = false,
	     _timestamp_last_external_trajectory_setpoint_set = false, timestamp_last_vehicle_status_set = false;
	bool timeout_message_sent = false;
	bool _previous_external_trajectory_setpoint_stale = false;
	bool previous_active = false;
	T position[3];
	T linear_velocity[3];
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _register_ext_component_reply_sub{ORB_ID(register_ext_component_reply)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _arming_check_request_sub{ORB_ID(arming_check_request)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<raptor_status_s> _raptor_status_pub{ORB_ID(raptor_status)};
	uORB::Publication<raptor_input_s> _raptor_input_pub{ORB_ID(raptor_input)};
	uORB::Publication<trajectory_setpoint_s> _intref_trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<tune_control_s> _tune_control_pub{ORB_ID(tune_control)};
	uORB::Publication<register_ext_component_request_s> _register_ext_component_request_pub{ORB_ID(register_ext_component_request)};
	uORB::Publication<unregister_ext_component_s> _unregister_ext_component_pub{ORB_ID(unregister_ext_component)};
	uORB::Publication<vehicle_control_mode_s> _config_control_setpoints_pub{ORB_ID(config_control_setpoints)};
	uORB::Publication<arming_check_reply_s> _arming_check_reply_pub{ORB_ID(arming_check_reply)};
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
	perf_counter_t	_loop_interval_policy_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval_policy")};
	using EXECUTOR_CONFIG = mc_raptor_core::RaptorExecutorConfig<DEVICE, TI>;
	using EXECUTOR_SPEC = EXECUTOR_CONFIG::EXECUTOR_SPEC;
	rl_tools::inference::applications::L2F<EXECUTOR_SPEC> executor;
#ifdef MC_RAPTOR_EMBED_POLICY
	const decltype(mc_raptor_core::checkpoint::actor::module) &policy = mc_raptor_core::checkpoint::actor::module;
#else
	EXECUTOR_CONFIG::POLICY policy;
#endif
	static constexpr TI CHECKPOINT_NAME_LENGTH = 100;
	char checkpoint_name[CHECKPOINT_NAME_LENGTH] = "n/a";
#ifdef MC_RAPTOR_EMBED_POLICY
	bool test_policy();
#else
	bool test_policy(FILE *f, TI input_offset, TI output_offset);
#endif
	void reset();
	void observe(rl_tools::inference::applications::l2f::Observation<EXECUTOR_SPEC> &observation);
	void fill_status_from_intref_snapshot(const mc_raptor_intref::IntRefStatusSnapshot &snapshot, raptor_status_s &status) const;
	void update_runtime_config_from_params(raptor_status_s &status);
	void update_external_setpoint_subscription(hrt_abstime current_time, bool next_active, bool use_external_reference,
			raptor_status_s &status);
	void step_internal_reference_if_needed(hrt_abstime current_time, bool next_active, raptor_status_s &status);
	void apply_external_stale_logic(hrt_abstime current_time, bool next_active, raptor_status_s &status);
	bool handle_shutdown_request();
	void process_mode_registration();
	void initialize_status_message(hrt_abstime current_time, raptor_status_s &status) const;
	void update_trajectory_setpoint_timing_stats(raptor_status_s &status);
	bool update_observations_and_mode_state(hrt_abstime current_time, bool &next_active, raptor_status_s &status);
	void execute_policy_and_publish(hrt_abstime current_time, bool next_active, raptor_status_s &status);
	void update_executor_frequency_statistics(const EXECUTOR_CONFIG::EXECUTOR_STATUS &executor_status, hrt_abstime current_time);
	static constexpr bool REMAP_FROM_CRAZYFLIE = true; // crazyflie output order to PX4 quad X order
	static constexpr TI POLICY_INTERVAL_WARNING_THRESHOLD = 100; // us
	static constexpr TI POLICY_FREQUENCY_CHECK_INTERVAL = 1000 * 1000; // 1s
	static constexpr TI POLICY_FREQUENCY_INFO_INTERVAL = 10; // 10 x POLICY_FREQUENCY_CHECK_INTERVAL = 10x
	static constexpr TI POLICY_CONTROL_FREQUENCY_TRAINING = 100;
	TI num_statii;
	TI num_healthy_executor_statii_intermediate, num_non_healthy_executor_statii_intermediate, num_healthy_executor_statii_native,
	num_non_healthy_executor_statii_native;
	EXECUTOR_CONFIG::EXECUTOR_STATUS last_intermediate_status, last_native_status;
	bool last_intermediate_status_set, last_native_status_set;
	TI policy_frequency_check_counter;
	hrt_abstime timestamp_last_policy_frequency_check;
	bool timestamp_last_policy_frequency_check_set = false;
	static constexpr TI NUM_TRAJECTORY_SETPOINT_DTS = 100;
	int32_t trajectory_setpoint_dts[NUM_TRAJECTORY_SETPOINT_DTS];
	TI trajectory_setpoint_dt_index = 0;
	TI trajectory_setpoint_dt_max_since_reset = 0;
	bool trajectory_setpoint_dts_full = false;
	static constexpr TI TRAJECTORY_SETPOINT_INVALID_COUNT_WARNING_INTERVAL = 100;
	TI trajectory_setpoint_invalid_count = 0;
	float previous_action[EXECUTOR_SPEC::OUTPUT_DIM];
	pthread_mutex_t _intref_runtime_mutex {};
	mc_raptor_intref::IntRefRuntimeManager _intref_runtime {};
	mc_raptor_intref::InternalReferenceConfigured internal_reference {mc_raptor_intref::InternalReferenceConfigured::NONE};
	mc_raptor_intref::ReferenceSource reference_source {mc_raptor_intref::ReferenceSource::EXTERNAL};
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::IMU_GYRO_RATEMAX>) _param_imu_gyro_ratemax,
		(ParamBool<px4::params::MC_RAPTOR_VERBOS>) _param_mc_raptor_verbose,
		(ParamBool<px4::params::MC_RAPTOR_OFFB>) _param_mc_raptor_offboard,
		(ParamFloat<px4::params::MC_RAPTOR_TRNS_T>) _param_mc_raptor_transition_time,
		(ParamFloat<px4::params::MC_RAPTOR_TRNS_Y>) _param_mc_raptor_transition_yaw_rate,
		(ParamInt<px4::params::MC_RAPTOR_INTREF>) _param_mc_raptor_intref
	)
};
