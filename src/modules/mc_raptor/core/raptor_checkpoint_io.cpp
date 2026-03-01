#include "raptor_checkpoint_io.hpp"

#include "../mc_raptor.hpp"

#include <rl_tools/inference/applications/l2f/operations_generic.h>
#include <rl_tools/persist/backends/tar/operations_posix.h>
#include <rl_tools/nn/optimizers/adam/instance/persist.h>
#include <rl_tools/nn/layers/gru/persist.h>
#include <rl_tools/nn/layers/dense/persist.h>
#include <rl_tools/nn_models/sequential/persist.h>

#include <sys/stat.h>
#include <string.h>

#ifdef MC_RAPTOR_EMBED_POLICY
bool Raptor::test_policy()
{
#else
bool Raptor::test_policy(FILE *f, TI input_offset, TI output_offset)
{
#endif
	using namespace rl_tools::inference::applications::l2f;
#ifndef RL_TOOLS_DISABLE_TEST
	using POLICY = EXECUTOR_CONFIG::POLICY_TEST;
	POLICY::template Buffer<false> buffers_test;
	POLICY::State<false> policy_state_test;
	rl_tools::Tensor<rl_tools::tensor::Specification<EXECUTOR_CONFIG::TYPE_POLICY::DEFAULT, TI,
			rl_tools::tensor::Shape<TI, 1, POLICY::OUTPUT_SHAPE::LAST>, false>> test_output;
	rl_tools::Mode<rl_tools::mode::Evaluation<>> mode;
	using EXAMPLE_INPUT_SPEC = mc_raptor_core::checkpoint::example::input::SPEC;
	using EXAMPLE_OUTPUT_SPEC = mc_raptor_core::checkpoint::example::output::SPEC;
	float acc = 0;
	uint64_t num_values = 0;
	rl_tools::inference::applications::l2f::Action<EXECUTOR_SPEC> action;

	for (TI batch_i = 0; batch_i < EXECUTOR_CONFIG::TEST_BATCH_SIZE_ACTUAL; batch_i++) {
		rl_tools::reset(device, policy, policy_state_test, rng);

		for (TI step_i = 0; step_i < EXECUTOR_CONFIG::TEST_SEQUENCE_LENGTH_ACTUAL; step_i++) {
#ifdef MC_RAPTOR_EMBED_POLICY
			const auto step_input = rl_tools::view(device, mc_raptor_core::checkpoint::example::input::container, step_i);
			const auto batch_input = rl_tools::view_range(device, step_input, batch_i, rlt::tensor::ViewSpec<0, 1> {});
			const auto step_output_target = rl_tools::view(device, mc_raptor_core::checkpoint::example::output::container, step_i);
			const auto batch_output_target = rl_tools::view_range(device, step_output_target, batch_i, rlt::tensor::ViewSpec<0, 1> {});
#else
			rl_tools::Tensor<rl_tools::tensor::Specification<EXECUTOR_CONFIG::TYPE_POLICY::DEFAULT, TI,
				rl_tools::tensor::Shape<TI, 1, EXAMPLE_INPUT_SPEC::SHAPE::LAST>, false>> batch_input;
			rl_tools::Tensor<rl_tools::tensor::Specification<EXECUTOR_CONFIG::TYPE_POLICY::DEFAULT, TI,
				rl_tools::tensor::Shape<TI, 1, EXAMPLE_OUTPUT_SPEC::SHAPE::LAST>, false>> batch_output_target;
			fseek(f, input_offset + (step_i * EXAMPLE_INPUT_SPEC::STRIDE::FIRST
					   + batch_i * EXAMPLE_INPUT_SPEC::STRIDE::template GET<1>) * sizeof(EXAMPLE_INPUT_SPEC::T), SEEK_SET);
			fread(batch_input._data, sizeof(EXAMPLE_INPUT_SPEC::T), EXAMPLE_INPUT_SPEC::SHAPE::LAST, f);
			fseek(f, output_offset + (step_i * EXAMPLE_OUTPUT_SPEC::STRIDE::FIRST
					    + batch_i * EXAMPLE_OUTPUT_SPEC::STRIDE::template GET<1>) * sizeof(EXAMPLE_OUTPUT_SPEC::T), SEEK_SET);
			fread(batch_output_target._data, sizeof(EXAMPLE_OUTPUT_SPEC::T), EXAMPLE_OUTPUT_SPEC::SHAPE::LAST, f);
#endif
			rl_tools::utils::assert_exit(device, !rl_tools::is_nan(device, batch_input), "input is nan");
			rl_tools::evaluate_step(device, policy, batch_input, policy_state_test, test_output, buffers_test, rng, mode);
			rl_tools::utils::assert_exit(device, !rl_tools::is_nan(device, test_output), "output is nan");

			for (TI action_i = 0; action_i < EXECUTOR_CONFIG::OUTPUT_DIM; action_i++) {
				acc += rl_tools::math::abs(device.math,
					   rl_tools::get(device, test_output, 0, action_i) - rl_tools::get(device, batch_output_target, 0, action_i));
				num_values += 1;
				rl_tools::utils::assert_exit(device, !rl_tools::math::is_nan(device.math, acc), "output is nan");

				if (batch_i == 0 && step_i == EXECUTOR_CONFIG::TEST_SEQUENCE_LENGTH_ACTUAL - 1) {
					action.action[action_i] = rl_tools::get(device, test_output, 0, action_i);
				}
			}
		}
	}

	const float abs_diff = acc / num_values;
	PX4_INFO("Checkpoint test diff: %f", (double)abs_diff);

	for (TI output_i = 0; output_i < EXECUTOR_CONFIG::OUTPUT_DIM; output_i++) {
		PX4_INFO("output[%d]: %f", (int)output_i, (double)action.action[output_i]);
	}

	constexpr float EPSILON = 1e-5;

	if (abs_diff >= EPSILON) {
		PX4_ERR("Checkpoint test failed with diff %.10f", (double)abs_diff);
		return false;
	}

	PX4_INFO("Checkpoint test passed with diff %.10f", (double)abs_diff);
	return true;
#else
	return 0;
#endif
}

bool Raptor::init()
{
	init_time = hrt_absolute_time();

	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle_angular_velocity_sub callback registration failed");
		return false;
	}

#ifndef MC_RAPTOR_EMBED_POLICY
	const char *path = PX4_STORAGEDIR "/raptor/policy.tar";
	struct stat st;

	if (stat(path, &st) == 0) {
		PX4_INFO("Policy checkpoint %s exists", path);
		FILE *f = fopen(path, "rb");

		if (!f) {
			PX4_ERR("Failed to open %s: %s", path, strerror(errno));
			return false;
		}

		if (fseek(f, 0, SEEK_END) != 0) {
			PX4_ERR("fseek failed: %s", strerror(errno));
			fclose(f);
			return false;
		}

		const long size = ftell(f);

		if (size < 0) {
			PX4_ERR("ftell failed: %s", strerror(errno));
			fclose(f);
			return false;
		}

		rewind(f);
		bool successfully_loaded = false;
		using SPEC = rlt::persist::backends::tar::ReaderGroupSpecification<TI, rlt::persist::backends::tar::PosixFileData<TI>>;
		rlt::persist::backends::tar::ReaderGroup<SPEC> reader_group;
		reader_group.data.f = f;
		reader_group.data.size = size;
		auto actor_group = rlt::get_group(device, reader_group, "actor");
		successfully_loaded = rlt::load(device, policy, actor_group);
		constexpr TI METADATA_BUFFER_SIZE = 256;
		char metadata_buffer[METADATA_BUFFER_SIZE];
		TI read_size = 0;
		rlt::persist::backends::tar::get(device, reader_group.data, "actor/meta", metadata_buffer, METADATA_BUFFER_SIZE, read_size);
		TI checkpoint_name_position = 0;
		TI checkpoint_name_len = 0;

		if (!rlt::persist::backends::tar::seek_in_metadata(device, metadata_buffer, METADATA_BUFFER_SIZE, "checkpoint_name",
				checkpoint_name_position, checkpoint_name_len)) {
			PX4_ERR("Failed to get checkpoint name from metadata");
			fclose(f);
			return false;
		}

		strncpy(checkpoint_name, metadata_buffer + checkpoint_name_position, CHECKPOINT_NAME_LENGTH);
		checkpoint_name[checkpoint_name_len < CHECKPOINT_NAME_LENGTH ? checkpoint_name_len : CHECKPOINT_NAME_LENGTH - 1] = '\0';

		if (!successfully_loaded) {
			PX4_ERR("Failed to load policy from file %s", path);
			fclose(f);
			return false;
		}

		PX4_INFO("Policy loaded from file %s", path);
		TI input_offset = 0;
		TI input_size = 0;
		rlt::persist::backends::tar::seek(device, reader_group.data, "example/input/data", input_offset, input_size);
		PX4_INFO("Input offset: %d", (int)input_offset);
		TI output_offset = 0;
		TI output_size = 0;
		rlt::persist::backends::tar::seek(device, reader_group.data, "example/output/data", output_offset, output_size);
		PX4_INFO("Output offset: %d", (int)output_offset);

		if (!test_policy(f, input_offset, output_offset)) {
			PX4_ERR("Checkpoint test failed");
			fclose(f);
			return false;
		}

		fclose(f);

	} else {
		PX4_INFO("File %s does not exist", path);
		return false;
	}
#else
	strncpy(checkpoint_name, mc_raptor_core::checkpoint::meta::name, CHECKPOINT_NAME_LENGTH);

	if (!test_policy()) {
		PX4_ERR("Checkpoint test failed");
		return false;
	}
#endif

	PX4_INFO("Checkpoint name: %s", checkpoint_name);

	register_ext_component_request_s register_ext_component_request{};
	register_ext_component_request.timestamp = hrt_absolute_time();
	strncpy(register_ext_component_request.name, "RAPTOR", sizeof(register_ext_component_request.name) - 1);
	register_ext_component_request.request_id = Raptor::EXT_COMPONENT_REQUEST_ID;
	register_ext_component_request.px4_ros2_api_version = 1;
	register_ext_component_request.register_arming_check = true;
	register_ext_component_request.register_mode = true;
	register_ext_component_request.enable_replace_internal_mode = _param_mc_raptor_offboard.get();
	register_ext_component_request.replace_internal_mode = vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
	_register_ext_component_request_pub.publish(register_ext_component_request);

	const int32_t imu_gyro_ratemax = _param_imu_gyro_ratemax.get();

	if (imu_gyro_ratemax % POLICY_CONTROL_FREQUENCY_TRAINING != 0) {
		PX4_WARN("IMU_GYRO_RATEMAX=%d Hz is not a multiple of the training frequency (%d Hz)",
			 (int)imu_gyro_ratemax, (int)POLICY_CONTROL_FREQUENCY_TRAINING);
	}

	const int32_t force_sync_native = imu_gyro_ratemax / POLICY_CONTROL_FREQUENCY_TRAINING;
	executor.executor.force_sync_native = force_sync_native;
	executor.executor.force_sync_native_initialized = true;
	PX4_INFO("IMU_GYRO_RATEMAX=%d Hz", (int)imu_gyro_ratemax);
	PX4_INFO("POLICY_CONTROL_FREQUENCY_TRAINING=%d Hz", (int)POLICY_CONTROL_FREQUENCY_TRAINING);
	PX4_INFO("Setting force_sync_native = %d Hz / %d Hz = %d",
		 (int)imu_gyro_ratemax, (int)POLICY_CONTROL_FREQUENCY_TRAINING, (int)force_sync_native);

	reset();
	return true;
}
