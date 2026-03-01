#pragma once

#include <px4_platform_common/defines.h>

#include <rl_tools/nn/layers/standardize/operations_generic.h>
#include <rl_tools/nn/layers/dense/operations_arm/opt.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/nn/layers/gru/operations_generic.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>

#include <rl_tools/inference/executor/executor.h>
#include <rl_tools/inference/applications/l2f/l2f.h>

#include "../blob/policy.h"

namespace rlt = rl_tools;

namespace mc_raptor_core
{
namespace checkpoint
{
namespace actor = rlt::checkpoint::actor;
namespace example = rlt::checkpoint::example;
namespace meta = rlt::checkpoint::meta;
} // namespace checkpoint

template<typename DEVICE_T, typename TI_T>
struct RaptorExecutorConfig {
	using DEVICE = DEVICE_T;
	using TI = TI_T;

	using ACTOR_TYPE_ORIGINAL = checkpoint::actor::TYPE;
	using POLICY_TEST = typename checkpoint::actor::TYPE::template CHANGE_BATCH_SIZE<TI, 1>::template CHANGE_SEQUENCE_LENGTH<TI, 1>;
	using POLICY_BATCH_SIZE = typename ACTOR_TYPE_ORIGINAL::template CHANGE_BATCH_SIZE<TI, 1>;

#ifdef MC_RAPTOR_EMBED_POLICY
	using POLICY = POLICY_BATCH_SIZE;
#else
	using POLICY = typename POLICY_BATCH_SIZE::template CHANGE_CAPABILITY<rlt::nn::capability::Forward<false, false>>;
#endif

	using TYPE_POLICY = typename POLICY::TYPE_POLICY;

#if defined(__PX4_POSIX)
	// Relax warning levels for Gazebo SITL (250 Hz IMU isn't a clean multiple of 100 Hz training frequency).
	struct WARNING_LEVELS: rlt::inference::executor::WarningLevelsDefault<TYPE_POLICY> {
		using T = typename TYPE_POLICY::DEFAULT;
		static constexpr T INTERMEDIATE_TIMING_JITTER_HIGH_THRESHOLD = 2.0;
		static constexpr T INTERMEDIATE_TIMING_JITTER_LOW_THRESHOLD = 0.5;
		static constexpr T INTERMEDIATE_TIMING_BIAS_HIGH_THRESHOLD = 2.0;
		static constexpr T INTERMEDIATE_TIMING_BIAS_LOW_THRESHOLD = 0.5;
		static constexpr T NATIVE_TIMING_JITTER_HIGH_THRESHOLD = 2.0;
		static constexpr T NATIVE_TIMING_JITTER_LOW_THRESHOLD = 0.5;
		static constexpr T NATIVE_TIMING_BIAS_HIGH_THRESHOLD = 2.0;
		static constexpr T NATIVE_TIMING_BIAS_LOW_THRESHOLD = 0.5;
	};
#else
	struct WARNING_LEVELS: rlt::inference::executor::WarningLevelsDefault<TYPE_POLICY> {
		using T = typename TYPE_POLICY::DEFAULT;
		static constexpr T NATIVE_TIMING_JITTER_HIGH_THRESHOLD = 1.5;
		static constexpr T NATIVE_TIMING_JITTER_LOW_THRESHOLD = 0.5;
	};
#endif

	using TIMESTAMP = hrt_abstime;
	static constexpr TI OUTPUT_DIM = 4;
	static constexpr TI TEST_SEQUENCE_LENGTH_ACTUAL = 5;
	static constexpr TI TEST_BATCH_SIZE_ACTUAL = 2;

	static constexpr TI ACTION_HISTORY_LENGTH = 1;
	static constexpr TI CONTROL_INTERVAL_INTERMEDIATE_NS = 2.5 * 1000 * 1000; // Inference is 500 Hz
	static constexpr TI CONTROL_INTERVAL_NATIVE_NS = 10 * 1000 * 1000; // Training is 100 Hz
	static constexpr TI TIMING_STATS_NUM_STEPS = 100;
	static constexpr bool FORCE_SYNC_INTERMEDIATE = true;
	static constexpr bool FORCE_SYNC_NATIVE_RUNTIME = true;
	static constexpr TI FORCE_SYNC_NATIVE = 8;
	static constexpr bool DYNAMIC_ALLOCATION = false;

	using EXECUTOR_SPEC =
		rlt::inference::applications::l2f::Specification<TYPE_POLICY, TI, TIMESTAMP, ACTION_HISTORY_LENGTH,
		OUTPUT_DIM, POLICY, CONTROL_INTERVAL_INTERMEDIATE_NS, CONTROL_INTERVAL_NATIVE_NS, FORCE_SYNC_INTERMEDIATE,
		FORCE_SYNC_NATIVE, FORCE_SYNC_NATIVE_RUNTIME, WARNING_LEVELS, DYNAMIC_ALLOCATION>;
	using EXECUTOR_STATUS = rlt::inference::executor::Status<typename EXECUTOR_SPEC::EXECUTOR_SPEC>;
};

} // namespace mc_raptor_core
