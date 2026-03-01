#include "intref_cli.hpp"

#include "intref_registry.hpp"
#include "intref_runtime.hpp"

#include <px4_platform_common/module.h>

#include <stdio.h>
#include <string.h>

namespace mc_raptor_intref
{

namespace
{

void print_trajectory_summary(const TrajectoryCommand &command)
{
	PX4_INFO("trajectory=%s id=%u", command.name, (unsigned)command.id);
	PX4_INFO_RAW("args:");

	for (uint8_t i = 0; i < command.arg_count; ++i) {
		PX4_INFO_RAW(" %.4f", (double)command.args[i]);
	}

	PX4_INFO_RAW("\n");
}

void print_snapshot(const IntRefRuntimeManager &runtime)
{
	const IntRefStatusSnapshot snapshot = runtime.statusSnapshot();
	PX4_INFO("configured mode: %s (%u)", configured_mode_name(snapshot.configured_mode), (unsigned)snapshot.configured_mode);
	PX4_INFO("reference mode: %s (%u)", reference_mode_name(snapshot.reference_mode), (unsigned)snapshot.reference_mode);
	PX4_INFO("reference source: %s (%u)", reference_source_name(snapshot.reference_source), (unsigned)snapshot.reference_source);
	PX4_INFO("active trajectory: id=%u name=%s", (unsigned)snapshot.active_trajectory_id,
		 snapshot.active_trajectory_name[0] == '\0' ? "None" : snapshot.active_trajectory_name);
	PX4_INFO("hold: active=%s pending_capture=%s", snapshot.hold_active ? "true" : "false",
		 snapshot.hold_pending_capture ? "true" : "false");
	PX4_INFO("hold anchor: x=%.3f y=%.3f z=%.3f yaw=%.3f", (double)snapshot.hold_position[0],
		 (double)snapshot.hold_position[1], (double)snapshot.hold_position[2], (double)snapshot.hold_yaw);
}

} // namespace

int handle_mode_command(IntRefRuntimeManager &runtime, int argc, char *argv[])
{
	if (argc < 2 || strcmp(argv[0], "mode") != 0) {
		PX4_ERR("Usage: mc_raptor mode <show|set>");
		return PX4_ERROR;
	}

	if (strcmp(argv[1], "show") == 0) {
		if (argc != 2) {
			PX4_ERR("Usage: mc_raptor mode show");
			return PX4_ERROR;
		}

		print_snapshot(runtime);
		return PX4_OK;
	}

	if (strcmp(argv[1], "set") == 0) {
		if (argc != 3) {
			PX4_ERR("Usage: mc_raptor mode set <extref|intref|hold>");
			return PX4_ERROR;
		}

		ReferenceMode mode = ReferenceMode::EXTREF;

		if (strcmp(argv[2], "extref") == 0) {
			mode = ReferenceMode::EXTREF;

		} else if (strcmp(argv[2], "intref") == 0) {
			mode = ReferenceMode::INTREF;

		} else if (strcmp(argv[2], "hold") == 0) {
			mode = ReferenceMode::HOLD;

		} else {
			PX4_ERR("invalid mode: %s", argv[2]);
			return PX4_ERROR;
		}

		char error[128] {};

		if (!runtime.setReferenceMode(mode, error, sizeof(error))) {
			PX4_ERR("%s", error[0] != '\0' ? error : "failed to set mode");
			return PX4_ERROR;
		}

		PX4_INFO("reference mode set to: %s", reference_mode_name(mode));
		return PX4_OK;
	}

	PX4_ERR("unknown mode subcommand: %s", argv[1]);
	print_mode_usage();
	return PX4_ERROR;
}

int handle_intref_command(IntRefRuntimeManager &runtime, int argc, char *argv[])
{
	if (argc < 2 || strcmp(argv[0], "intref") != 0) {
		PX4_ERR("Usage: mc_raptor intref <list|show|help|set>");
		return PX4_ERROR;
	}

	const TrajectoryRegistry &registry = TrajectoryRegistry::instance();
	const char *subcommand = argv[1];

	if (strcmp(subcommand, "list") == 0) {
		const TrajectoryPluginDescriptor *plugins = registry.plugins();
		const size_t plugin_count = registry.count();
		PX4_INFO("available trajectories (%u):", (unsigned)plugin_count);

		for (size_t i = 0; i < plugin_count; ++i) {
			PX4_INFO("- %s (id=%u): %s", plugins[i].name, (unsigned)plugins[i].id, plugins[i].description);
		}

		return PX4_OK;
	}

	if (strcmp(subcommand, "help") == 0) {
		if (argc == 2) {
			print_intref_usage();
			return PX4_OK;
		}

		const TrajectoryPluginDescriptor *plugin = registry.findByName(argv[2]);

		if (plugin == nullptr) {
			PX4_ERR("unknown trajectory: %s", argv[2]);
			return PX4_ERROR;
		}

		PX4_INFO("%s", plugin->usage);
		return PX4_OK;
	}

	if (strcmp(subcommand, "show") == 0) {
		print_snapshot(runtime);
		const TrajectoryCommand lissajous = runtime.configuredLissajousCommand();
		const TrajectoryCommand circle = runtime.configuredCircleCommand();
		PX4_INFO("configured lissajous:");
		print_trajectory_summary(lissajous);
		PX4_INFO("configured circle:");
		print_trajectory_summary(circle);

		if (runtime.hasSelectedInternalTrajectory()) {
			PX4_INFO("selected internal trajectory:");
			print_trajectory_summary(runtime.selectedInternalTrajectory());
		}

		return PX4_OK;
	}

	if (strcmp(subcommand, "set") == 0) {
		if (argc < 3) {
			PX4_ERR("Usage: mc_raptor intref set <trajectory_name> [args...]");
			return PX4_ERROR;
		}

		const char *trajectory_name = argv[2];
		const TrajectoryPluginDescriptor *plugin = registry.findByName(trajectory_name);

		if (plugin == nullptr) {
			PX4_ERR("unknown trajectory: %s", trajectory_name);
			return PX4_ERROR;
		}

		if (plugin->parse_validate == nullptr) {
			PX4_ERR("trajectory plugin has no parser: %s", trajectory_name);
			return PX4_ERROR;
		}

		char error[128] {};
		TrajectoryCommand command {};

		if (!plugin->parse_validate(argc - 3, &argv[3], command, error, sizeof(error))) {
			PX4_ERR("%s", error[0] != '\0' ? error : "invalid trajectory arguments");
			return PX4_ERROR;
		}

		if (!runtime.setTrajectoryCommand(command, error, sizeof(error))) {
			PX4_ERR("%s", error[0] != '\0' ? error : "failed to set trajectory");
			return PX4_ERROR;
		}

		PX4_INFO("intref trajectory configured: %s", trajectory_name);
		PX4_INFO("note: source mode unchanged, use 'mc_raptor mode set intref' to activate internal reference");
		print_trajectory_summary(command);
		return PX4_OK;
	}

	PX4_ERR("unknown intref subcommand: %s", subcommand);
	print_intref_usage();
	return PX4_ERROR;
}

void print_mode_usage()
{
	PX4_INFO_RAW("  mode show\n");
	PX4_INFO_RAW("  mode set <extref|intref|hold>\n");
}

void print_intref_usage()
{
	PX4_INFO_RAW("  intref list\n");
	PX4_INFO_RAW("  intref show\n");
	PX4_INFO_RAW("  intref help [trajectory_name]\n");
	PX4_INFO_RAW("  intref set lissajous <A> <B> <C> <fa> <fb> <fc> <duration> <ramp>\n");
	PX4_INFO_RAW("  intref set circle <radius_m> <speed_mps> <ramp_s>\n");
	PX4_INFO_RAW("  intref set <trajectory_name> <args...>\n");
}

} // namespace mc_raptor_intref
