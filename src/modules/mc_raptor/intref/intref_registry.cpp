#include "intref_registry.hpp"

#include "plugins/plugin_exports.hpp"

#include <string.h>

namespace mc_raptor_intref
{

namespace
{

const TrajectoryPluginDescriptor g_plugins[] {
	g_lissajous_plugin,
	g_circle_plugin
};

constexpr size_t g_plugin_count = sizeof(g_plugins) / sizeof(g_plugins[0]);

} // namespace

const TrajectoryRegistry &TrajectoryRegistry::instance()
{
	static const TrajectoryRegistry registry;
	return registry;
}

const TrajectoryPluginDescriptor *TrajectoryRegistry::findByName(const char *name) const
{
	if (name == nullptr || name[0] == '\0') {
		return nullptr;
	}

	for (size_t i = 0; i < g_plugin_count; ++i) {
		if (strcmp(g_plugins[i].name, name) == 0) {
			return &g_plugins[i];
		}
	}

	return nullptr;
}

const TrajectoryPluginDescriptor *TrajectoryRegistry::findById(uint8_t id) const
{
	for (size_t i = 0; i < g_plugin_count; ++i) {
		if (g_plugins[i].id == id) {
			return &g_plugins[i];
		}
	}

	return nullptr;
}

const TrajectoryPluginDescriptor *TrajectoryRegistry::plugins() const
{
	return g_plugins;
}

size_t TrajectoryRegistry::count() const
{
	return g_plugin_count;
}

} // namespace mc_raptor_intref
