#pragma once

#include "intref_types.hpp"

#include <stddef.h>

namespace mc_raptor_intref
{

class TrajectoryRegistry
{
public:
	static const TrajectoryRegistry &instance();

	const TrajectoryPluginDescriptor *findByName(const char *name) const;
	const TrajectoryPluginDescriptor *findById(uint8_t id) const;

	const TrajectoryPluginDescriptor *plugins() const;
	size_t count() const;

private:
	TrajectoryRegistry() = default;
};

} // namespace mc_raptor_intref
