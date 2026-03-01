#pragma once

namespace mc_raptor_intref
{

class IntRefRuntimeManager;

int handle_intref_command(IntRefRuntimeManager &runtime, int argc, char *argv[]);
int handle_mode_command(IntRefRuntimeManager &runtime, int argc, char *argv[]);
void print_intref_usage();
void print_mode_usage();

} // namespace mc_raptor_intref
