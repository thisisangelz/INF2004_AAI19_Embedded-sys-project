// Preserve direct USB console output as JSON without appending it to the report.
#include <stdio.h>
#include "sim.h"
#define main firmware_main
#define printf(...) sim_console_printf(__VA_ARGS__)
#include "getting_there.c"
#undef printf
#undef main
