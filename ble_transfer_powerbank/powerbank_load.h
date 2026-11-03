#ifndef BEACON_POWERBANK_LOAD_H
#define BEACON_POWERBANK_LOAD_H

#include <stdbool.h>

// Call once on core 0, before CYW43/BTstack initialisation. Returns after core 1
// has set up the flash-write pause mechanism, or false if that setup failed.
bool powerbank_load_start(void);

#endif
