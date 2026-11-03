#ifndef HOST_CRITICAL_SECTION_H
#define HOST_CRITICAL_SECTION_H
#include <assert.h>
typedef struct { unsigned held; } critical_section_t;
static inline void critical_section_init(critical_section_t *c) { c->held = 0; }
static inline void critical_section_enter_blocking(critical_section_t *c)
{ assert(!c->held); c->held = 1; }
static inline void critical_section_exit(critical_section_t *c)
{ assert(c->held); c->held = 0; }
#endif
