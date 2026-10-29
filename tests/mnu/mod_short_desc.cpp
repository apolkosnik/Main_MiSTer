// Descriptor that ends after ui(), like a module built against a shorter
// mnu_module_t. The optional slots hold poison: the host must never read them.
#include <stdio.h>
#include <stddef.h>
#include "mnu_api.h"
static int init(const mnu_host_api_t *, const mnu_core_info_t *) { printf("short: init\n"); return 0; }
static void unload(int reason) { printf("short: unload %d\n", reason); }
static int ui(const mnu_event_t *) { return MNU_ACT_STAY; }
static const mnu_module_t module =
{
	offsetof(mnu_module_t, file_selected),   // size covers init/unload/ui only
	MNU_ABI_VERSION, 0, "short", "1",
	init, unload, ui,
	(int (*)(const char *))0x1,              // poison
	(void (*)(void))0x1,
	(void (*)(void))0x1,
	(void (*)(void))0x1,
	0x5A5A5A5A,                              // family
	(void (*)(int, int))0x1,                 // ide_request
};
extern "C" MNU_EXPORT const mnu_module_t *mnu_entry(void) { return &module; }
