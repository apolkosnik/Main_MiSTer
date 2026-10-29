// init() refuses: the host must dlclose without calling unload().
#include <stdio.h>
#include "mnu_api.h"
static int init(const mnu_host_api_t *, const mnu_core_info_t *) { printf("bad_init: init\n"); return 7; }
static void unload(int) { printf("bad_init: unload\n"); }
static int ui(const mnu_event_t *) { return MNU_ACT_STAY; }
static const mnu_module_t module = { sizeof(mnu_module_t), MNU_ABI_VERSION, 0, "bad_init", "1", init, unload, ui, NULL, NULL, NULL, NULL, MNU_FAMILY_NONE, NULL };
extern "C" MNU_EXPORT const mnu_module_t *mnu_entry(void) { return &module; }
