// Built against a future ABI: the host must refuse it before calling init().
#include <stdio.h>
#include "mnu_api.h"
static int init(const mnu_host_api_t *, const mnu_core_info_t *) { printf("bad_abi: init\n"); return 0; }
static void unload(int) { printf("bad_abi: unload\n"); }
static int ui(const mnu_event_t *) { return MNU_ACT_STAY; }
static const mnu_module_t module = { sizeof(mnu_module_t), MNU_ABI_VERSION + 1, 0, "bad_abi", "1", init, unload, ui, NULL, NULL, NULL, NULL, MNU_FAMILY_NONE, NULL };
extern "C" MNU_EXPORT const mnu_module_t *mnu_entry(void) { return &module; }
