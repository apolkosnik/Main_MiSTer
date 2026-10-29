// Uses poll() and request_unload() through the vtable.
#include <stdio.h>
#include "mnu_api.h"
static const mnu_host_api_t *host;
static int polled = 0;
static int init(const mnu_host_api_t *h, const mnu_core_info_t *) { host = h; polled = 0; printf("poller: init\n"); return 0; }
static void unload(int reason) { printf("poller: unload %d, polled %d\n", reason, polled); }
static int ui(const mnu_event_t *ev)
{
	if (ev->select) host->request_unload();
	return MNU_ACT_STAY;
}
static void poll(void) { polled++; }
static const mnu_module_t module = { sizeof(mnu_module_t), MNU_ABI_VERSION, MNU_FLAG_WANT_POLL, "poller", "1", init, unload, ui, NULL, NULL, NULL, poll, MNU_FAMILY_NONE, NULL };
extern "C" MNU_EXPORT const mnu_module_t *mnu_entry(void) { return &module; }
