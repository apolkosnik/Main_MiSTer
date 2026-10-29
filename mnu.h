// Host side of the per-core menu module (.mnu) mechanism.
// See mnu_api.h for the ABI a module implements.
#ifndef MNU_H
#define MNU_H

#include <stdint.h>
#include "mnu_api.h"

int         mnu_active();
uint32_t    mnu_flags();
const char *mnu_file();          // path of the loaded module, "" if none

// 1 if a module would load with this rbf: menu_module=1 and a .mnu exists for it.
// Used by the core browser to mark such cores.
int  mnu_available(const char *rbf_path);

// Startup, first half: right after the INI is parsed, open <rbf>.mnu if it
// exists and read the family it runs the core as (MNU_FAMILY_*), which
// mnu_family() reports from then on. init() waits for mnu_try_load().
uint32_t mnu_probe(const char *rbf_arg, const char *xml);
uint32_t mnu_family();
// Startup, second half: remember the core paths and init the probed module,
// or find and load <rbf>.mnu if nothing was probed. Called at the end of
// user_io_init(). Returns 1 when a module is active.
int  mnu_try_load(const char *rbf_arg, const char *xml);
// Load a specific module file (unloads the current one first). UI coroutine or startup only.
// Both loaders are no-ops unless menu_module=1 is set in MiSTer.ini.
int  mnu_load(const char *mnu_path);
// For the F9 key in a stock main menu: the module is loaded already (1), or
// the last one loads again, or <rbf>.mnu is looked up once more. 0 = nothing to show.
int  mnu_load_again();
// Unload the current module, idempotent. MNU_UNLOAD_CORE_EXIT skips dlclose and
// leaves the menu state alone because the process is about to exec a new core.
void mnu_unload(int reason);

// Deferred requests, safe from any context. mnu_service() executes them on the
// UI coroutine; HandleUI() calls it once per pass.
void mnu_cmd(const char *cmd);   // "mnu_load <path>" | "mnu_unload" | "mnu_reload"
void mnu_service();

// Poll coroutine hook, called from user_io_poll().
void mnu_poll();
// IDE requests 1-3 (ide_io), for the module's ide_request().
void mnu_ide_request(int port, int req);
// The Minimig family's service table (mnu_minimig.cpp).
const void *mnu_minimig_api();

// menu.cpp glue
int  mnu_ui(const mnu_event_t *ev);        // returns MNU_ACT_*
int  mnu_file_selected(const char *path);  // returns MNU_ACT_*
void mnu_osd_opened();                     // MENU_MODULE1 entered
void mnu_osd_closed();                     // MENU_NONE1 entered
void mnu_set_reopen();                     // open the module/stock menu on the next idle pass
int  mnu_take_reopen();                    // consume the reopen request

#endif
