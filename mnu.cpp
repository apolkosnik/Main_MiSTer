// Host side of the per-core menu module (.mnu) mechanism.
//
// A module is a shared object next to the core's .rbf. It is dlopen'ed once
// the core is configured, takes over the OSD main menu while loaded, and is
// dlclose'd again on request, after which the stock menu is back exactly as
// it was. See mnu_api.h for the contract.

#include <stdio.h>
#include <string.h>
#include <dlfcn.h>
#include <strings.h>

#include "mnu.h"
#include "mnu_path.h"
#include "menu.h"
#include "osd.h"
#include "user_io.h"
#include "file_io.h"
#include "hardware.h"
#include "spi.h"
#include "cfg.h"
#include "mnu_minimig.h"

extern const char *version;

static void              *handle = NULL;
static const mnu_module_t *mod = NULL;

static char rbf_arg_s[MNU_PATH_MAX] = {};   // as passed to user_io_init()
static char xml_s[MNU_PATH_MAX] = {};
static char rbf_abs[MNU_PATH_MAX] = {};
static char mod_path[MNU_PATH_MAX] = {};
static mnu_core_info_t info = {};

static int in_osd = 0;          // module menu has been shown since the OSD last closed
static int sel_pending = 0;     // select_file() called, no result yet
static int reopen = 0;          // open the menu again on the next idle pass
static int unload_req = 0;      // module asked to be unloaded

// deferred commands from /dev/MiSTer_cmd
static char pending_load[MNU_PATH_MAX] = {};
static int  pending_unload = 0;
static int  pending_reload = 0;

// opened by mnu_probe() before the core is set up, init() still to come
static void              *probe_handle = NULL;
static const mnu_module_t *probe_mod = NULL;
static char probe_path[MNU_PATH_MAX] = {};

// the family the core runs as; set by the probe, kept for the process' life
static uint32_t family = MNU_FAMILY_NONE;

// A module built against an older, shorter mnu_module_t may not have the
// optional callbacks at all: only touch a field the module's size covers.
#define MNU_HAS(m, field) ((m)->size >= offsetof(mnu_module_t, field) + sizeof((m)->field))
#define MNU_MIN_SIZE      (offsetof(mnu_module_t, ui) + sizeof(mod->ui))

int mnu_active() { return mod != NULL; }
uint32_t mnu_flags() { return mod ? mod->flags : 0; }
const char *mnu_file() { return mod ? mod_path : ""; }

/////////////////////////////////////////////////////////////////////////
// host API handed to the module

// A module menu's title carries a star, like the core browser marks cores
// that load a module, so the user can tell it from the stock menu. It has to
// go in here: the OSD sends the title's pixels along with every row write,
// so a title changed after the rows are drawn shows up only on the next redraw.
static void h_osd_set_title(const char *title, int arrow)
{
	char s[64];
	snprintf(s, sizeof(s), "%s*", title ? title : "");
	OsdSetTitle(s, arrow);
}
static void h_osd_set_size(int rows) { OsdSetSize(rows > 8 ? 16 : 8); }
static int  h_osd_get_size(void) { return OsdGetSize(); }
static void h_osd_clear(void) { OsdClear(); }
static void h_osd_write(int row, const char *text, int invert, int stipple)
{
	if (row < 0 || row > 255) return;
	OsdWrite((unsigned char)row, text ? text : "", invert ? 1 : 0, stipple ? 1 : 0);
}
static void h_menu_draw_begin(void) { menu_mod_draw_begin(); }
static void h_menu_write(int row, const char *text, int invert, int stipple, int arrow)
{
	if (row < 0 || row > 255) return;
	menu_mod_write(row, text ? text : "", invert ? 1 : 0, stipple ? 1 : 0, arrow);
}
static int  h_menu_draw_end(void) { return menu_mod_draw_end(); }
static void h_menu_reset_scroll(void) { menu_mod_reset_scroll(); }

static void h_info(const char *text, int timeout_ms) { Info(text ? text : "", timeout_ms); }
static void h_info_message(const char *text, int timeout_ms, const char *title)
{
	InfoMessage(text ? text : "", timeout_ms, title ? title : "Message");
}
static void h_progress(const char *title, const char *text, int current, int max)
{
	ProgressMessage(title, text, current, max);
}

static void h_select_file(const char *start_path, const char *extensions, uint32_t options)
{
	if (!mod) return;
	sel_pending = 1;
	menu_mod_select_file(start_path ? start_path : "", extensions ? extensions : "", options);
}

static const char *h_confstr(int index) { return user_io_get_confstr(index); }
static uint32_t h_status_get(const char *opt, int ex) { return opt ? user_io_status_get(opt, ex) : 0; }
static void h_status_set(const char *opt, uint32_t value, int ex) { if (opt) user_io_status_set(opt, value, ex); }
static int  h_status_save(const char *filename) { return filename ? user_io_status_save(filename) : 0; }
static int  h_file_tx(const char *path, unsigned char index, int opensave, int mute, int composite, uint32_t load_addr)
{
	return path ? user_io_file_tx(path, index, (char)opensave, (char)mute, (char)composite, load_addr) : 0;
}
static int  h_file_mount(const char *path, unsigned char index) { return user_io_file_mount(path ? path : "", index); }
static unsigned char h_ext_idx(const char *path, const char *extensions)
{
	static char p[MNU_PATH_MAX], e[64];
	snprintf(p, sizeof(p), "%s", path ? path : "");
	snprintf(e, sizeof(e), "%s", extensions ? extensions : "");
	return user_io_ext_idx(p, e);
}
static void h_kbd_reset(int assert) { user_io_set_kbd_reset(assert); }
static void h_send_buttons(int force) { user_io_send_buttons((char)force); }

static uint16_t h_spi_uio_cmd(uint16_t cmd) { return spi_uio_cmd(cmd); }
static uint8_t  h_spi_uio_cmd8(uint8_t cmd, uint8_t parm) { return spi_uio_cmd8(cmd, parm); }
static uint16_t h_spi_uio_cmd16(uint8_t cmd, uint16_t parm) { return spi_uio_cmd16(cmd, parm); }
static void     h_spi_uio_cmd32(uint8_t cmd, uint32_t parm, int wide) { spi_uio_cmd32(cmd, parm, wide); }

static const char *h_home_dir(void) { return HomeDir(); }
static const char *h_root_dir(void) { return getRootDir(); }
static const char *h_full_path(const char *name) { return getFullPath(name ? name : ""); }
// The file calls build their full path in the buffer full_path() returns:
// a path from full_path() is copied first.
static const char *h_path_copy(const char *path)
{
	static char p[MNU_PATH_MAX];
	snprintf(p, sizeof(p), "%s", path);
	return p;
}
static int h_file_exists(const char *path) { return path ? FileExists(h_path_copy(path)) : 0; }
static int h_file_load(const char *path, void *buf, int size) { return path ? FileLoad(h_path_copy(path), buf, size) : 0; }
static int h_file_save(const char *path, const void *buf, int size) { return path ? FileSave(h_path_copy(path), (void*)buf, size) : 0; }
static int h_config_load(const char *name, void *buf, int size) { return name ? FileLoadConfig(name, buf, size) : 0; }
static int h_config_save(const char *name, const void *buf, int size) { return name ? FileSaveConfig(name, (void*)buf, size) : 0; }

static uint32_t h_get_timer(uint32_t ms) { return (uint32_t)GetTimer(ms); }
static int      h_check_timer(uint32_t t) { return CheckTimer(t) ? 1 : 0; }

static void h_request_unload(void) { unload_req = 1; }
static int  h_osd_is_visible(void) { return user_io_osd_is_visible() ? 1 : 0; }

static int h_ar_name(int ar, char *buf, int len) { return menu_mod_ar_name(ar, buf, len); }
static int h_ar_next(int ar, int minus) { return menu_mod_ar_next(ar, minus); }

static const void *h_family_api(uint32_t f)
{
	if (f == MNU_FAMILY_MINIMIG && is_minimig()) return mnu_minimig_api();
	return NULL;
}

static const mnu_host_api_t host_api =
{
	sizeof(mnu_host_api_t),
	MNU_ABI_VERSION,

	h_osd_set_title,
	h_osd_set_size,
	h_osd_get_size,
	h_osd_clear,
	h_osd_write,
	h_menu_draw_begin,
	h_menu_write,
	h_menu_draw_end,
	h_menu_reset_scroll,

	h_info,
	h_info_message,
	h_progress,

	h_select_file,

	h_confstr,
	h_status_get,
	h_status_set,
	h_status_save,
	h_file_tx,
	h_file_mount,
	h_ext_idx,
	h_kbd_reset,
	h_send_buttons,

	h_spi_uio_cmd,
	h_spi_uio_cmd8,
	h_spi_uio_cmd16,
	h_spi_uio_cmd32,

	h_home_dir,
	h_root_dir,
	h_full_path,
	h_file_exists,
	h_file_load,
	h_file_save,
	h_config_load,
	h_config_save,

	h_get_timer,
	h_check_timer,

	h_request_unload,
	h_osd_is_visible,

	h_family_api,

	h_ar_name,
	h_ar_next,
};

/////////////////////////////////////////////////////////////////////////
// lifecycle

// dlopen a module and check its descriptor; init() is not called yet
static const mnu_module_t *mnu_open(const char *mnu_path, void **handle_out)
{
	void *h = dlopen(mnu_path, RTLD_NOW | RTLD_LOCAL);
	if (!h)
	{
		printf("MNU: dlopen(%s) failed: %s\n", mnu_path, dlerror());
		return NULL;
	}

	mnu_entry_fn entry = (mnu_entry_fn)dlsym(h, MNU_ENTRY_SYMBOL);
	if (!entry)
	{
		printf("MNU: %s has no %s(): %s\n", mnu_path, MNU_ENTRY_SYMBOL, dlerror());
		dlclose(h);
		return NULL;
	}

	const mnu_module_t *m = entry();
	if (!m)
	{
		printf("MNU: %s: %s() returned NULL\n", mnu_path, MNU_ENTRY_SYMBOL);
		dlclose(h);
		return NULL;
	}
	if (m->abi_version != MNU_ABI_VERSION)
	{
		printf("MNU: %s: ABI %u, host has %u\n", mnu_path, m->abi_version, MNU_ABI_VERSION);
		dlclose(h);
		return NULL;
	}
	if (m->size < MNU_MIN_SIZE || !m->init || !m->unload || !m->ui)
	{
		printf("MNU: %s: descriptor incomplete (size %u, need %u)\n", mnu_path, m->size, (unsigned)MNU_MIN_SIZE);
		dlclose(h);
		return NULL;
	}

	*handle_out = h;
	return m;
}

static uint32_t mnu_desc_family(const mnu_module_t *m)
{
	return MNU_HAS(m, family) ? m->family : MNU_FAMILY_NONE;
}

// init() an opened module and make it the active one
static int mnu_start(void *h, const mnu_module_t *m, const char *mnu_path)
{
	snprintf(mod_path, sizeof(mod_path), "%s", mnu_path);
	memset(&info, 0, sizeof(info));
	info.size = sizeof(info);
	info.rbf_path = rbf_abs;
	info.mnu_path = mod_path;
	info.xml_path = xml_s[0] ? xml_s : NULL;
	info.core_name = user_io_get_core_name();
	info.orig_name = user_io_get_core_name(1);
	info.mister_version = version + 5;

	in_osd = 0;
	sel_pending = 0;
	unload_req = 0;

	int ret = m->init(&host_api, &info);
	if (ret)
	{
		printf("MNU: %s: init() returned %d, using the stock menu\n", mnu_path, ret);
		dlclose(h);
		mod_path[0] = 0;
		return 0;
	}

	handle = h;
	mod = m;
	if (mnu_desc_family(m) != family)
		printf("MNU: %s is for family %u, the core runs as %u\n", mnu_path, mnu_desc_family(m), family);
	printf("MNU: loaded %s (%s %s, flags 0x%X)\n", mnu_path, m->name ? m->name : "?", m->version ? m->version : "", m->flags);
	return 1;
}

static const char *mnu_find(const char *rbf_arg, const char *xml);

int mnu_load_again()
{
	if (mod) return 1;
	if (!cfg.menu_module) return 0;
	if (mod_path[0])
	{
		char path[MNU_PATH_MAX];   // mnu_load() overwrites mod_path, so hand it a copy
		snprintf(path, sizeof(path), "%s", mod_path);
		return mnu_load(path);
	}
	const char *path = mnu_find(rbf_arg_s, xml_s[0] ? xml_s : NULL);
	return path ? mnu_load(path) : 0;
}

int mnu_load(const char *mnu_path)
{
	if (!mnu_path || !mnu_path[0]) return 0;
	if (!cfg.menu_module)
	{
		printf("MNU: menu modules are disabled, set menu_module=1 in MiSTer.ini to use %s\n", mnu_path);
		return 0;
	}
	if (mod) mnu_unload(MNU_UNLOAD_COMMAND);

	void *h = NULL;
	const mnu_module_t *m = mnu_open(mnu_path, &h);
	return m ? mnu_start(h, m, mnu_path) : 0;
}

// The module file that belongs to an absolute rbf path, or NULL.
static const char *find_module(const char *rbf)
{
	static char cand[2][MNU_PATH_MAX];
	int n = mnu_candidates(rbf, cand, 2);
	for (int i = 0; i < n; i++)
	{
		if (FileExists(cand[i], 0)) return cand[i];
	}
	return NULL;
}

int mnu_available(const char *rbf_path)
{
	if (!cfg.menu_module || !rbf_path || !rbf_path[0]) return 0;

	// the menu core never loads a module, see mnu_try_load()
	const char *base = strrchr(rbf_path, '/');
	base = base ? base + 1 : rbf_path;
	if (!strcasecmp(base, "menu.rbf")) return 0;

	return find_module(rbf_path) != NULL;
}

// The module next to the rbf, if modules are enabled and this core may have one.
static const char *mnu_find(const char *rbf_arg, const char *xml)
{
	snprintf(rbf_arg_s, sizeof(rbf_arg_s), "%s", rbf_arg ? rbf_arg : "");
	snprintf(xml_s, sizeof(xml_s), "%s", xml ? xml : "");
	mnu_resolve_rbf(rbf_arg_s, getStorageDir(0), getRootDir(), rbf_abs, sizeof(rbf_abs));

	if (!cfg.menu_module) return NULL;
	if (!rbf_abs[0]) return NULL;
	if (is_menu()) return NULL;
	if (user_io_core_type() != CORE_TYPE_8BIT) return NULL;

	const char *found = find_module(rbf_abs);
	if (found) printf("MNU: found %s\n", found);
	return found;
}

uint32_t mnu_probe(const char *rbf_arg, const char *xml)
{
	if (probe_mod || mod) return family;

	const char *path = mnu_find(rbf_arg, xml);
	if (!path) return family;

	probe_mod = mnu_open(path, &probe_handle);
	if (!probe_mod) return family;

	snprintf(probe_path, sizeof(probe_path), "%s", path);
	family = mnu_desc_family(probe_mod);
	if (family != MNU_FAMILY_NONE) printf("MNU: %s runs the core as family %u\n", path, family);
	return family;
}

uint32_t mnu_family() { return family; }

int mnu_try_load(const char *rbf_arg, const char *xml)
{
	if (probe_mod)
	{
		// opened by mnu_probe(): only init() is left
		const mnu_module_t *m = probe_mod;
		void *h = probe_handle;
		probe_mod = NULL;
		probe_handle = NULL;
		if (mod) mnu_unload(MNU_UNLOAD_COMMAND);
		return mnu_start(h, m, probe_path);
	}

	const char *path = mnu_find(rbf_arg, xml);
	return path ? mnu_load(path) : 0;
}

void mnu_unload(int reason)
{
	if (!mod) return;

	const mnu_module_t *m = mod;
	void *h = handle;
	int owned = menu_mod_owns_osd();

	printf("MNU: unloading %s (reason %d)\n", mod_path, reason);

	// Clear first so anything the module calls from unload() sees no module.
	mod = NULL;
	handle = NULL;
	in_osd = 0;
	sel_pending = 0;
	unload_req = 0;

	m->unload(reason);

	if (reason == MNU_UNLOAD_CORE_EXIT) return;   // process is about to exec, nothing to restore

	dlclose(h);

	// Back to the stock menu. If the module menu was on screen, reopen the
	// stock one in its place instead of leaving a blank OSD.
	if (owned)
	{
		menu_mod_restore();
		reopen = 1;
	}
}

/////////////////////////////////////////////////////////////////////////
// deferred commands

void mnu_cmd(const char *cmd)
{
	if (!cmd) return;
	if (!strncmp(cmd, "mnu_load ", 9))
	{
		const char *p = cmd + 9;
		while (*p == ' ' || *p == '\t') p++;
		snprintf(pending_load, sizeof(pending_load), "%s", p);
	}
	else if (!strcmp(cmd, "mnu_unload")) pending_unload = 1;
	else if (!strcmp(cmd, "mnu_reload")) pending_reload = 1;
	else printf("MNU: unknown command: %s\n", cmd);
}

void mnu_service()
{
	if (pending_unload)
	{
		pending_unload = 0;
		mnu_unload(MNU_UNLOAD_COMMAND);
	}

	if (pending_reload)
	{
		pending_reload = 0;
		char path[MNU_PATH_MAX];
		snprintf(path, sizeof(path), "%s", mod_path);
		if (path[0])
		{
			mnu_unload(MNU_UNLOAD_COMMAND);
			mnu_load(path);
		}
		else printf("MNU: nothing to reload\n");
	}

	if (pending_load[0])
	{
		char path[MNU_PATH_MAX];
		snprintf(path, sizeof(path), "%s", pending_load);
		pending_load[0] = 0;
		mnu_load(path[0] == '/' ? path : getFullPath(path));
	}

	if (unload_req && mod)
	{
		unload_req = 0;
		mnu_unload(MNU_UNLOAD_REQUEST);
	}
}

void mnu_ide_request(int port, int req)
{
	if (mod && MNU_HAS(mod, ide_request) && mod->ide_request) mod->ide_request(port, req);
}

void mnu_poll()
{
	if (mod && (mod->flags & MNU_FLAG_WANT_POLL) && MNU_HAS(mod, poll) && mod->poll) mod->poll();
}

/////////////////////////////////////////////////////////////////////////
// menu.cpp glue

int mnu_ui(const mnu_event_t *ev)
{
	if (!mod) return MNU_ACT_CLOSE;
	if (unload_req) return MNU_ACT_UNLOAD;
	return mod->ui(ev);
}

int mnu_file_selected(const char *path)
{
	sel_pending = 0;
	if (!mod) return MNU_ACT_CLOSE;
	if (MNU_HAS(mod, file_selected) && mod->file_selected) return mod->file_selected(path ? path : "");
	return MNU_ACT_REDRAW;
}

void mnu_osd_opened()
{
	in_osd = 1;
	if (sel_pending)
	{
		// Back in the module menu without a file_selected(): the selector was cancelled.
		sel_pending = 0;
		if (mod && MNU_HAS(mod, file_cancelled) && mod->file_cancelled) mod->file_cancelled();
	}
}

void mnu_osd_closed()
{
	sel_pending = 0;
	if (!in_osd) return;
	in_osd = 0;
	if (mod && MNU_HAS(mod, osd_closed) && mod->osd_closed) mod->osd_closed();
}

void mnu_set_reopen() { reopen = 1; }

int mnu_take_reopen()
{
	int r = reopen;
	reopen = 0;
	return r;
}
