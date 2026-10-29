/*
 * mnu_api.h - ABI between Main_MiSTer and a per-core menu module (.mnu)
 *
 * The mechanism is off by default; menu_module=1 in MiSTer.ini enables it.
 *
 * A .mnu file is an ELF shared object, built -fPIC -shared with the same
 * arm-none-linux-gnueabihf toolchain as MiSTer, that sits next to the core:
 *
 *     /media/fat/_Console/NES_20261010.rbf
 *     /media/fat/_Console/NES_20261010.mnu      exact match, tried first
 *     /media/fat/_Console/NES.mnu               date code stripped, fallback
 *
 * The module exports exactly one symbol, mnu_entry(). Everything else is
 * private to the module. MiSTer is not linked -rdynamic, so a module cannot
 * call host functions directly; it talks back through the mnu_host_api_t
 * vtable handed to init(). Plain C ABI on both sides.
 *
 * Lifecycle, as seen by the module:
 *
 *   mnu_entry()       right after the INI is parsed, before the core is set
 *                     up: the host reads the descriptor (family) only
 *   init()            once, right after the core is configured
 *   ui()              every HandleUI pass while the module menu owns the OSD
 *   file_selected()   the file selector returned (empty path = cleared)
 *   file_cancelled()  the file selector was left without a choice
 *   osd_closed()      the OSD went away for any reason
 *   poll()            every user_io_poll pass (only with MNU_FLAG_WANT_POLL)
 *   ide_request()     the core raised an IDE request the host's controller
 *                     does not make (1-3), on the poll coroutine
 *   unload()          once, before dlclose() or before the process execs a new core
 *
 * ui(), file_selected(), file_cancelled() and osd_closed() run on the UI
 * coroutine, poll() on the poll coroutine. Both are the same OS thread, so
 * no locking is needed, but a module must never block or yield.
 * ide_request() may run a whole exchange with the core before it returns,
 * as the host's own IDE command handling does.
 *
 * Core families: a module whose descriptor names a family (MNU_FAMILY_*) has
 * the host run the core as that family whatever the core calls itself (the
 * Minimig protocol: keyboard, floppies, configuration, IDE), and gets the
 * family's own services from host->family_api(); see mnu_minimig.h.
 *
 * Two things the host does on top of the module, so the user can always tell
 * the two menus apart and get back to the stock one: a module menu shows a
 * star after its title (the core browser marks cores that load a module the
 * same way), and F9 is reserved: in the module menu it unloads the
 * module (MNU_UNLOAD_USER) and shows the stock menu in its place; in the
 * stock main menu it loads the module again and shows its menu. ui() never
 * sees F9.
 */
#ifndef MNU_API_H
#define MNU_API_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MNU_ABI_VERSION      1
#define MNU_ENTRY_SYMBOL     "mnu_entry"
#define MNU_EXPORT           __attribute__((visibility("default")))

/* mnu_module_t.flags */
#define MNU_FLAG_KEY_REPEAT       0x0001  /* auto-repeat held keys like the file selector does */
#define MNU_FLAG_WANT_POLL        0x0002  /* call poll() every pass of user_io_poll() */
#define MNU_FLAG_HANDLE_MENU_KEY  0x0004  /* pass the Menu/Esc key to ui() instead of closing the OSD */
#define MNU_FLAG_SELECT_KEEPS_OSD 0x0008  /* keep the OSD open across file_selected(); default hides it first */

/* mnu_module_t.family */
#define MNU_FAMILY_NONE      0   /* the core is what its name says */
#define MNU_FAMILY_MINIMIG   1   /* a Minimig-protocol core; host->family_api(MNU_FAMILY_MINIMIG) */

/* unload reasons */
enum {
	MNU_UNLOAD_REQUEST   = 0,  /* module returned MNU_ACT_UNLOAD or called request_unload() */
	MNU_UNLOAD_COMMAND   = 1,  /* "mnu_unload" / "mnu_reload" / "mnu_load" on /dev/MiSTer_cmd */
	MNU_UNLOAD_CORE_EXIT = 2,  /* this process is about to exec a new core; no dlclose follows */
	MNU_UNLOAD_ERROR     = 3,  /* host decided the module misbehaved (e.g. ui() returned garbage) */
	MNU_UNLOAD_USER      = 4,  /* F9 pressed in the module menu: the stock menu takes over; F9 again reloads */
};

/* return values of ui() and file_selected() */
enum {
	MNU_ACT_STAY        = 0,  /* keep showing the module menu; also the right answer after select_file() */
	MNU_ACT_REDRAW      = 1,  /* call ui() again next pass with redraw = 1 */
	MNU_ACT_CLOSE       = 2,  /* close the OSD */
	MNU_ACT_STOCK_MENU  = 3,  /* hand over to the stock config-string menu of the core */
	MNU_ACT_SYSTEM_MENU = 4,  /* hand over to the stock "System" menu */
	MNU_ACT_UNLOAD      = 5,  /* unload the module now; the host restores its own menu */
	MNU_ACT_MISC_MENU   = 6,  /* the stock page left of a core's menu (Main, volume ...); its Right comes back */
};
/* After file_selected() with the OSD hidden (no MNU_FLAG_SELECT_KEEPS_OSD),
 * STAY and REDRAW bring the module menu back, CLOSE leaves the OSD closed,
 * STOCK_MENU, SYSTEM_MENU and MISC_MENU are treated like REDRAW. */

/* file selector options, same bit values as SCANO_* in file_io.h */
#define MNU_SCAN_DIR      0x001   /* include subdirectories */
#define MNU_SCAN_UMOUNT   0x002   /* backspace returns an empty path (unmount) */
#define MNU_SCAN_TXT      0x008
#define MNU_SCAN_NOENTER  0x020
#define MNU_SCAN_NOZIP    0x040
#define MNU_SCAN_CLEAR    0x080
#define MNU_SCAN_SAVES    0x100

#define MNU_OSD_ARROW_LEFT   1
#define MNU_OSD_ARROW_RIGHT  2

/* One decoded input pass, exactly what HandleUI() computes for stock menus. */
typedef struct mnu_event {
	uint32_t size;        /* sizeof(mnu_event_t), for forward compatibility */
	uint32_t key;         /* raw keycode incl. UPSTROKE bit, 0 if none */
	uint32_t key_mod;     /* get_key_mod() */
	uint16_t hold;        /* repeat count while a key is held */
	uint8_t  redraw;      /* 1 on (re)entry: draw everything */
	uint8_t  up, down, left, right;
	uint8_t  select, back, menu;
	uint8_t  plus, minus, recent;
} mnu_event_t;

typedef struct mnu_core_info {
	uint32_t    size;
	const char *rbf_path;       /* resolved absolute path of the loaded .rbf */
	const char *mnu_path;       /* absolute path of this module */
	const char *xml_path;       /* MRA/MGL that launched the core, or NULL */
	const char *core_name;      /* user_io_get_core_name() */
	const char *orig_name;      /* user_io_get_core_name(1) */
	const char *mister_version; /* VDATE of the host build */
} mnu_core_info_t;

/* Host services. The pointer stays valid for the life of the module. */
typedef struct mnu_host_api {
	uint32_t size;
	uint32_t abi_version;

	/* --- OSD drawing (UI coroutine only) --- */
	void (*osd_set_title)(const char *title, int arrow);
	void (*osd_set_size)(int rows);               /* 8 or 16 */
	int  (*osd_get_size)(void);
	void (*osd_clear)(void);
	void (*osd_write)(int row, const char *text, int invert, int stipple);
	/* Scrolling list helper, same contract as the stock MenuWrite loop:
	 *   do { menu_draw_begin(); for (...) menu_write(i, text, i == cursor, 0, 0); } while (menu_draw_end());
	 */
	void (*menu_draw_begin)(void);
	void (*menu_write)(int row, const char *text, int invert, int stipple, int arrow);
	int  (*menu_draw_end)(void);                  /* 1 = list scrolled, draw again */
	void (*menu_reset_scroll)(void);

	/* --- messages --- */
	void (*info)(const char *text, int timeout_ms);
	void (*info_message)(const char *text, int timeout_ms, const char *title);
	void (*progress)(const char *title, const char *text, int current, int max);

	/* --- file selector: the result arrives via file_selected()/file_cancelled() --- */
	void (*select_file)(const char *start_path, const char *extensions, uint32_t options);

	/* --- core I/O --- */
	const char *(*confstr)(int index);            /* user_io_get_confstr(): 0 = core name, 1 = extensions, 2.. = options */
	uint32_t (*status_get)(const char *opt, int ex);
	void     (*status_set)(const char *opt, uint32_t value, int ex);
	int      (*status_save)(const char *filename);
	int      (*file_tx)(const char *path, unsigned char index, int opensave, int mute, int composite, uint32_t load_addr);
	int      (*file_mount)(const char *path, unsigned char index);
	unsigned char (*ext_idx)(const char *path, const char *extensions);
	void     (*kbd_reset)(int assert);
	void     (*send_buttons)(int force);

	/* --- raw SPI escape hatch (advanced; same semantics as spi.h) --- */
	uint16_t (*spi_uio_cmd)(uint16_t cmd);
	uint8_t  (*spi_uio_cmd8)(uint8_t cmd, uint8_t parm);
	uint16_t (*spi_uio_cmd16)(uint8_t cmd, uint16_t parm);
	void     (*spi_uio_cmd32)(uint8_t cmd, uint32_t parm, int wide);

	/* --- paths and files --- */
	const char *(*home_dir)(void);                 /* HomeDir(): the core's games directory */
	const char *(*root_dir)(void);                 /* getRootDir() */
	const char *(*full_path)(const char *name);    /* getFullPath() */
	int  (*file_exists)(const char *path);
	int  (*file_load)(const char *path, void *buf, int size);   /* buf NULL = size query */
	int  (*file_save)(const char *path, const void *buf, int size);
	int  (*config_load)(const char *name, void *buf, int size); /* /media/fat/config/<name> */
	int  (*config_save)(const char *name, const void *buf, int size);

	/* --- time --- */
	uint32_t (*get_timer)(uint32_t ms);
	int      (*check_timer)(uint32_t t);

	/* --- control --- */
	void (*request_unload)(void);                  /* safe from any callback; takes effect on the next UI pass */
	int  (*osd_is_visible)(void);

	/* --- core families --- */
	/* The family's service table (mnu_minimig.h for MNU_FAMILY_MINIMIG), or
	 * NULL when the core is not run as that family. */
	const void *(*family_api)(uint32_t family);

	/* --- aspect ratios, as the stock menus offer them (0 original, 1 full
	 * screen, 2-3 MiSTer.ini's custom ones when set) --- */
	int  (*ar_name)(int ar, char *buf, int len);   /* the name; returns ar, 0 if it is not set up */
	int  (*ar_next)(int ar, int minus);            /* the next one set up */
} mnu_host_api_t;

/* What the module hands back from mnu_entry(). Must be static storage. */
typedef struct mnu_module {
	uint32_t    size;
	uint32_t    abi_version;        /* MNU_ABI_VERSION the module was built against */
	uint32_t    flags;              /* MNU_FLAG_* */
	const char *name;               /* shown in logs */
	const char *version;

	/* Required. Return 0 to accept; anything else and the host dlcloses and falls back. */
	int  (*init)(const mnu_host_api_t *host, const mnu_core_info_t *core);
	/* Required. Join threads, flush state. Host dlcloses right after. */
	void (*unload)(int reason);
	/* Required. Called every HandleUI pass while the module menu owns the OSD. Returns MNU_ACT_*. */
	int  (*ui)(const mnu_event_t *ev);

	/* Optional (may be NULL). */
	int  (*file_selected)(const char *path);   /* returns MNU_ACT_*; empty path = cleared with backspace */
	void (*file_cancelled)(void);
	void (*osd_closed)(void);                  /* the OSD went away (Menu key, Info timeout, unload, ...) */
	void (*poll)(void);                        /* poll coroutine, only with MNU_FLAG_WANT_POLL */

	/* Read before init(), from the descriptor alone (see "Core families"). */
	uint32_t    family;                         /* MNU_FAMILY_* */
	/* An IDE request 1-3 on channel port (0, 1); NULL ignores them. */
	void (*ide_request)(int port, int req);
} mnu_module_t;

/* The single exported symbol:
 *     MNU_EXPORT const mnu_module_t *mnu_entry(void);
 */
typedef const mnu_module_t *(*mnu_entry_fn)(void);

#ifdef __cplusplus
}
#endif
#endif /* MNU_API_H */
