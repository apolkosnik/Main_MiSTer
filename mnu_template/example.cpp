// Reference menu module for Main_MiSTer.
//
// Build:  make modules        -> bin/modules/example.mnu
// Use:    copy it next to a core as <core>.mnu, e.g. /media/fat/_Console/NES.mnu
//
// It replaces the core's main OSD menu with a small list that shows how to
// draw, read keys, use the file selector, poke status bits, hand over to the
// stock menus and unload itself. Everything goes through the host vtable
// from mnu_api.h; the module links with --no-undefined to prove it.

#include <stdio.h>
#include <string.h>
#include "mnu_api.h"

#define EXAMPLE_VERSION "1.0"

static const mnu_host_api_t *host = NULL;
static mnu_core_info_t core;            // string pointers stay valid while loaded
static char ext[16] = "";               // extensions of the core's first F option
static char last_file[1024] = "";
static int  cursor = 0;

enum
{
	ROW_LOAD = 0,
	ROW_RESET,
	ROW_STOCK,
	ROW_SYSTEM,
	ROW_UNLOAD,
	ROW_EXIT,
	ROW_COUNT
};

// First "F..." entry of the config string gives us the file extensions,
// e.g. "F,BINROM,Load ROM" or "FS1,SAV,Load Save".
static void find_extensions(void)
{
	ext[0] = 0;
	for (int i = 2; i < 128; i++)
	{
		const char *p = host->confstr(i);
		if (!p) break;

		// skip hide/disable and page prefixes the same way the stock menu does
		while ((p[0] == 'H' || p[0] == 'D' || p[0] == 'h' || p[0] == 'd') && strlen(p) > 2) p += 2;
		if (p[0] == 'P' && p[1] >= '0' && p[1] <= '9' && p[2] != ',') p += 2;
		if (p[0] != 'F') continue;

		const char *c = strchr(p, ',');
		if (!c) continue;
		c++;
		const char *e = strchr(c, ',');
		size_t n = e ? (size_t)(e - c) : strlen(c);
		if (n > sizeof(ext) - 1) n = sizeof(ext) - 1;
		memcpy(ext, c, n);
		ext[n] = 0;
		return;
	}
}

static void draw(void)
{
	char s[64];
	host->osd_set_title(core.core_name, 0);

	do
	{
		host->menu_draw_begin();
		int m = 0;

		snprintf(s, sizeof(s), " Example module v%s", EXAMPLE_VERSION);
		host->menu_write(m++, s, 0, 0, 0);
		snprintf(s, sizeof(s), " MiSTer %s", core.mister_version);
		host->menu_write(m++, s, 0, 0, 0);
		host->menu_write(m++, "", 0, 0, 0);

		if (ext[0]) snprintf(s, sizeof(s), " Load file (%s)", ext);
		else snprintf(s, sizeof(s), " Load file");
		host->menu_write(m++, s, cursor == ROW_LOAD, ext[0] ? 0 : 1, 0);
		host->menu_write(m++, " Reset core", cursor == ROW_RESET, 0, 0);
		host->menu_write(m++, " Stock core menu", cursor == ROW_STOCK, 0, 0);
		host->menu_write(m++, " System settings", cursor == ROW_SYSTEM, 0, 0);
		host->menu_write(m++, " Unload this module", cursor == ROW_UNLOAD, 0, 0);

		for (; m < host->osd_get_size() - 1; m++) host->menu_write(m, "", 0, 0, 0);
		host->menu_write(m, "            exit", cursor == ROW_EXIT, 0, MNU_OSD_ARROW_RIGHT);
	} while (host->menu_draw_end());
}

static int init(const mnu_host_api_t *h, const mnu_core_info_t *c)
{
	// A host with a shorter vtable than we were built against cannot be used.
	if (!h || h->abi_version != MNU_ABI_VERSION || h->size < sizeof(mnu_host_api_t)) return 1;
	if (!c || c->size < sizeof(mnu_core_info_t)) return 2;

	host = h;
	core = *c;
	cursor = 0;
	last_file[0] = 0;
	find_extensions();

	printf("example: init for %s (%s), module %s, xml %s\n", core.core_name, core.rbf_path, core.mnu_path, core.xml_path ? core.xml_path : "-");
	return 0;
}

static void unload(int reason)
{
	printf("example: unload, reason %d\n", reason);
	host = NULL;
}

static int ui(const mnu_event_t *ev)
{
	if (ev->redraw)
	{
		draw();
		return MNU_ACT_STAY;
	}

	if (ev->up)
	{
		cursor = (cursor + ROW_COUNT - 1) % ROW_COUNT;
		return MNU_ACT_REDRAW;
	}
	if (ev->down)
	{
		cursor = (cursor + 1) % ROW_COUNT;
		return MNU_ACT_REDRAW;
	}
	if (ev->back) return MNU_ACT_CLOSE;
	if (ev->right) return MNU_ACT_SYSTEM_MENU;

	if (ev->select)
	{
		switch (cursor)
		{
		case ROW_LOAD:
			if (!ext[0])
			{
				host->info("Core has no file option", 2000);
				return MNU_ACT_STAY;
			}
			host->select_file(last_file, ext, MNU_SCAN_DIR);
			return MNU_ACT_STAY;

		case ROW_RESET:
			// same pulse the stock "R" option sends
			host->status_set("[0]", 1, 0);
			host->status_set("[0]", 0, 0);
			return MNU_ACT_CLOSE;

		case ROW_STOCK:  return MNU_ACT_STOCK_MENU;
		case ROW_SYSTEM: return MNU_ACT_SYSTEM_MENU;
		case ROW_UNLOAD: return MNU_ACT_UNLOAD;
		case ROW_EXIT:   return MNU_ACT_CLOSE;
		}
	}

	return MNU_ACT_STAY;
}

static int file_selected(const char *path)
{
	if (!path[0]) return MNU_ACT_REDRAW;   // cleared with backspace

	snprintf(last_file, sizeof(last_file), "%s", path);
	unsigned char idx = (unsigned char)(host->ext_idx(path, ext) << 6);   // F option index 0
	printf("example: sending %s (index 0x%02X)\n", path, idx);
	host->file_tx(path, idx, 0, 0, 0, 0);
	return MNU_ACT_CLOSE;
}

static void file_cancelled(void)
{
	printf("example: file selection cancelled\n");
}

static void osd_closed(void)
{
	printf("example: osd closed\n");
}

static const mnu_module_t module =
{
	sizeof(mnu_module_t),
	MNU_ABI_VERSION,
	MNU_FLAG_KEY_REPEAT,
	"example",
	EXAMPLE_VERSION,

	init,
	unload,
	ui,

	file_selected,
	file_cancelled,
	osd_closed,
	NULL,   // poll

	MNU_FAMILY_NONE,
	NULL,   // ide_request
};

extern "C" MNU_EXPORT const mnu_module_t *mnu_entry(void)
{
	return &module;
}
