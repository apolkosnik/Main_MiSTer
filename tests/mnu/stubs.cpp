#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>

#include "stubs.h"
#include "menu.h"
#include "osd.h"
#include "user_io.h"
#include "file_io.h"
#include "hardware.h"
#include "spi.h"
#include "cfg.h"
#include "mnu.h"

stub_state stub;
cfg_t cfg;
const char *version = "$VER:test1";

void stub_reset(void)
{
	memset(&stub, 0, sizeof(stub));
	stub.core_type = CORE_TYPE_8BIT;
	stub.root = "/media/fat";
	stub.osd_size = 16;
	stub.ext_idx_ret = 0;
	stub.core_name = "NES";
	stub.home = "/media/fat/games/NES";
}

// ---- menu.h ----
void Info(const char *message, int timeout, int, int, int)
{
	stub.info_calls++;
	snprintf(stub.info_text, sizeof(stub.info_text), "%s", message);
	(void)timeout;
}
void InfoMessage(const char *message, int timeout, const char *title)
{
	stub.info_calls++;
	snprintf(stub.info_text, sizeof(stub.info_text), "%s|%s", title, message);
	(void)timeout;
}
void ProgressMessage(const char *, const char *, int, int) {}
void menu_mod_draw_begin() { stub.draw_begin_calls++; }
void menu_mod_write(int row, const char *s, unsigned char invert, unsigned char stipple, int)
{
	stub.write_calls++;
	if (row < 0 || row >= 32) return;
	snprintf(stub.rows[row], sizeof(stub.rows[row]), "%s", s);
	stub.row_invert[row] = invert;
	stub.row_stipple[row] = stipple;
}
int  menu_mod_draw_end() { stub.draw_end_calls++; return 0; }
void menu_mod_reset_scroll() { stub.reset_scroll_calls++; }
// as the stock menus have them with no custom aspect ratios in MiSTer.ini
int  menu_mod_ar_name(int ar, char *buf, int len)
{
	ar = (ar == 1) ? 1 : 0;
	snprintf(buf, len, "%s", ar ? "Full Screen" : "Original");
	return ar;
}
int  menu_mod_ar_next(int ar, int minus) { stub.ar_next_calls++; return (ar == 0) != !minus ? 0 : 1; }
void menu_mod_select_file(const char *path, const char *ext, uint32_t options)
{
	stub.select_calls++;
	snprintf(stub.select_path, sizeof(stub.select_path), "%s", path);
	snprintf(stub.select_ext, sizeof(stub.select_ext), "%s", ext);
	stub.select_opts = options;
}
int  menu_mod_owns_osd() { return stub.owns_osd; }
void menu_mod_restore() { stub.restore_calls++; }

// ---- osd.h ----
void OsdSetTitle(const char *s, int)
{
	stub.title_calls++;
	snprintf(stub.title, sizeof(stub.title), "%s", s);
}
void OsdSetSize(int n) { stub.osd_size = n; }
int  OsdGetSize() { return stub.osd_size; }
void OsdClear() {}
void OsdWrite(unsigned char, const char *, unsigned char, unsigned char, char, int, int) {}

// ---- user_io.h ----
char *user_io_get_confstr(int index)
{
	stub.confstr_calls++;
	for (int i = 0; i <= index && i < 16; i++) if (!stub.confstr[i]) return NULL;
	return index < 16 ? (char*)stub.confstr[index] : NULL;
}
uint32_t user_io_status_get(const char *, int) { return 0; }
void user_io_status_set(const char *opt, uint32_t value, int)
{
	int i = stub.status_set_calls < 8 ? stub.status_set_calls : 7;
	snprintf(stub.status_set_opt[i], sizeof(stub.status_set_opt[i]), "%s", opt);
	stub.status_set_val[i] = value;
	stub.status_set_calls++;
}
int  user_io_status_save(const char *) { return 1; }
int  user_io_file_tx(const char *name, unsigned char index, char, char, char, uint32_t)
{
	stub.file_tx_calls++;
	snprintf(stub.file_tx_path, sizeof(stub.file_tx_path), "%s", name);
	stub.file_tx_index = index;
	return 1;
}
int  user_io_file_mount(const char *, unsigned char, char, int, int) { return 1; }
unsigned char user_io_ext_idx(char *, char *) { return stub.ext_idx_ret; }
void user_io_set_kbd_reset(int) {}
void user_io_send_buttons(char) {}
char *user_io_get_core_name(int) { return (char*)stub.core_name; }
unsigned char user_io_core_type() { return (unsigned char)stub.core_type; }
char is_menu() { return (char)stub.menu_core; }
char user_io_osd_is_visible() { return 0; }
char *user_io_get_core_path(const char *, int) { return (char*)stub.home; }
char is_minimig() { return (char)stub.minimig; }

// ---- mnu.h (mnu_minimig.cpp) ----
const void *mnu_minimig_api() { return stub.minimig_api; }

// ---- file_io.h ----
const char *getStorageDir(int dev) { return dev ? "/media/usb0" : "/media/fat"; }
const char *getRootDir() { return stub.root; }
const char *getFullPath(const char *name)
{
	static char full[1024];
	if (name[0] == '/') snprintf(full, sizeof(full), "%s", name);
	else snprintf(full, sizeof(full), "%s/%s", stub.root, name);
	return full;
}
int FileExists(const char *name, int)
{
	struct stat st;
	return !stat(getFullPath(name), &st) && S_ISREG(st.st_mode);
}
int FileLoad(const char *, void *, int) { return 0; }
int FileSave(const char *, void *, int) { return 0; }
int FileLoadConfig(const char *, void *, int) { return 0; }
int FileSaveConfig(const char *, void *, int) { return 0; }

// ---- hardware.h ----
unsigned long GetTimer(unsigned long offset) { return 1000 + offset; }
unsigned long CheckTimer(unsigned long) { return 1; }

// ---- spi.h ----
uint16_t spi_uio_cmd(uint16_t) { return 0; }
uint8_t  spi_uio_cmd8(uint8_t, uint8_t) { return 0; }
uint16_t spi_uio_cmd16(uint8_t, uint16_t) { return 0; }
void     spi_uio_cmd32(uint8_t, uint32_t, int) {}
