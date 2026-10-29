// Drives the real host side (mnu.cpp, mnu_path.cpp) and real modules through
// the stub layer in stubs.cpp. argv[1] = fixture directory prepared by run.sh,
// argv[2] = directory holding the natively built test modules.
//
// Module and host printf() output is captured in <fixture>/stdout.log so the
// test can verify which callbacks ran; test results go to stderr.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

#include "mnu.h"
#include "cfg.h"
#include "user_io.h"
#include "stubs.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond) do { checks++; if (!(cond)) { failures++; fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define CHECK_INT(got, want) do { long g_ = (long)(got), w_ = (long)(want); checks++; if (g_ != w_) { failures++; fprintf(stderr, "FAIL %s:%d: %s = %ld, want %ld\n", __FILE__, __LINE__, #got, g_, w_); } } while (0)
#define CHECK_STR(got, want) do { const char *g_ = (got), *w_ = (want); checks++; if (strcmp(g_, w_)) { failures++; fprintf(stderr, "FAIL %s:%d: %s = \"%s\", want \"%s\"\n", __FILE__, __LINE__, #got, g_, w_); } } while (0)

static char logpath[1100];

static int log_count(const char *needle)
{
	fflush(stdout);
	FILE *f = fopen(logpath, "r");
	if (!f) return -1;
	static char buf[1 << 16];
	size_t n = fread(buf, 1, sizeof(buf) - 1, f);
	buf[n] = 0;
	fclose(f);

	int count = 0;
	for (const char *p = buf; (p = strstr(p, needle)) != NULL; p += strlen(needle)) count++;
	return count;
}

static int log_has(const char *needle) { return log_count(needle) > 0; }

static void dump_log(void)
{
	fflush(stdout);
	FILE *f = fopen(logpath, "r");
	if (!f) return;
	char line[1024];
	fprintf(stderr, "---- captured stdout ----\n");
	while (fgets(line, sizeof(line), f)) fputs(line, stderr);
	fclose(f);
}

static void copy_file(const char *from, const char *to)
{
	FILE *in = fopen(from, "rb");
	FILE *out = fopen(to, "wb");
	if (!in || !out)
	{
		fprintf(stderr, "cannot copy %s to %s\n", from, to);
		exit(2);
	}
	char buf[4096];
	size_t n;
	while ((n = fread(buf, 1, sizeof(buf), in)) > 0) fwrite(buf, 1, n, out);
	fclose(in);
	fclose(out);
	chmod(to, 0755);
}

static const char *core_confstr[] = { "NES", "BINROM", "F,BINROM,Load ROM;", "O1,Swap,Off,On;", NULL };

static void fresh(const char *root)
{
	stub_reset();
	stub.root = root;
	for (int i = 0; core_confstr[i]; i++) stub.confstr[i] = core_confstr[i];
	memset(&cfg, 0, sizeof(cfg));   // menu_module stays 0: the mechanism is off by default
}

// Send one decoded input pass to the module, like HandleUI() does.
static int send(const char *what)
{
	mnu_event_t ev;
	memset(&ev, 0, sizeof(ev));
	ev.size = sizeof(ev);
	if (!strcmp(what, "redraw")) ev.redraw = 1;
	else if (!strcmp(what, "up")) ev.up = 1;
	else if (!strcmp(what, "down")) ev.down = 1;
	else if (!strcmp(what, "left")) ev.left = 1;
	else if (!strcmp(what, "right")) ev.right = 1;
	else if (!strcmp(what, "select")) ev.select = 1;
	else if (!strcmp(what, "back")) ev.back = 1;
	else if (!strcmp(what, "menu")) ev.menu = 1;
	return mnu_ui(&ev);
}

int main(int argc, char **argv)
{
	if (argc < 3)
	{
		fprintf(stderr, "usage: host_sim_test <fixture dir> <module build dir>\n");
		return 2;
	}
	const char *fixture = argv[1];
	const char *build = argv[2];

	char nes_rbf[1100], nes_mnu[1100], nes_exact_mnu[1100], nothing_rbf[1100], game_bin[1100];
	char ark_rbf[1100], ark_mnu[1100], ark_mra[1100], menu_rbf[1100];
	char bad_noentry[1100], bad_abi[1100], bad_init[1100], short_desc[1100], poller[1100];
	snprintf(nes_rbf, sizeof(nes_rbf), "%s/_Console/NES_20261010.rbf", fixture);
	snprintf(nes_mnu, sizeof(nes_mnu), "%s/_Console/NES.mnu", fixture);
	snprintf(nes_exact_mnu, sizeof(nes_exact_mnu), "%s/_Console/NES_20261010.mnu", fixture);
	snprintf(nothing_rbf, sizeof(nothing_rbf), "%s/_Other/Nothing_20260101.rbf", fixture);
	snprintf(game_bin, sizeof(game_bin), "%s/games/NES/game.bin", fixture);
	snprintf(ark_rbf, sizeof(ark_rbf), "%s/_Arcade/cores/Arkanoid.rbf", fixture);
	snprintf(ark_mnu, sizeof(ark_mnu), "%s/_Arcade/cores/Arkanoid.mnu", fixture);
	snprintf(ark_mra, sizeof(ark_mra), "%s/_Arcade/Arkanoid.mra", fixture);
	snprintf(menu_rbf, sizeof(menu_rbf), "%s/menu.rbf", fixture);
	snprintf(bad_noentry, sizeof(bad_noentry), "%s/bad_noentry.so", build);
	snprintf(bad_abi, sizeof(bad_abi), "%s/bad_abi.so", build);
	snprintf(bad_init, sizeof(bad_init), "%s/bad_init.so", build);
	snprintf(short_desc, sizeof(short_desc), "%s/short_desc.so", build);
	snprintf(poller, sizeof(poller), "%s/poller.so", build);

	snprintf(logpath, sizeof(logpath), "%s/stdout.log", fixture);
	int fd = open(logpath, O_CREAT | O_TRUNC | O_WRONLY, 0644);
	if (fd < 0 || dup2(fd, 1) < 0)
	{
		fprintf(stderr, "cannot redirect stdout to %s\n", logpath);
		return 2;
	}
	close(fd);

	char expect[4096];
	int unload1 = 0;   // expected number of "example: unload, reason 1" lines so far

	fresh(fixture);

	/////////////////////////////////////////////////////////////////////
	// discovery gates: off by default, and off means off for every loader
	CHECK_INT(cfg.menu_module, 0);
	CHECK_INT(mnu_try_load(nes_rbf, NULL), 0);
	CHECK(!mnu_active());
	CHECK_INT(mnu_load(nes_mnu), 0);                  // explicit loads are gated too
	CHECK(!mnu_active());
	snprintf(expect, sizeof(expect), "mnu_load %s", nes_mnu);
	mnu_cmd(expect);
	mnu_service();
	CHECK(!mnu_active());
	CHECK(log_has("MNU: menu modules are disabled"));
	cfg.menu_module = 1;

	stub.menu_core = 1;
	CHECK_INT(mnu_try_load(nes_rbf, NULL), 0);
	CHECK(!mnu_active());
	stub.menu_core = 0;

	stub.core_type = 0;
	CHECK_INT(mnu_try_load(nes_rbf, NULL), 0);
	CHECK(!mnu_active());
	stub.core_type = CORE_TYPE_8BIT;

	CHECK_INT(mnu_try_load(nothing_rbf, NULL), 0);   // rbf without a module next to it
	CHECK_INT(mnu_available(nes_rbf), 1);              // browser marker: date-stripped match
	CHECK_INT(mnu_available(ark_rbf), 1);              // exact match
	CHECK_INT(mnu_available(nothing_rbf), 0);
	CHECK_INT(mnu_available(menu_rbf), 0);             // menu core is never marked, even with menu.mnu present
	CHECK_INT(mnu_available(""), 0);
	CHECK_INT(mnu_available(NULL), 0);
	cfg.menu_module = 0;
	CHECK_INT(mnu_available(nes_rbf), 0);              // no marker while the mechanism is off
	cfg.menu_module = 1;
	CHECK_INT(mnu_try_load("", NULL), 0);
	CHECK_INT(mnu_try_load(NULL, NULL), 0);
	CHECK(!mnu_active());

	/////////////////////////////////////////////////////////////////////
	// date code fallback: NES_20261010.rbf picks up NES.mnu
	CHECK_INT(mnu_try_load(nes_rbf, NULL), 1);
	CHECK(mnu_active());
	CHECK_STR(mnu_file(), nes_mnu);
	CHECK(stub.confstr_calls > 0);
	snprintf(expect, sizeof(expect), "example: init for NES (%s), module %s, xml -", nes_rbf, nes_mnu);
	CHECK_INT(log_count(expect), 1);
	CHECK_INT(mnu_flags(), MNU_FLAG_KEY_REPEAT);

	/////////////////////////////////////////////////////////////////////
	// drawing through the vtable
	mnu_osd_opened();
	CHECK_INT(send("redraw"), MNU_ACT_STAY);
	CHECK_STR(stub.title, "NES*");                    // the host stars a module menu's title
	CHECK_INT(stub.draw_begin_calls, 1);
	CHECK_INT(stub.draw_end_calls, 1);
	CHECK_INT(stub.write_calls, 16);
	CHECK_STR(stub.rows[0], " Example module v1.0");
	CHECK_STR(stub.rows[1], " MiSTer test1");
	CHECK_STR(stub.rows[3], " Load file (BINROM)");
	CHECK_INT(stub.row_invert[3], 1);
	CHECK_INT(stub.row_invert[4], 0);
	CHECK_STR(stub.rows[4], " Reset core");
	CHECK_STR(stub.rows[5], " Stock core menu");
	CHECK_STR(stub.rows[6], " System settings");
	CHECK_STR(stub.rows[7], " Unload this module");
	CHECK_STR(stub.rows[15], "            exit");

	/////////////////////////////////////////////////////////////////////
	// navigation
	CHECK_INT(send("down"), MNU_ACT_REDRAW);          // cursor: Reset
	CHECK_INT(send("redraw"), MNU_ACT_STAY);
	CHECK_INT(stub.row_invert[3], 0);
	CHECK_INT(stub.row_invert[4], 1);
	CHECK_INT(send("back"), MNU_ACT_CLOSE);
	CHECK_INT(send("right"), MNU_ACT_SYSTEM_MENU);
	CHECK_INT(send("left"), MNU_ACT_STAY);

	// reset row sends the same pulse as a stock "R" option
	CHECK_INT(send("select"), MNU_ACT_CLOSE);
	CHECK_INT(stub.status_set_calls, 2);
	CHECK_STR(stub.status_set_opt[0], "[0]");
	CHECK_INT(stub.status_set_val[0], 1);
	CHECK_STR(stub.status_set_opt[1], "[0]");
	CHECK_INT(stub.status_set_val[1], 0);

	/////////////////////////////////////////////////////////////////////
	// file selector: cancelled
	CHECK_INT(send("up"), MNU_ACT_REDRAW);            // cursor: Load
	CHECK_INT(send("select"), MNU_ACT_STAY);
	CHECK_INT(stub.select_calls, 1);
	CHECK_STR(stub.select_ext, "BINROM");
	CHECK_INT(stub.select_opts, MNU_SCAN_DIR);
	CHECK_STR(stub.select_path, "");
	mnu_osd_opened();                                 // back in the module menu without a result
	CHECK_INT(log_count("example: file selection cancelled"), 1);
	mnu_osd_opened();                                 // an ordinary redraw entry is not a cancel
	CHECK_INT(log_count("example: file selection cancelled"), 1);

	// file selector: a file was picked
	CHECK_INT(send("select"), MNU_ACT_STAY);
	CHECK_INT(stub.select_calls, 2);
	stub.ext_idx_ret = 2;
	CHECK_INT(mnu_file_selected(game_bin), MNU_ACT_CLOSE);
	CHECK_INT(stub.file_tx_calls, 1);
	CHECK_STR(stub.file_tx_path, game_bin);
	CHECK_INT(stub.file_tx_index, 0x80);
	mnu_osd_opened();
	CHECK_INT(log_count("example: file selection cancelled"), 1);   // a result arrived, so no cancel

	// the module remembers the last file as the selector start path
	CHECK_INT(send("select"), MNU_ACT_STAY);
	CHECK_STR(stub.select_path, game_bin);
	CHECK_INT(mnu_file_selected(""), MNU_ACT_REDRAW);  // cleared with backspace
	CHECK_INT(stub.file_tx_calls, 1);

	// osd closed notification arrives once per open
	mnu_osd_opened();
	mnu_osd_closed();
	mnu_osd_closed();
	CHECK_INT(log_count("example: osd closed"), 1);

	/////////////////////////////////////////////////////////////////////
	// remaining rows
	CHECK_INT(send("down"), MNU_ACT_REDRAW);          // Reset
	CHECK_INT(send("down"), MNU_ACT_REDRAW);          // Stock
	CHECK_INT(send("select"), MNU_ACT_STOCK_MENU);
	CHECK_INT(send("down"), MNU_ACT_REDRAW);          // System
	CHECK_INT(send("select"), MNU_ACT_SYSTEM_MENU);
	CHECK_INT(send("down"), MNU_ACT_REDRAW);          // Unload
	CHECK_INT(send("select"), MNU_ACT_UNLOAD);
	CHECK(mnu_active());                              // menu.cpp applies the action, not mnu_ui()
	CHECK_INT(send("down"), MNU_ACT_REDRAW);          // Exit
	CHECK_INT(send("select"), MNU_ACT_CLOSE);
	CHECK_INT(send("redraw"), MNU_ACT_STAY);
	CHECK_INT(stub.row_invert[15], 1);
	CHECK_INT(send("down"), MNU_ACT_REDRAW);          // wraps to Load
	CHECK_INT(send("redraw"), MNU_ACT_STAY);
	CHECK_INT(stub.row_invert[3], 1);
	CHECK_INT(send("up"), MNU_ACT_REDRAW);            // wraps to Exit
	CHECK_INT(send("redraw"), MNU_ACT_STAY);
	CHECK_INT(stub.row_invert[15], 1);

	/////////////////////////////////////////////////////////////////////
	// unload on request while the module menu is on screen
	stub.owns_osd = 1;
	stub.restore_calls = 0;
	mnu_unload(MNU_UNLOAD_REQUEST);
	CHECK(!mnu_active());
	CHECK_INT(stub.restore_calls, 1);
	CHECK_INT(mnu_take_reopen(), 1);
	CHECK_INT(mnu_take_reopen(), 0);
	CHECK_INT(log_count("example: unload, reason 0"), 1);
	CHECK_STR(mnu_file(), "");
	CHECK_INT(mnu_flags(), 0);
	CHECK_INT(send("redraw"), MNU_ACT_CLOSE);
	CHECK_INT(mnu_file_selected("x"), MNU_ACT_CLOSE);
	mnu_unload(MNU_UNLOAD_REQUEST);                   // idempotent
	mnu_poll();
	mnu_service();
	mnu_osd_opened();
	mnu_osd_closed();
	CHECK_INT(stub.restore_calls, 1);
	CHECK_INT(log_count("example: unload"), 1);
	stub.owns_osd = 0;

	/////////////////////////////////////////////////////////////////////
	// the exact name wins over the date-stripped one
	copy_file(nes_mnu, nes_exact_mnu);
	CHECK_INT(mnu_try_load(nes_rbf, NULL), 1);
	CHECK_STR(mnu_file(), nes_exact_mnu);

	/////////////////////////////////////////////////////////////////////
	// deferred commands from /dev/MiSTer_cmd
	mnu_cmd("mnu_unload");
	CHECK(mnu_active());                              // nothing happens until the UI pass
	mnu_service();
	CHECK(!mnu_active());
	unload1++;
	CHECK_INT(log_count("example: unload, reason 1"), unload1);
	CHECK_INT(stub.restore_calls, 1);                 // module menu was not on screen
	CHECK_INT(mnu_take_reopen(), 0);

	snprintf(expect, sizeof(expect), "mnu_load %s", nes_mnu);
	mnu_cmd(expect);
	mnu_service();
	CHECK(mnu_active());
	CHECK_STR(mnu_file(), nes_mnu);

	int inits = log_count("example: init for NES");
	mnu_cmd("mnu_reload");
	mnu_service();
	CHECK(mnu_active());
	CHECK_STR(mnu_file(), nes_mnu);
	unload1++;
	CHECK_INT(log_count("example: init for NES"), inits + 1);
	CHECK_INT(log_count("example: unload, reason 1"), unload1);

	mnu_cmd("mnu_bogus");
	mnu_service();
	CHECK(mnu_active());
	CHECK(log_has("MNU: unknown command: mnu_bogus"));

	// a relative path resolves against the root directory
	mnu_cmd("mnu_load   _Console/NES_20261010.mnu");
	mnu_service();
	CHECK(mnu_active());
	CHECK_STR(mnu_file(), nes_exact_mnu);
	unload1++;
	CHECK_INT(log_count("example: unload, reason 1"), unload1);

	// reload after an explicit unload brings the last module back (dev loop)
	mnu_cmd("mnu_unload");
	mnu_service();
	CHECK(!mnu_active());
	unload1++;
	mnu_cmd("mnu_reload");
	mnu_service();
	CHECK(mnu_active());
	CHECK_STR(mnu_file(), nes_exact_mnu);

	/////////////////////////////////////////////////////////////////////
	// core exit: unload() runs, nothing is restored or reopened
	stub.owns_osd = 1;
	int restores = stub.restore_calls;
	mnu_unload(MNU_UNLOAD_CORE_EXIT);
	CHECK(!mnu_active());
	CHECK_INT(stub.restore_calls, restores);
	CHECK_INT(mnu_take_reopen(), 0);
	CHECK_INT(log_count("example: unload, reason 2"), 1);
	stub.owns_osd = 0;

	/////////////////////////////////////////////////////////////////////
	// MRA launched core: the module sees the xml path
	CHECK_INT(mnu_try_load(ark_rbf, ark_mra), 1);
	CHECK_STR(mnu_file(), ark_mnu);
	snprintf(expect, sizeof(expect), "example: init for NES (%s), module %s, xml %s", ark_rbf, ark_mnu, ark_mra);
	CHECK_INT(log_count(expect), 1);
	mnu_unload(MNU_UNLOAD_REQUEST);
	CHECK(!mnu_active());

	/////////////////////////////////////////////////////////////////////
	// modules the host must refuse
	CHECK_INT(mnu_load(bad_noentry), 0);
	CHECK(!mnu_active());
	CHECK(log_has("has no mnu_entry()"));

	CHECK_INT(mnu_load(bad_abi), 0);
	CHECK(!mnu_active());
	CHECK(!log_has("bad_abi: init"));
	snprintf(expect, sizeof(expect), "ABI %u, host has %u", MNU_ABI_VERSION + 1, MNU_ABI_VERSION);
	CHECK(log_has(expect));

	CHECK_INT(mnu_load(bad_init), 0);
	CHECK(!mnu_active());
	CHECK(log_has("bad_init: init"));
	CHECK(!log_has("bad_init: unload"));
	CHECK(log_has("init() returned 7"));

	CHECK_INT(mnu_load("/nonexistent/x.mnu"), 0);
	CHECK(log_has("dlopen(/nonexistent/x.mnu) failed"));
	CHECK_INT(mnu_load(""), 0);
	CHECK_INT(mnu_load(NULL), 0);
	CHECK_STR(mnu_file(), "");

	/////////////////////////////////////////////////////////////////////
	// short descriptor: optional callbacks beyond size are never touched
	CHECK_INT(mnu_load(short_desc), 1);
	CHECK(mnu_active());
	CHECK_INT(mnu_file_selected("x"), MNU_ACT_REDRAW);
	mnu_osd_opened();
	mnu_osd_closed();
	mnu_poll();
	mnu_ide_request(0, 1);
	CHECK(!log_has("is for family"));                // the family beyond size reads as none
	CHECK_INT(send("redraw"), MNU_ACT_STAY);
	mnu_unload(MNU_UNLOAD_REQUEST);
	CHECK(log_has("short: unload 0"));

	/////////////////////////////////////////////////////////////////////
	// poll hook and request_unload()
	CHECK_INT(mnu_load(poller), 1);
	mnu_poll();
	mnu_poll();
	mnu_poll();
	CHECK_INT(send("select"), MNU_ACT_STAY);         // module asks to go
	CHECK_INT(send("redraw"), MNU_ACT_UNLOAD);       // host reports it before calling ui() again
	CHECK(mnu_active());
	mnu_service();                                   // or the service pass does it
	CHECK(!mnu_active());
	CHECK(log_has("poller: unload 0, polled 3"));

	/////////////////////////////////////////////////////////////////////
	// loading over a loaded module unloads the old one first
	CHECK_INT(mnu_load(nes_mnu), 1);
	CHECK_INT(mnu_load(poller), 1);
	CHECK_STR(mnu_file(), poller);
	unload1++;
	CHECK_INT(log_count("example: unload, reason 1"), unload1);
	mnu_unload(MNU_UNLOAD_COMMAND);
	CHECK(!mnu_active());
	CHECK(log_has("poller: unload 1, polled 0"));

	/////////////////////////////////////////////////////////////////////
	// End key: unload in place, load again
	CHECK_INT(mnu_load(nes_mnu), 1);
	CHECK_INT(mnu_load_again(), 1);                  // already loaded: nothing happens
	CHECK_STR(mnu_file(), nes_mnu);
	stub.owns_osd = 1;
	restores = stub.restore_calls;
	mnu_unload(MNU_UNLOAD_USER);
	CHECK(!mnu_active());
	CHECK_INT(stub.restore_calls, restores + 1);
	CHECK_INT(mnu_take_reopen(), 1);
	CHECK(log_has("example: unload, reason 4"));
	stub.owns_osd = 0;
	cfg.menu_module = 0;
	CHECK_INT(mnu_load_again(), 0);                  // the switch still rules
	CHECK(!mnu_active());
	cfg.menu_module = 1;
	inits = log_count("example: init for NES");
	CHECK_INT(mnu_load_again(), 1);                  // the last module comes back
	CHECK(mnu_active());
	CHECK_STR(mnu_file(), nes_mnu);
	CHECK_INT(log_count("example: init for NES"), inits + 1);
	mnu_unload(MNU_UNLOAD_COMMAND);

	fprintf(stderr, "host_sim_test: %d checks, %d failures\n", checks, failures);
	if (failures) dump_log();
	return failures ? 1 : 0;
}
