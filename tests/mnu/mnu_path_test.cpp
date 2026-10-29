// Unit test for mnu_path.cpp: rbf argument resolution and .mnu candidates.
// Build and run with tests/mnu/run.sh on the build host.

#include <stdio.h>
#include <string.h>
#include "mnu_path.h"

static int failures = 0;
static int checks = 0;

#define CHECK_STR(got, want) do { checks++; if (strcmp((got), (want))) { failures++; printf("FAIL %s:%d: got \"%s\", want \"%s\"\n", __FILE__, __LINE__, (got), (want)); } } while (0)
#define CHECK_INT(got, want) do { checks++; if ((got) != (want)) { failures++; printf("FAIL %s:%d: got %d, want %d\n", __FILE__, __LINE__, (int)(got), (int)(want)); } } while (0)

static void test_resolve(void)
{
	char out[MNU_PATH_MAX];

	mnu_resolve_rbf("/media/fat/_Console/NES_20261010.rbf", "/media/fat", "/media/usb0", out, sizeof(out));
	CHECK_STR(out, "/media/fat/_Console/NES_20261010.rbf");

	// menu.rbf always lives on the SD card, even when root is a USB stick
	mnu_resolve_rbf("menu.rbf", "/media/fat", "/media/usb0", out, sizeof(out));
	CHECK_STR(out, "/media/fat/menu.rbf");
	mnu_resolve_rbf("MENU.RBF", "/media/fat", "/media/usb0", out, sizeof(out));
	CHECK_STR(out, "/media/fat/MENU.RBF");

	// any other relative name is relative to the root directory
	mnu_resolve_rbf("_Console/NES.rbf", "/media/fat", "/media/usb0", out, sizeof(out));
	CHECK_STR(out, "/media/usb0/_Console/NES.rbf");

	mnu_resolve_rbf("", "/media/fat", "/media/fat", out, sizeof(out));
	CHECK_STR(out, "");
	mnu_resolve_rbf(NULL, "/media/fat", "/media/fat", out, sizeof(out));
	CHECK_STR(out, "");
}

static void test_candidates(void)
{
	char cand[2][MNU_PATH_MAX];
	int n;

	// dated release: exact name first, then the date code stripped
	n = mnu_candidates("/media/fat/_Console/NES_20261010.rbf", cand, 2);
	CHECK_INT(n, 2);
	CHECK_STR(cand[0], "/media/fat/_Console/NES_20261010.mnu");
	CHECK_STR(cand[1], "/media/fat/_Console/NES.mnu");

	// the browser strips from the first "_20" with at least 6 chars after it
	n = mnu_candidates("/media/fat/_Computer/Minimig_20260101_dev.rbf", cand, 2);
	CHECK_INT(n, 2);
	CHECK_STR(cand[0], "/media/fat/_Computer/Minimig_20260101_dev.mnu");
	CHECK_STR(cand[1], "/media/fat/_Computer/Minimig.mnu");

	// no date code: one candidate
	n = mnu_candidates("/media/fat/_Arcade/cores/Arkanoid.rbf", cand, 2);
	CHECK_INT(n, 1);
	CHECK_STR(cand[0], "/media/fat/_Arcade/cores/Arkanoid.mnu");

	// "_20" followed by fewer than 6 chars is part of the name
	n = mnu_candidates("/media/fat/_Other/Core_2026.rbf", cand, 2);
	CHECK_INT(n, 1);
	CHECK_STR(cand[0], "/media/fat/_Other/Core_2026.mnu");

	// a name that starts with the date code keeps it
	n = mnu_candidates("/media/fat/_Console/_20261010.rbf", cand, 2);
	CHECK_INT(n, 1);
	CHECK_STR(cand[0], "/media/fat/_Console/_20261010.mnu");

	// extension case and absence
	n = mnu_candidates("/x/NES.RBF", cand, 2);
	CHECK_INT(n, 1);
	CHECK_STR(cand[0], "/x/NES.mnu");
	n = mnu_candidates("/x/NES", cand, 2);
	CHECK_INT(n, 1);
	CHECK_STR(cand[0], "/x/NES.mnu");

	// a dot in a directory name is not an extension
	n = mnu_candidates("/media/fat/_Console.old/NES", cand, 2);
	CHECK_INT(n, 1);
	CHECK_STR(cand[0], "/media/fat/_Console.old/NES.mnu");

	// the date code is only looked for in the file name, not the directory
	n = mnu_candidates("/media/fat/_Backup_20260101/NES.rbf", cand, 2);
	CHECK_INT(n, 1);
	CHECK_STR(cand[0], "/media/fat/_Backup_20260101/NES.mnu");

	// caller may limit the list
	n = mnu_candidates("/media/fat/_Console/NES_20261010.rbf", cand, 1);
	CHECK_INT(n, 1);
	CHECK_STR(cand[0], "/media/fat/_Console/NES_20261010.mnu");

	// degenerate inputs
	CHECK_INT(mnu_candidates("/x/.rbf", cand, 2), 0);
	CHECK_INT(mnu_candidates("/x/", cand, 2), 0);
	CHECK_INT(mnu_candidates("", cand, 2), 0);
	CHECK_INT(mnu_candidates(NULL, cand, 2), 0);
	CHECK_INT(mnu_candidates("/x/NES.rbf", cand, 0), 0);
	CHECK_INT(mnu_candidates("/x/NES.rbf", NULL, 2), 0);

	// relative paths (not produced by the host, but must not misbehave)
	n = mnu_candidates("NES_20261010.rbf", cand, 2);
	CHECK_INT(n, 2);
	CHECK_STR(cand[0], "NES_20261010.mnu");
	CHECK_STR(cand[1], "NES.mnu");
}

int main(void)
{
	test_resolve();
	test_candidates();
	printf("mnu_path_test: %d checks, %d failures\n", checks, failures);
	return failures ? 1 : 0;
}
