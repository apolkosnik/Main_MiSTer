// Pure path logic for menu modules (.mnu). No MiSTer dependencies so it can
// be unit tested on the build host.
#ifndef MNU_PATH_H
#define MNU_PATH_H

#include <stddef.h>

#define MNU_PATH_MAX 1024

// Resolve the rbf argument main() received the same way fpga_load_rbf() does:
// absolute paths stay, "menu.rbf" lives on storage 0, anything else is
// relative to the root directory. Empty input gives an empty output.
void mnu_resolve_rbf(const char *arg, const char *storage0, const char *root, char *out, size_t outlen);

// Fill cand[] with the module paths to try for an absolute rbf path, most
// specific first: the exact name with .mnu, then the name with the release
// date code stripped (same "_20xxxxxx" rule as the core browser). Returns the
// number of candidates written, 0..max.
int mnu_candidates(const char *rbf, char cand[][MNU_PATH_MAX], int max);

#endif
