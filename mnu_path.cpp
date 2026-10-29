#include <stdio.h>
#include <string.h>
#include <strings.h>
#include "mnu_path.h"

void mnu_resolve_rbf(const char *arg, const char *storage0, const char *root, char *out, size_t outlen)
{
	if (!out || !outlen) return;
	out[0] = 0;
	if (!arg || !arg[0]) return;

	if (arg[0] == '/') snprintf(out, outlen, "%s", arg);
	else snprintf(out, outlen, "%s/%s", !strcasecmp(arg, "menu.rbf") ? storage0 : root, arg);
}

int mnu_candidates(const char *rbf, char cand[][MNU_PATH_MAX], int max)
{
	if (!rbf || !rbf[0] || !cand || max < 1) return 0;

	const char *base = strrchr(rbf, '/');
	base = base ? base + 1 : rbf;
	if (!base[0]) return 0;

	// stem = path without the extension of the basename
	const char *dot = strrchr(base, '.');
	size_t stem_len = dot ? (size_t)(dot - rbf) : strlen(rbf);
	if (stem_len == (size_t)(base - rbf)) return 0;             // ".rbf" alone, no name
	if (stem_len + sizeof(".mnu") > MNU_PATH_MAX) return 0;

	int n = 0;
	snprintf(cand[n++], MNU_PATH_MAX, "%.*s.mnu", (int)stem_len, rbf);
	if (n >= max) return n;

	// Date code: the core browser (file_io.cpp) treats the first "_20" that
	// has at least 6 more characters after it as the start of the release
	// date and hides it. Mirror that so NES.mnu matches what the user sees.
	char stem[MNU_PATH_MAX];
	memcpy(stem, rbf, stem_len);
	stem[stem_len] = 0;
	char *name = stem + (base - rbf);
	char *p = strstr(name, "_20");
	if (p && p > name && strlen(p + 3) >= 6)
	{
		snprintf(cand[n++], MNU_PATH_MAX, "%.*s.mnu", (int)(p - stem), stem);
	}
	return n;
}
