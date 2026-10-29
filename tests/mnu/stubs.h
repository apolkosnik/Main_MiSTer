// Recording stand-ins for the MiSTer functions mnu.cpp calls. The real host
// side (mnu.cpp, mnu_path.cpp) and the real modules run on top of these.
#ifndef MNU_TEST_STUBS_H
#define MNU_TEST_STUBS_H

#include <stdint.h>

struct stub_state
{
	// knobs
	int core_type;              // user_io_core_type()
	int menu_core;              // is_menu()
	int owns_osd;               // menu_mod_owns_osd()
	const char *root;           // getRootDir()
	const char *confstr[16];    // user_io_get_confstr(i), NULL terminated
	unsigned char ext_idx_ret;  // user_io_ext_idx()
	const char *core_name;      // user_io_get_core_name()
	const char *home;           // user_io_get_core_path(), HomeDir()
	int minimig;                // is_minimig()
	const void *minimig_api;    // mnu_minimig_api()

	// records
	int restore_calls;
	int select_calls;
	char select_path[1024];
	char select_ext[64];
	uint32_t select_opts;
	int file_tx_calls;
	char file_tx_path[1024];
	unsigned char file_tx_index;
	int confstr_calls;
	int status_set_calls;
	char status_set_opt[8][32];
	uint32_t status_set_val[8];
	int title_calls;
	char title[64];
	int write_calls;
	char rows[32][64];
	int row_invert[32];
	int row_stipple[32];
	int draw_begin_calls;
	int draw_end_calls;
	int reset_scroll_calls;
	int ar_next_calls;
	int info_calls;
	char info_text[256];
	int osd_size;
};

extern stub_state stub;
void stub_reset(void);

#endif
