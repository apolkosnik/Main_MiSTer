// A shared object without mnu_entry(): the host must refuse it.
#include "mnu_api.h"
extern "C" MNU_EXPORT int not_a_menu_module(void) { return 42; }
