#ifndef MENU_H
#define MENU_H

#include <inttypes.h>

void SelectFile(const char* path, const char* pFileExt, int Options, unsigned char MenuSelect, unsigned char MenuCancel, int recent_idx = -1);

void HandleUI(void);
void menu_key_set(unsigned int c);
void menu_process_save();
void PrintDirectory(int expand = 0);
void ScrollLongName(void);

void ProgressMessage(const char* title = 0, const char* text = 0, int current = 0, int max = 0);
void InfoMessage(const char *message, int timeout = 2000, const char *title = "Message");
void Info(const char *message, int timeout = 2000, int width = 0, int height = 0, int frame = 0);
void MenuHide();
void SelectINI();

void open_joystick_setup();
int menu_lightgun_cb(int idx, uint16_t type, uint16_t code, int value);

int menu_allow_cfg_switch();
void StoreIdx_F(int idx, const char *path);
void StoreIdx_S(int idx, const char *path);
const char *GetIdx_S(int idx);

int menu_present();

// glue for the menu module host (mnu.cpp)
void menu_mod_draw_begin();
void menu_mod_write(int row, const char *s, unsigned char invert, unsigned char stipple, int arrow);
int  menu_mod_draw_end();
void menu_mod_reset_scroll();
int  menu_mod_ar_name(int ar, char *buf, int len);
int  menu_mod_ar_next(int ar, int minus);
void menu_mod_select_file(const char *path, const char *ext, uint32_t options);
int  menu_mod_owns_osd();
void menu_mod_restore();

#endif
