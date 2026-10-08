// Host regression of the production profile code and Falcon menu cases.
// File/HPS/OSD adapters expose inputs, outputs and injected I/O failures.
#include "support/falcon/falcon_config.h"
#include "user_io.h"
#include "menu.h"
#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

static unsigned checks;
#define CHECK(x) do { ++checks; if (!(x)) throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__) + ": " #x); } while (0)
static std::string root, info;
static uint32_t status;
static int joy, nv_loads;
static bool fail_save, require_reset;
static std::string fail_tx, fail_mount;
static unsigned tx_index;
static bool downloading;
static std::vector<uint8_t> cartridge;
static std::vector<std::string> events;
static std::array<std::string, 16> files, disks;
static std::array<std::string, 7> mounted;
static bool last_composite;

fileTYPE::fileTYPE() : filp(nullptr), mode(0), type(0), zip(nullptr), size(0), offset(0), path{}, name{} {}
fileTYPE::~fileTYPE() { if (filp) fclose(filp); }
int FileOpen(fileTYPE *f, const char *name, char)
{
 f->filp = fopen(name, "rb");
 if (!f->filp) return 0;
 fseek(f->filp, 0, SEEK_END); f->size = ftell(f->filp); rewind(f->filp);
 return 1;
}
int FileClose(fileTYPE *f) { int r = f->filp ? fclose(f->filp) : 0; f->filp = nullptr; return r; }
__off64_t FileGetSize(fileTYPE *f) { return f->size; }
int FileLoadConfig(const char *name, void *buffer, int size)
{
 std::ifstream f(root + "/" + name, std::ios::binary | std::ios::ate);
 if (!f) return 0;
 int len = f.tellg();
 if (!buffer) return len;
 f.seekg(0); f.read(static_cast<char *>(buffer), size ? size : len);
 return f.gcount();
}
int FileSaveConfig(const char *name, void *buffer, int size)
{
 if (fail_save) return 0;
 std::ofstream f(root + "/" + name, std::ios::binary);
 f.write(static_cast<char *>(buffer), size);
 return f ? size : 0;
}
uint32_t user_io_status_get(const char *opt, int)
{
 CHECK(!strcmp(opt, "[6:1]")); return (status >> 1) & 63;
}
void user_io_status_set(const char *opt, uint32_t value, int)
{
 events.push_back(std::string(opt) + "=" + std::to_string(value));
 if (!strcmp(opt, "[6:1]")) status = (status & ~126u) | ((value & 63) << 1);
 else
 {
  unsigned bit; CHECK(sscanf(opt, "[%u]", &bit) == 1);
  status = (status & ~(1u << bit)) | ((value & 1) << bit);
 }
}
void user_io_set_joyswap(int value) { joy = value; }
int user_io_get_joyswap() { return joy; }
void StoreIdx_F(int i, const char *path) { files.at(i) = path; }
void StoreIdx_S(int i, const char *path) { disks.at(i) = path; }
void falcon_nvram_load()
{
 CHECK(status & 1); ++nv_loads;
 events.push_back("nvram:" + std::to_string((status >> 1) & 3));
}
int user_io_file_tx(const char *path, unsigned char index, char, char, char composite, uint32_t)
{
 if (require_reset) CHECK(status & 1);
 CHECK((index & 63) <= 2);
 events.push_back("rom:" + std::to_string(index));
 if (fail_tx == path) return 0;
 last_composite = composite;
 falcon_config_record_rom(index, path, composite);
 return 1;
}
int user_io_file_mount(const char *path, unsigned char index, char, int)
{
 if (require_reset) CHECK(status & 1);
 CHECK(index < 7);
 events.push_back("disk:" + std::to_string(index));
 bool ok = *path && fail_mount != path;
 mounted.at(index) = ok ? path : "";
 falcon_config_record_disk(index, mounted.at(index).c_str());
 return ok;
}
void user_io_set_index(unsigned char index) { tx_index = index; }
void user_io_set_download(unsigned char enable, int)
{
 CHECK(tx_index == 2); CHECK(status & 1);
 downloading = enable;
 if (enable) cartridge.clear();
}
void user_io_file_tx_data(const uint8_t *bytes, uint32_t size)
{
 CHECK(downloading && tx_index == 2); CHECK(status & 1);
 cartridge.insert(cartridge.end(), bytes, bytes + size);
}

// Compile the Falcon menu code verbatim, extracted from menu.cpp by run.sh.
#include "menu_enum.inc"
static unsigned menustate, parentstate, menusub, saved_menustate;
static uint64_t menumask;
static unsigned firstmenu, page, m, helptext_idx;
static bool menu, back, right, left, select_pressed, recent;
static char s[2048], fs_pFileExt[64], selPath[1024], SelectedDir[1024], SelectedLabel[1024];
static char Selected_S[16][1024];
static int fs_Options, fs_MenuSelect, fs_MenuCancel;
static std::array<std::string,16> rows;
static std::array<bool,16> highlighted, disabled;
static std::string title;
static int picker_recent, recent_saved;
static const char *STD_EXIT = " Exit", *STD_BACK = " Back";
static const int OSD_ARROW_RIGHT = 1, OSD_ARROW_LEFT = 2;
static void OsdSetSize(int size) { CHECK(size == 16); }
static void OsdSetTitle(const char *text, int) { title = text; }
static void OsdWrite(unsigned row, const char *text = "", bool selected = false, bool disable = false, int = 0)
{
 rows.at(row) = text; highlighted.at(row) = selected; disabled.at(row) = disable;
}
void SelectFile(const char *, const char *ext, int, unsigned char selected, unsigned char cancel, int recent_idx)
{
 CHECK(!strcmp(ext, "ST ")); CHECK(cancel == MENU_FALCON_MAIN1);
 menustate = MENU_FILE_SELECT1; fs_MenuSelect = selected; picker_recent = recent_idx;
}
static int recent_init(int i) { picker_recent = i; return 1; }
static void recent_update(const char *, const char *, const char *, int i) { recent_saved = i; }
void MenuHide() { menustate = MENU_NONE1; }
void Info(const char *message, int, int, int, int)
{
 CHECK(menustate == MENU_NONE1); info = message;
}
static void handle_menu()
{
 bool select = select_pressed;
 switch (menustate)
 {
#include "menu_cases.inc"
 default: break;
 }
 menu = back = right = left = select_pressed = recent = false;
}
static void click(unsigned sub)
{
 menusub = sub; select_pressed = true; handle_menu();
}
static void open_root()
{
 menustate = MENU_FALCON_MAIN1; menusub = 0; handle_menu();
 CHECK(menustate == MENU_FALCON_MAIN2);
}
static void choose_profile(bool save, unsigned slot)
{
 open_root(); click(save ? 4 : 3); handle_menu();
 CHECK(menustate == (save ? MENU_FALCON_SAVE_CONFIG2 : MENU_FALCON_LOAD_CONFIG2));
 click(slot);
}

static std::string media(unsigned i)
{
 return root + "/media-" + std::to_string(i) + (i < 2 ? ".rom" : ".img");
}
static std::vector<char> read_slot(unsigned slot)
{
 std::ifstream f(root + "/FALCON" + std::to_string(slot) + ".CFG", std::ios::binary);
 return std::vector<char>(std::istreambuf_iterator<char>(f), {});
}
static void write_slot(unsigned slot, const std::vector<char> &data)
{
 std::ofstream f(root + "/FALCON" + std::to_string(slot) + ".CFG", std::ios::binary);
 f.write(data.data(), data.size());
}
static void setup_media()
{
 falcon_config_init();
 for (unsigned i = 0; i < 9; ++i) { std::ofstream f(media(i)); f << "media"; }
 falcon_config_record_rom(0, media(0).c_str(), false);
 falcon_config_record_rom(2 | 128, media(1).c_str(), false);
 for (unsigned i = 0; i < 7; ++i) falcon_config_record_disk(i, media(i + 2).c_str());
 CHECK(files[1] == media(0) && files[2] == media(1));
}

int main(int argc, char **argv)
{
 try
 {
  CHECK(argc == 2); CHECK(MENU_ATARI8BIT_CART2 < 256);
  root = argv[1]; menustate = MENU_NONE1;
  falcon_config_init();
  CHECK(!falcon_config_save(0));
  events.clear(); CHECK(!falcon_config_boot()); CHECK(events.empty() && !nv_loads);
  open_root(); CHECK(title == "Falcon" && menumask == 0x3FF);
  CHECK(rows[3].find("Modify config") != std::string::npos);
  CHECK(rows[4].find("Load config") != std::string::npos);
  CHECK(rows[5].find("Save config") != std::string::npos);
  click(3); handle_menu(); handle_menu();
  CHECK(menusub == 9 && menumask == 0x200 && highlighted[15]);
  CHECK(menustate == MENU_FALCON_LOAD_CONFIG2);
  click(9); CHECK(menustate == MENU_FALCON_MAIN1 && menusub == 3);
  setup_media();
  require_reset = true;
  // Every setting combination, with action/unrelated bits present at save.
  for (unsigned options = 0; options < 64; ++options)
  {
   status = 0x180 | 1 | (options << 1) | (1 << 20); joy = options & 1;
   CHECK(falcon_config_save(options % 9));
   status = 1 << 20; joy = 0; events.clear();
   CHECK(falcon_config_load(options % 9));
   CHECK(status == ((options << 1) | (1 << 20)) && joy == int(options & 1));
   CHECK(events.front() == "[0]=1" && events.back() == "[0]=0");
   CHECK(std::find(events.begin(), events.end(), "[8]=1") == events.end());
   CHECK(!nv_loads);
   for (unsigned i = 0; i < 7; ++i) CHECK(mounted[i] == media(i + 2));
  }
  for (unsigned slot = 0; slot < 9; ++slot)
  {
   choose_profile(true, slot); CHECK(info == "Config saved");
   char label[64]; CHECK(falcon_config_describe(slot, label, sizeof(label)));
   CHECK(label[0] == char('0' + slot));
   choose_profile(false, slot); CHECK(info == "Config loaded");
  }
  // Boot applies the selected monitor before NVRAM, then TOS, then disks.
  status = 2 << 1; joy = 1; CHECK(falcon_config_save(0));
  events.clear(); status = 1; menustate = MENU_NONE1;
  CHECK(falcon_config_boot()); CHECK(status == 5 && nv_loads == 1);
  auto nv = std::find(events.begin(), events.end(), "nvram:2");
  auto rom = std::find(events.begin(), events.end(), "rom:1");
  CHECK(nv != events.end() && nv < rom && events.back() == "[7]=0");
  // Empty media actively removes old mounts and erases a stale cartridge.
  falcon_config_record_rom(2, "", false);
  for (unsigned i = 0; i < 7; ++i) falcon_config_record_disk(i, "");
  CHECK(falcon_config_save(8)); setup_media();
  CHECK(falcon_config_load(8));
  CHECK(!*falcon_config_rom(2) && cartridge.size() == 128 * 1024);
  CHECK(std::all_of(cartridge.begin(), cartridge.end(), [](uint8_t x) { return x == 255; }));
  for (unsigned i = 0; i < 7; ++i) CHECK(mounted[i].empty() && disks[i].empty());
  // Composite boot images retain their loader flag through save/load.
  falcon_config_record_rom(0, media(0).c_str(), true);
  CHECK(falcon_config_save(7)); CHECK(falcon_config_load(7)); CHECK(last_composite);
  auto valid = read_slot(7); CHECK(valid.size() == 9228);
  // Reject malformed files without any reset, ROM or mount side effects.
  std::vector<std::vector<char>> bad;
  auto b = valid; b.pop_back(); bad.push_back(b);
  b = valid; b.push_back(0); bad.push_back(b);
  for (unsigned byte : {0u, 8u, 9u, 10u, 11u}) { b = valid; b[byte] = char(255); bad.push_back(b); }
  b = valid; std::fill(b.begin() + 12, b.begin() + 1036, 'x'); bad.push_back(b);
  b = valid; b[12] = 0; bad.push_back(b);
  for (auto &data : bad)
  {
   write_slot(6, data); events.clear();
   uint32_t before = status; CHECK(!falcon_config_load(6));
   CHECK(events.empty() && status == before && nv_loads == 1);
  }
  auto startup = read_slot(0);
  write_slot(0, bad.front()); events.clear();
  CHECK(!falcon_config_boot()); CHECK(events.empty() && nv_loads == 1);
  write_slot(0, startup);
  CHECK(!falcon_config_save(9)); CHECK(!falcon_config_load(9));
  setup_media(); CHECK(falcon_config_save(5));
  CHECK(!remove(media(8).c_str())); events.clear();
  CHECK(!falcon_config_load(5)); CHECK(events.empty());
  CHECK(std::string(falcon_config_error()).find("media-8") != std::string::npos);
  setup_media();
  auto previous = read_slot(5); fail_save = true;
  choose_profile(true, 5); CHECK(info.find("Cannot save") != std::string::npos);
  CHECK(previous == read_slot(5)); fail_save = false;
  // Report late transport/mount errors and release reset, allowing recovery.
  fail_tx = media(0); CHECK(!falcon_config_load(5)); CHECK(!(status & 1)); fail_tx.clear();
  fail_mount = media(7); CHECK(!falcon_config_load(5)); CHECK(!(status & 1));
  CHECK(mounted[5].empty()); fail_mount.clear();
  require_reset = false;
  open_root(); click(2); CHECK(menustate == MENU_GENERIC_MAIN1 && !page && !menusub && !firstmenu);
  open_root(); right = true; handle_menu(); CHECK(menustate == MENU_COMMON1);
  open_root(); left = true; handle_menu(); CHECK(menustate == MENU_MISC1);
  open_root(); joy = 0; click(5); CHECK(joy == 1);
  open_root(); click(7); CHECK(menustate == MENU_NONE1 && !(status & 128));
  // Both floppy pickers use the same recent-list namespace as the editor.
  for (unsigned i = 0; i < 2; ++i)
  {
   falcon_config_record_disk(i, ""); open_root(); click(i);
   CHECK(menustate == MENU_FILE_SELECT1 && picker_recent == int(500 + i));
   strcpy(selPath, media(i + 2).c_str()); menustate = fs_MenuSelect; handle_menu();
   CHECK(menustate == MENU_FALCON_MAIN1 && recent_saved == int(500 + i));
   handle_menu(); click(i); CHECK(!*falcon_config_disk(i));
  }
  printf("PASS: %u checks (64 option combinations, 9 slots, startup/NVRAM order, media restore/eject, malformed and failed I/O, production menu navigation)\n", checks);
 }
 catch (const std::exception &e) { fprintf(stderr, "FAIL: %s\n", e.what()); return 1; }
}
