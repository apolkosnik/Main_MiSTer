#include "falcon_config.h"
#include "falcon_nvram.h"
#include "../../file_io.h"
#include "../../user_io.h"
#include "../../menu.h"
#include <cstdio>
#include <cstring>

namespace
{
// Byte-only, versioned format, independent of compiler padding/endianness.
// Only status[6:1] is persistent: reset/cold-reset/NVRAM-reset are actions.
struct Config
{
	char magic[8];
	uint8_t version, options, joyswap, composite;
	char path[9][1024]; // TOS, cartridge, floppy A/B, IDE 0/1, SCSI 0/1/2
};
static_assert(sizeof(Config) == 9228, "Falcon config layout");
const char magic[8] = {'F', 'L', 'C', 'N', 'C', 'F', 'G', 0};
Config current = {};
char error[160] = {};

bool fail(const char *message, const char *path = "")
{
	const char *name = strrchr(path, '/');
	snprintf(error, sizeof(error), "%s%s%s", message, path[0] ? ": " : "", name ? name + 1 : path);
	printf("Falcon config: %s\n", error);
	return false;
}

bool filename(unsigned slot, char *name)
{
	if (slot > 8) return fail("Invalid config slot");
	snprintf(name, 16, "FALCON%u.CFG", slot);
	return true;
}

bool read_config(unsigned slot, Config &config)
{
	char name[16];
	if (!filename(slot, name)) return false;
	if (FileLoadConfig(name, nullptr, 0) != sizeof(config) ||
	    FileLoadConfig(name, &config, sizeof(config)) != sizeof(config))
		return fail("Missing or invalid config", name);
	if (memcmp(config.magic, magic, sizeof(magic)) || config.version != 1 ||
	    config.options > 63 || config.joyswap > 1 || config.composite > 1)
		return fail("Invalid config format", name);
	for (auto &path : config.path)
		if (!memchr(path, 0, sizeof(path))) return fail("Invalid config path", name);
	if (!config.path[0][0]) return fail("Config has no TOS image", name);
	return true;
}

bool preflight(const Config &config)
{
	// Reject missing/unreadable files before resetting or changing the machine.
	for (const auto &path : config.path)
	{
		if (!path[0]) continue;
		fileTYPE file;
		if (!FileOpen(&file, path)) return fail("Cannot open config media", path);
		bool ok = FileGetSize(&file) > 0;
		FileClose(&file);
		if (!ok) return fail("Empty config media", path);
	}
	return true;
}

void clear_cartridge()
{
	// The cartridge is RAM in DDR3, without a presence bit. Erase its entire
	// 128 KiB aperture when a profile has no cartridge, including after boot.
	uint8_t bytes[1024];
	memset(bytes, 0xFF, sizeof(bytes));
	user_io_set_index(2);
	user_io_set_download(1);
	for (unsigned i = 0; i < 128; ++i) user_io_file_tx_data(bytes, sizeof(bytes));
	user_io_set_download(0);
	falcon_config_record_rom(2, "", false);
}

bool apply(const Config &config, bool boot)
{
	user_io_status_set("[0]", 1);
	user_io_status_set("[6:1]", config.options);
	user_io_set_joyswap(config.joyswap);
	// Startup defaults must see the saved monitor type. Loading a profile
	// during use must preserve guest NVRAM, including changes not yet saved.
	if (boot) falcon_nvram_load();
	bool ok = user_io_file_tx(config.path[0], 1, 0, 0, config.composite);
	if (!ok) fail("Cannot load TOS", config.path[0]);
	if (ok)
	{
		// Clear first so a smaller cartridge cannot leave bytes from an old one.
		clear_cartridge();
		if (config.path[1][0] && !user_io_file_tx(config.path[1], 2))
			ok = fail("Cannot load cartridge", config.path[1]);
		for (unsigned i = 0; i < 7; ++i)
		{
			user_io_set_index(i);
			int mounted = user_io_file_mount(config.path[i + 2], i);
			if (config.path[i + 2][0] && !mounted)
				ok = fail("Cannot mount config media", config.path[i + 2]);
		}
	}
	// A profile switch is a cold boot. Keep reset asserted until all media
	// changes have finished; the normal startup path releases reset at boot.
	user_io_status_set("[7]", 1);
	user_io_status_set("[7]", 0);
	if (!boot) user_io_status_set("[0]", 0);
	return ok;
}
}

void falcon_config_init()
{
	memset(&current, 0, sizeof(current));
	error[0] = 0;
}

void falcon_config_record_rom(unsigned index, const char *path, bool composite)
{
	index &= 63; // high bits identify the file extension
	if (index > 2) return;
	unsigned entry = index == 2 ? 1 : 0;
	snprintf(current.path[entry], sizeof(current.path[entry]), "%s", path);
	if (!entry) current.composite = composite;
	StoreIdx_F(entry + 1, current.path[entry]);
}

void falcon_config_record_disk(unsigned index, const char *path)
{
	if (index >= 7) return;
	snprintf(current.path[index + 2], sizeof(current.path[index + 2]), "%s", path);
	StoreIdx_S(index, current.path[index + 2]);
}

const char *falcon_config_rom(unsigned index)
{
	return index >= 1 && index <= 2 ? current.path[index - 1] : "";
}

const char *falcon_config_disk(unsigned index)
{
	return index < 7 ? current.path[index + 2] : "";
}

bool falcon_config_save(unsigned slot)
{
	char name[16];
	if (!filename(slot, name)) return false;
	if (!current.path[0][0]) return fail("Load a TOS image before saving");
	Config config = current;
	memcpy(config.magic, magic, sizeof(magic));
	config.version = 1;
	config.options = user_io_status_get("[6:1]");
	config.joyswap = !!user_io_get_joyswap();
	if (FileSaveConfig(name, &config, sizeof(config)) != sizeof(config))
		return fail("Cannot save config", name);
	return true;
}

bool falcon_config_load(unsigned slot)
{
	Config config;
	return read_config(slot, config) && preflight(config) && apply(config, false);
}

bool falcon_config_boot()
{
	Config config;
	// No usable startup config: retain the normal boot.rom/boot.vhd path.
	if (!read_config(0, config) || !preflight(config)) return false;
	if (!apply(config, true)) Info(error, 5000);
	return true;
}

bool falcon_config_describe(unsigned slot, char *text, unsigned size)
{
	Config config;
	bool valid = read_config(slot, config);
	if (!valid) snprintf(text, size, "%u: <empty/invalid>", slot);
	else
	{
		const char *monitors[] = {"VGA", "RGB", "TV", "Mono"};
		const char *name = strrchr(config.path[0], '/');
		snprintf(text, size, "%u: %uM %s %s", slot, config.options & 4 ? 4 : 14,
		         monitors[config.options & 3], name ? name + 1 : config.path[0]);
	}
	return valid;
}

const char *falcon_config_error()
{
	return error;
}
