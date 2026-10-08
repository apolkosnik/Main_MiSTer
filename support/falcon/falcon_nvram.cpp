// Falcon MC146818 persistence: 50 raw bytes, compatible with hatari.nvram.
// The core uploads a coherent snapshot plus a generation token. Only a
// successful atomic file replacement is acknowledged; failures are retried
// by the FPGA and writes made during a save remain dirty.
#include "falcon_nvram.h"
#include "../../file_io.h"
#include "../../user_io.h"
#include "../../spi.h"
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <sys/stat.h>

static const unsigned NVRAM_SIZE = 50;
static const unsigned NVRAM_INDEX = 3;
static const unsigned ACK_INDEX = 4;

static bool nvram_path(char *path, size_t size)
{
	int n = snprintf(path, size, "%s/saves/Falcon/falcon.nvram", getRootDir());
	return n > 0 && (size_t)n < size;
}

static void download(unsigned index, const uint8_t *bytes, unsigned count)
{
	user_io_set_index(index);
	user_io_set_download(1);
	if (count) user_io_file_tx_data(bytes, count);
	user_io_set_download(0);
}

void falcon_nvram_load()
{
	char path[1200];
	uint8_t bytes[NVRAM_SIZE + 1];
	unsigned count = 0;
	if (!nvram_path(path, sizeof(path))) return;
	FILE *f = fopen(path, "rb");
	if (f)
	{
		size_t n = fread(bytes, 1, sizeof(bytes), f);
		if (n == NVRAM_SIZE && !ferror(f)) count = NVRAM_SIZE;
		else printf("Falcon NVRAM: invalid size/read error in %s; using defaults\n", path);
		fclose(f);
	}
	else if (errno != ENOENT)
	{
		printf("Falcon NVRAM: cannot read %s: %s\n", path, strerror(errno));
		return; // Do not replace a file that could not be read with defaults.
	}
	// Empty image initializes defaults from the current OSD monitor setting.
	download(NVRAM_INDEX, bytes, count);
	printf("Falcon NVRAM: %s %s\n", count ? "loaded" : "defaults for", path);
}

static bool save_image(const uint8_t *bytes)
{
	char path[1200], temp[1216], dir[1200];
	if (!nvram_path(path, sizeof(path))) return false;
	snprintf(dir, sizeof(dir), "%s/saves", getRootDir());
	if (mkdir(dir, 0777) && errno != EEXIST) return false;
	snprintf(dir, sizeof(dir), "%s/saves/Falcon", getRootDir());
	if (mkdir(dir, 0777) && errno != EEXIST) return false;
	snprintf(temp, sizeof(temp), "%s.tmp", path);
	FILE *f = fopen(temp, "wb");
	if (!f) return false;
	bool ok = fwrite(bytes, 1, NVRAM_SIZE, f) == NVRAM_SIZE;
	if (fflush(f)) ok = false;
	if (fsync(fileno(f))) ok = false;
	if (fclose(f)) ok = false;
	if (ok && rename(temp, path)) ok = false;
	if (ok)
	{
		int fd = open(dir, O_RDONLY | O_DIRECTORY);
		if (fd < 0) ok = false;
		else
		{
			if (fsync(fd)) ok = false;
			if (close(fd)) ok = false;
		}
	}
	if (!ok) unlink(temp);
	return ok;
}

void falcon_nvram_poll()
{
	uint16_t req = spi_uio_cmd(UIO_CHK_UPLOAD);
	if (!(req & 1) || (req >> 8) != NVRAM_INDEX) return;
	uint8_t packet[NVRAM_SIZE + 5];
	user_io_set_index(NVRAM_INDEX);
	user_io_set_upload(1);
	user_io_file_rx_data(packet, sizeof(packet));
	user_io_set_upload(0);
	if (packet[NVRAM_SIZE + 4] != 0xA5) return; // obsolete request after restore/reset
	if (!save_image(packet))
	{
		printf("Falcon NVRAM: save failed: %s; will retry\n", strerror(errno));
		return;
	}
	download(ACK_INDEX, packet + NVRAM_SIZE, 4);
}
