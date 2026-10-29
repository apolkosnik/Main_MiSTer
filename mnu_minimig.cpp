// The Minimig family's services for menu modules, see mnu_minimig.h.

#include <stdio.h>
#include <string.h>

#include "mnu_minimig.h"
#include "ide.h"
#include "file_io.h"
#include "support/minimig/minimig_config.h"
#include "support/minimig/minimig_fdd.h"
#include "support/minimig/minimig_a2065.h"

static void m_reset(void) { minimig_reset(); }
static int  m_cfg_save(int num) { return minimig_cfg_save(num); }
static int  m_cfg_load(int num) { return minimig_cfg_load(num); }
static void m_config_cpu(void) { minimig_ConfigCPU(minimig_config.cpu); }
static void m_config_chipset(void) { minimig_ConfigChipset(&minimig_config); }
static void m_config_floppy(void)
{
	minimig_ConfigFloppy(minimig_config.floppy.drives, minimig_config.floppy.speed);
	minimig_ConfigFloppyExt(minimig_config.externalfloppy.exDrives[0], minimig_config.externalfloppy.exDrives[1],
		minimig_config.externalfloppy.exDrives[2], minimig_config.externalfloppy.exDrives[3]);
}
static void m_set_kickstart(const char *path) { minimig_set_kickstart(path ? path : ""); }
static void m_set_extrom(const char *path) { minimig_set_extrom(path ? path : ""); }
static const char *m_get_extrom(void) { return minimig_get_extrom(); }
static uint32_t m_get_extcfg(void) { return minimig_get_extcfg(); }
static void m_set_hrtmon_a1200(uint8_t value) { minimig_set_hrtmon_a1200(value); }

static int m_floppy_status(int drive, char *name, int namelen)
{
	if (drive < 0 || drive > 3) return 0;
	if (name && namelen > 0) snprintf(name, namelen, "%s", (df[drive].status & DSK_INSERTED) ? df[drive].name : "");
	return df[drive].status & (DSK_INSERTED | DSK_WRITABLE);
}

static void m_floppy_insert(int drive, const char *path)
{
	if (drive < 0 || drive > 3 || !path) return;
	static char p[1024];
	snprintf(p, sizeof(p), "%s", path);
	InsertFloppy(&df[drive], p);
}

// as the stock menu ejects: the flux reader is closed, not deleted (race)
static void m_floppy_eject(int drive)
{
	if (drive < 0 || drive > 3) return;
	df[drive].status = 0;
	FileClose(&df[drive].file);
	if (df[drive].fluxFile) df[drive].fluxFile->closeFile();
}

static int m_floppy_active_count(void) { return minimig_floppy_active_count(); }
static int m_floppy_requested_count(void) { return minimig_floppy_requested_count(minimig_config.floppy.drives); }

static int m_floppy_step_drives(int direction)
{
	uint8_t n = minimig_floppy_step_drives(minimig_config.floppy.drives, direction < 0 ? -1 : 1);
	if (n != minimig_config.floppy.drives)
	{
		if (direction < 0) minimig_config.externalfloppy.exDrives[minimig_config.floppy.drives] = 0;  // disable before remove
		minimig_config.floppy.drives = n;
		m_config_floppy();
	}
	return minimig_floppy_requested_count(minimig_config.floppy.drives);
}

static const char *m_floppy_extensions(void) { return caps_init() ? "ADFSCPIPF" : "ADFSCP"; }

static uint16_t m_ide_check(void) { return ide_check(); }

static void m_ide_reg_set(int port, uint16_t reg, uint16_t value)
{
	if (port < 0 || port > 1) return;
	ide_reg_set(&ide_inst[port], reg, value);
}

static void m_ide_recvbuf(int port, uint16_t reg, uint32_t words, uint16_t *data)
{
	if (port < 0 || port > 1 || !data) return;
	ide_recvbuf(&ide_inst[port], reg, words, data);
}

static void m_ide_sendbuf(int port, uint16_t reg, uint32_t words, const uint16_t *data)
{
	if (port < 0 || port > 1 || !data) return;
	ide_sendbuf(&ide_inst[port], reg, words, (uint16_t*)data);
}

static drive_t *m_drive(int port, int drv)
{
	if (port < 0 || port > 1 || drv < 0 || drv > 1) return NULL;
	return &ide_inst[port].drive[drv];
}

static int m_drive_info(int port, int drv, mnu_mm_drive_t *info)
{
	drive_t *d = m_drive(port, drv);
	if (!d || !info || info->size < sizeof(mnu_mm_drive_t)) return 0;
	info->present = (d->present && d->f) ? 1 : 0;
	info->cd = d->cd;
	info->cylinders = d->cylinders;
	info->heads = d->heads;
	info->spt = d->spt;
	info->total_sectors = d->total_sectors;
	return 1;
}

static int m_drive_read(int port, int drv, uint32_t lba, uint8_t *buf, uint32_t cnt)
{
	drive_t *d = m_drive(port, drv);
	if (!d || !d->present || !d->f || d->cd || !buf) return 0;
	return ide_hdd_read(d, lba, buf, cnt);
}

static int m_drive_write(int port, int drv, uint32_t lba, const uint8_t *buf, uint32_t cnt)
{
	drive_t *d = m_drive(port, drv);
	if (!d || !d->present || !d->f || d->cd || !buf) return 0;
	return ide_hdd_write(d, lba, buf, cnt);
}

static void m_net_poll(void) { a2065_poll(); }

static void m_config_video(void) { minimig_ConfigVideo(minimig_config.scanlines); }
static void m_config_audio(void) { minimig_ConfigAudio(minimig_config.audio); }
static void m_set_extcfg(uint32_t ext) { minimig_set_extcfg(ext); }
static void m_config_autofire(uint8_t mask) { minimig_ConfigAutofire(minimig_config.autofire, mask); }
static int  m_get_adjust(void) { return minimig_get_adjust(); }
static void m_set_adjust(int on) { minimig_set_adjust(on ? 1 : 0); }
static int  m_dsnd_loaded(void) { return minimig_dsnd_loaded(); }
static const char *m_cfg_info(int num, int label) { return minimig_get_cfg_info(num, label); }

static int  m_net_modes(void) { return A2065_MODES; }
static int  m_net_get_iface(void) { return a2065_get_iface(); }
static void m_net_set_iface(int mode) { a2065_set_iface(mode); }
static int  m_net_mode_available(int mode) { return a2065_mode_available(mode); }
static const char *m_net_iface_name(int mode) { return a2065_iface_msg(mode); }

static mnu_minimig_api_t minimig_api =
{
	sizeof(mnu_minimig_api_t),

	&minimig_config,
	m_reset,
	m_cfg_save,
	m_cfg_load,
	m_config_cpu,
	m_config_chipset,
	m_config_floppy,
	m_set_kickstart,
	m_set_extrom,
	m_get_extrom,
	m_get_extcfg,
	m_set_hrtmon_a1200,

	m_floppy_status,
	m_floppy_insert,
	m_floppy_eject,
	m_floppy_active_count,
	m_floppy_requested_count,
	m_floppy_step_drives,
	m_floppy_extensions,

	m_ide_check,
	m_ide_reg_set,
	m_ide_recvbuf,
	m_ide_sendbuf,
	m_drive_info,
	m_drive_read,
	m_drive_write,

	m_net_poll,

	m_config_video,
	m_config_audio,
	m_set_extcfg,
	m_config_autofire,
	m_get_adjust,
	m_set_adjust,
	m_dsnd_loaded,

	m_cfg_info,

	m_net_modes,
	m_net_get_iface,
	m_net_set_iface,
	m_net_mode_available,
	m_net_iface_name,
};

const void *mnu_minimig_api() { return &minimig_api; }
