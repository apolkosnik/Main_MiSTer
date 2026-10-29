/*
 * mnu_minimig.h - the Minimig family's services for a menu module (.mnu)
 *
 * A module whose descriptor says MNU_FAMILY_MINIMIG has its core run with the
 * Minimig protocol, and gets this table from
 * host->family_api(MNU_FAMILY_MINIMIG). The configuration is the host's own
 * minimig_config (support/minimig/minimig_config.h, built with the same
 * toolchain): a module edits it in place and calls the config_* functions to
 * send what changed, or reset() to apply all of it, as the stock menus do.
 *
 * Everything here runs on the caller's coroutine and returns promptly, except
 * the disk transfers, which read or write the image file.
 */
#ifndef MNU_MINIMIG_H
#define MNU_MINIMIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* floppy_status() */
#define MNU_MM_DSK_INSERTED  0x01
#define MNU_MM_DSK_WRITABLE  0x10

/* drive_info() */
typedef struct mnu_mm_drive {
	uint32_t size;               /* sizeof(mnu_mm_drive_t) */
	uint8_t  present;            /* an image is open */
	uint8_t  cd;                 /* a CD-ROM, not a hard disk */
	uint16_t cylinders;
	uint16_t heads;
	uint16_t spt;
	uint32_t total_sectors;      /* the made-up RDB included */
} mnu_mm_drive_t;

typedef struct mnu_minimig_api {
	uint32_t size;               /* sizeof(mnu_minimig_api_t) */

	/* --- configuration --- */
	void *config;                /* mm_configTYPE *, the host's minimig_config */
	void (*reset)(void);         /* apply the whole configuration and reset the Amiga */
	int  (*cfg_save)(int num);   /* 0 = <core>.cfg, 1.. = <core><num>.cfg */
	int  (*cfg_load)(int num);   /* load it (no reset) */
	void (*config_cpu)(void);    /* send config->cpu */
	void (*config_chipset)(void);
	void (*config_floppy)(void); /* send the drive count and speed */
	void (*set_kickstart)(const char *path);   /* takes effect at reset() */
	void (*set_extrom)(const char *path);      /* "" = none */
	const char *(*get_extrom)(void);
	uint32_t (*get_extcfg)(void);              /* ext_cfg | ext_cfg2 << 16 */
	/* The HRTmon cartridge's "A1200" byte (Gayle's IDE at $DA2000): 0xFF as
	 * Minimig has it, 0 for a machine with its own IDE port or none. */
	void (*set_hrtmon_a1200)(uint8_t value);

	/* --- floppies (drive 0-3) --- */
	int  (*floppy_status)(int drive, char *name, int namelen);   /* MNU_MM_DSK_*, name of the image */
	void (*floppy_insert)(int drive, const char *path);
	void (*floppy_eject)(int drive);
	int  (*floppy_active_count)(void);         /* drives the core has now */
	int  (*floppy_requested_count)(void);      /* drives the configuration asks for */
	int  (*floppy_step_drives)(int direction); /* add (+1) or remove (-1) a drive, sent to the core; returns the new count */
	const char *(*floppy_extensions)(void);    /* what the file selector should offer */

	/* --- IDE channels (port 0, 1) and their drives (drv 0, 1) --- */
	uint16_t (*ide_check)(void);               /* the core's request bits, as ide_check() */
	void (*ide_reg_set)(int port, uint16_t reg, uint16_t value);
	void (*ide_recvbuf)(int port, uint16_t reg, uint32_t words, uint16_t *data);
	void (*ide_sendbuf)(int port, uint16_t reg, uint32_t words, const uint16_t *data);
	int  (*drive_info)(int port, int drv, mnu_mm_drive_t *info);   /* 1 = filled */
	/* cnt sectors at lba, as the IDE sees the drive (the made-up RDB below the
	 * image); 1 = done */
	int  (*drive_read)(int port, int drv, uint32_t lba, uint8_t *buf, uint32_t cnt);
	int  (*drive_write)(int port, int drv, uint32_t lba, const uint8_t *buf, uint32_t cnt);

	/* --- background work a long exchange with the core must keep going --- */
	void (*net_poll)(void);      /* the A2065 */

	/* --- video, audio, input --- */
	void (*config_video)(void);  /* send config->scanlines */
	void (*config_audio)(void);  /* send config->audio */
	void (*set_extcfg)(uint32_t ext);          /* store and send ext_cfg | ext_cfg2 << 16 */
	void (*config_autofire)(uint8_t mask);     /* send config->autofire's bits in mask */
	int  (*get_adjust)(void);    /* the screen position adjustment is on */
	void (*set_adjust)(int on);
	int  (*dsnd_loaded)(void);   /* the drive sounds' samples are there */

	/* --- configuration slots (0 = <core>.cfg, 1-9) --- */
	/* The slot's info line ("" without one), its label instead when label is
	 * set and it has one, NULL when the slot is empty. */
	const char *(*cfg_info)(int num, int label);

	/* --- the A2065's host side: OFF, eth0, eth1, macvlan, tap0 --- */
	int  (*net_modes)(void);
	int  (*net_get_iface)(void);
	void (*net_set_iface)(int mode);
	int  (*net_mode_available)(int mode);
	const char *(*net_iface_name)(int mode);
} mnu_minimig_api_t;

#ifdef __cplusplus
}
#endif
#endif /* MNU_MINIMIG_H */
