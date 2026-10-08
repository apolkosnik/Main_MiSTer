#ifndef FALCON_CONFIG_H
#define FALCON_CONFIG_H

// Slot 0 is loaded at startup; slots 1..8 are selected from the OSD.
void falcon_config_init();
bool falcon_config_boot();
bool falcon_config_load(unsigned slot);
bool falcon_config_save(unsigned slot);
bool falcon_config_describe(unsigned slot, char *text, unsigned size);
const char *falcon_config_error();

// Track successful transfers and mounted media, including boot/MGL loads.
void falcon_config_record_rom(unsigned index, const char *path, bool composite);
void falcon_config_record_disk(unsigned index, const char *path);
const char *falcon_config_rom(unsigned index);
const char *falcon_config_disk(unsigned index);

#endif
