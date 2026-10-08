#ifndef FALCON_NVRAM_H
#define FALCON_NVRAM_H

// Call after status/configuration has been sent, before loading the boot ROM.
void falcon_nvram_load();
// Service the FPGA's delayed save requests in the normal user_io poll loop.
void falcon_nvram_poll();

#endif
