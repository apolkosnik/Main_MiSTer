
#ifndef __MINIMIG_ETHERNET_H__
#define __MINIMIG_ETHERNET_H__

#include <stdint.h>



// #define IDE0_BASE 0xF000
// #define IDE1_BASE 0xF100
// #define FDD0_BASE   0xF200
// #define FDD1_BASE   0xF300
// #define RTC_BASE    0xF400

// Let's go for 0xF500 instead
#define ETH_BASE        0xF500
// Ethernet address space (0xF500-0xF57F)


// Ethernet registers
#define ETH_CTRL        (ETH_BASE + 0x00)   // Control register
#define ETH_STATUS      (ETH_BASE + 0x01)   // Status register
#define ETH_MAC_LOW     (ETH_BASE + 0x02)   // MAC address low 32 bits
#define ETH_MAC_HIGH    (ETH_BASE + 0x04)   // MAC address high 16 bits
#define ETH_TX_LEN      (ETH_BASE + 0x06)   // TX packet length
#define ETH_RX_LEN      (ETH_BASE + 0x07)   // RX packet length
#define ETH_TX_BUF      (ETH_BASE + 0x10)   // TX buffer start (0xF510)
#define ETH_RX_BUF      (ETH_BASE + 0x50)   // RX buffer start (0xF550)

// Control register bits
#define ETH_CTRL_TX_START   0x0001
#define ETH_CTRL_RX_ENABLE  0x0002
#define ETH_CTRL_RESET      0x0004

// Status register bits
#define ETH_STATUS_TX_BUSY  0x0001
#define ETH_STATUS_RX_READY 0x0002
#define ETH_STATUS_TX_DONE  0x0004
#define ETH_STATUS_RX_ERROR 0x0008


// Initial setup for the ethernet
void minimig_ethernet_init();

// Reset the ethernet plumbing 
void minimig_ethernet_reset();

// Main loop for handlling the ethernet
void minimig_ethernet_poll();


#endif
