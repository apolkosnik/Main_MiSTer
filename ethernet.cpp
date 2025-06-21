/* Ethernet handling */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "minimig_ethernet.h"
#include "../../user_io.h"
#include "../../spi.h"
#include "../../cfg.h"
#include "../../shmem.h"



static void eth_dma_write(uint32_t address, uint16_t data)
{
    EnableIO();
    spi8(UIO_DMA_WRITE);    // 0x61
    spi32_w(address);
    spi_w(data);
    DisableIO();
}

static uint16_t eth_dma_read(uint32_t address)
{
    uint16_t result;
    EnableIO();
    spi8(UIO_DMA_READ);     // 0x62
    spi32_w(address);
    spi_w(0);
    result = spi_w(0);
    DisableIO();
    return result;
}

static void eth_dma_write_buf(uint32_t address, uint16_t *data, uint32_t length)
{
    EnableIO();
    fpga_spi_fast(UIO_DMA_WRITE);
    fpga_spi_fast(address);
    fpga_spi_fast(0);
    fpga_spi_fast_block_write(data, length);
    DisableIO();
}

static void eth_dma_read_buf(uint32_t address, uint16_t *data, uint32_t length)
{
    EnableIO();
    fpga_spi_fast(UIO_DMA_READ);
    fpga_spi_fast(address);
    fpga_spi_fast(0);
    fpga_spi_fast_block_read(data, length);
    DisableIO();
}

void eth_init(void)
{
    // Reset ethernet controller
    eth_dma_write(ETH_CTRL, ETH_CTRL_RESET);
    usleep(1000);

    // Enable RX
    eth_dma_write(ETH_CTRL, ETH_CTRL_RX_ENABLE);

    printf("Ethernet controller initialized\n");
}

void eth_set_mac_address(const uint8_t *mac)
{
    uint32_t mac_low = mac[0] | (mac[1] << 8) | (mac[2] << 16) | (mac[3] << 24);
    uint16_t mac_high = mac[4] | (mac[5] << 8);

    eth_dma_write(ETH_MAC_LOW, mac_low & 0xFFFF);
    eth_dma_write(ETH_MAC_LOW + 1, mac_low >> 16);
    eth_dma_write(ETH_MAC_HIGH, mac_high);

    printf("MAC address set to %02X:%02X:%02X:%02X:%02X:%02X\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

int eth_send_packet(const uint8_t *packet, uint32_t length)
{
    if (length > 1518) {  // Max ethernet frame size
        printf("Packet too large: %d bytes\n", length);
        return -1;
    }

    // Check if transmitter is busy
    uint16_t status = eth_dma_read(ETH_STATUS);
    if (status & ETH_STATUS_TX_BUSY) {
        printf("Transmitter busy\n");
        return -1;
    }

    // Set packet length
    eth_dma_write(ETH_TX_LEN, length);

    // Copy packet to TX buffer (convert bytes to 16-bit words)
    uint16_t *buf = (uint16_t*)malloc((length + 1) / 2 * sizeof(uint16_t));
    for (uint32_t i = 0; i < length; i += 2) {
        buf[i/2] = packet[i] | ((i+1 < length) ? (packet[i+1] << 8) : 0);
    }

    eth_dma_write_buf(ETH_TX_BUF, buf, (length + 1) / 2);
    free(buf);

    // Start transmission
    eth_dma_write(ETH_CTRL, ETH_CTRL_RX_ENABLE | ETH_CTRL_TX_START);

    printf("Sent packet: %d bytes\n", length);
    return 0;
}

int eth_recv_packet(uint8_t *packet, uint32_t max_length)
{
    // Check if packet is ready
    uint16_t status = eth_dma_read(ETH_STATUS);
    if (!(status & ETH_STATUS_RX_READY)) {
        return 0;  // No packet available
    }

    if (status & ETH_STATUS_RX_ERROR) {
        printf("RX error detected\n");
        // Clear error by reading packet anyway
    }

    // Get packet length
    uint16_t length = eth_dma_read(ETH_RX_LEN);
    if (length > max_length) {
        printf("Received packet too large: %d bytes (max %d)\n", length, max_length);
        length = max_length;
    }

    if (length == 0) {
        return 0;
    }

    // Read packet from RX buffer
    uint16_t *buf = (uint16_t*)malloc((length + 1) / 2 * sizeof(uint16_t));
    eth_dma_read_buf(ETH_RX_BUF, buf, (length + 1) / 2);

    // Convert 16-bit words back to bytes
    for (uint32_t i = 0; i < length; i++) {
        packet[i] = (i & 1) ? (buf[i/2] >> 8) : (buf[i/2] & 0xFF);
    }

    free(buf);

    printf("Received packet: %d bytes\n", length);
    return length;
}

uint16_t eth_get_status(void)
{
    return eth_dma_read(ETH_STATUS);
}

int eth_tx_done(void)
{
    uint16_t status = eth_dma_read(ETH_STATUS);
    return (status & ETH_STATUS_TX_DONE) ? 1 : 0;
}

void eth_poll(void)
{
    static uint8_t rx_buffer[1600];
    static uint32_t last_status = 0;

    uint16_t status = eth_get_status();

    // Check for new status
    if (status != last_status) {
        if (status & ETH_STATUS_TX_DONE) {
            printf("TX completed\n");
            // Clear TX done flag by reading it
        }
        if (status & ETH_STATUS_RX_ERROR) {
            printf("RX error occurred\n");
        }
        last_status = status;
    }

    // Check for received packets
    int rx_len = eth_recv_packet(rx_buffer, sizeof(rx_buffer));
    if (rx_len > 0) {
        // Process received packet
        printf("Processing packet of %d bytes\n", rx_len);
        // Add your packet processing here
    }
}

// Initial setup for the ethernet
void minimig_ethernet_init(){

// Initialize ethernet
    uint8_t mac[6] = {0x02, 0x00, 0x00, 0x12, 0x34, 0x56};
    eth_init();
    eth_set_mac_address(mac);
}

// Reset the ethernet plumbing 
void minimig_ethernet_reset(){

    // Reset ethernet controller
    eth_dma_write(ETH_CTRL, ETH_CTRL_RESET);
    usleep(1000);

}

// Main loop for handlling the ethernet
void minimig_ethernet_poll(){

    // Send a packet
    eth_send_packet(packet, sizeof(packet));

    // Poll for responses
    for (int i = 0; i < 100; i++) {
        eth_poll();
        usleep(10000);  // 10ms delay
    }

}




// // Example usage
// void eth_example(void)
// {
//     uint8_t mac[6] = {0x02, 0x00, 0x00, 0x12, 0x34, 0x56};
//     uint8_t packet[] = {
//         // Destination MAC (broadcast)
//         0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
//         // Source MAC
//         0x02, 0x00, 0x00, 0x12, 0x34, 0x56,
//         // EtherType (ARP)
//         0x08, 0x06,
//         // ARP packet data...
//         0x00, 0x01, 0x08, 0x00, 0x06, 0x04, 0x00, 0x01,
//         // ... (rest of ARP packet)
//     };

//     // Initialize ethernet
//     eth_init();
//     eth_set_mac_address(mac);

//     // Send a packet
//     eth_send_packet(packet, sizeof(packet));

//     // Poll for responses
//     for (int i = 0; i < 100; i++) {
//         eth_poll();
//         usleep(10000);  // 10ms delay
//     }
// }
