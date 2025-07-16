#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/if_packet.h>
#include <linux/if_ether.h>
#include <ifaddrs.h>
#include <errno.h>

#include "minimig_eth.h"
#include "../../shmem.h"
#include "../../hardware.h"
#include "../../user_io.h"
#include "../../spi.h"


#define ETH_DEBUG

#ifdef ETH_DEBUG
    #define eth_debug printf
#else
    #define eth_debug(x,...) void()
#endif


#ifdef ETH_DEBUG
	#define dbg_print printf
	#define dbg_hexdump hexdump
#else
	#define dbg_print(x,...) void()
	#define dbg_hexdump(x,...) void()
#endif

#define SWAP_INT(a) ((((a)&0x000000ff)<<24)|(((a)&0x0000ff00)<<8)|(((a)&0x00ff0000)>>8)|(((a)&0xff000000)>>24))

// The DDR3 physical address for ethernet shared memory
// This corresponds to the ramaddr mapping above
// ramaddr = {1'b0, 1'b0, 4'b1101, 7'b0, offset[14:1]}
// Which gives us physical address 0x1A000000 + offset



static int raw_socket = -1;
static uint8_t *eth_shmem = 0;
static uint32_t hps_heartbeat_counter = 0;
static struct sockaddr_ll sock_addr;
static char bridge_interface[16] = "eth0";

// Default MAC address for X-Surf 100
static uint8_t default_mac[6] = {0x28, 0x12, 0x34, 0x56, 0x78, 0x9A};

// Access ethernet shared memory through mapped region
static void eth_write_shared_mem(uint32_t offset, const void *data, uint32_t size)
{
    if (!eth_shmem || eth_shmem == (uint8_t *)-1) {
        eth_debug("eth_write_shared_mem: no eth_shmem!\n");
        return;
    }
    if (offset + size > ETH_SHMEM_SIZE) {
        eth_debug("ETH: Write beyond shared memory bounds: offset=0x%X, size=%d\n", offset, size);
        return;
    }
    
    //eth_debug("ETH: Writing %d bytes to offset 0x%X\n", size, offset);
    memcpy(eth_shmem + offset, data, size);
}

static void eth_read_shared_mem(uint32_t offset, void *data, uint32_t size)
{
    if (!eth_shmem || eth_shmem == (uint8_t *)-1) {
        eth_debug("eth_read_shared_mem: no eth_shmem!\n");
        memset(data, 0, size);
        return;
    }
    
    if (offset + size > ETH_SHMEM_SIZE) {
        eth_debug("ETH: Read beyond shared memory bounds: offset=0x%X, size=%d\n", offset, size);
        memset(data, 0, size);
        return;
    }
    
    //eth_debug("ETH: Reading %d bytes from offset 0x%X\n", size, offset);
    memcpy(data, eth_shmem + offset, size);
}

static void eth_write_shared_u32(uint32_t offset, uint32_t value)
{
    eth_write_shared_mem(offset, &value, 4);
}

static void eth_write_shared_reg(uint32_t offset, uint8_t value)
{
    // Write 8-bit register value to LSB of 32-bit word
    uint32_t reg_val = (uint32_t)value;
    eth_write_shared_mem(offset, &reg_val, 4);
}

static uint8_t eth_read_shared_reg(uint32_t offset)
{
    // Read 8-bit register value from LSB of 32-bit word
    uint32_t reg_val;
    eth_read_shared_mem(offset, &reg_val, 4);
    return (uint8_t)(reg_val & 0xFF);
}

static uint32_t eth_read_shared_u32(uint32_t offset)
{
    uint32_t value;
    eth_read_shared_mem(offset, &value, 4);
    return value;
}

// Helper functions to access NE2000 memory directly from shared memory
static uint8_t read_ne_memory(uint16_t addr)
{
    if (addr >= NE_MEM_SIZE) {
        eth_debug("NE2000 memory read beyond bounds: addr=0x%04X\n", addr);
        return 0;
    }
    uint8_t value;
    eth_read_shared_mem(ETH_NE_MEMORY + addr, &value, 1);
    return value;
}

static void write_ne_memory(uint16_t addr, uint8_t value)
{
    if (addr >= NE_MEM_SIZE) {
        eth_debug("NE2000 memory write beyond bounds: addr=0x%04X\n", addr);
        return;
    }
    eth_write_shared_mem(ETH_NE_MEMORY + addr, &value, 1);
}

// read_ne_memory_block removed - unused after optimizations

static void write_ne_memory_block(uint16_t addr, const void* data, uint16_t size)
{
    if (addr + size > NE_MEM_SIZE) {
        eth_debug("NE2000 memory block write beyond bounds: addr=0x%04X, size=%d\n", addr, size);
        return;
    }
    eth_write_shared_mem(ETH_NE_MEMORY + addr, data, size);
}

// Helper functions to access NE2000 registers directly from shared memory
static uint8_t read_ne_register(uint8_t page, uint8_t reg)
{
    if (page > 3 || reg > 15) {
        eth_debug("NE2000 register read beyond bounds: page=%d, reg=0x%02X\n", page, reg);
        return 0;
    }
    // Registers are stored as 32-bit aligned values in shared memory
    // Each page has 16 registers, each taking 4 bytes
    uint32_t offset = (page * 16 + reg) * 4;
    return eth_read_shared_reg(ETH_CTRL_REGS + offset);
}

static void write_ne_register(uint8_t page, uint8_t reg, uint8_t value)
{
    if (page > 3 || reg > 15) {
        eth_debug("NE2000 register write beyond bounds: page=%d, reg=0x%02X\n", page, reg);
        return;
    }
    // Registers are stored as 32-bit aligned values in shared memory
    // Each page has 16 registers, each taking 4 bytes
    uint32_t offset = (page * 16 + reg) * 4;
    eth_write_shared_reg(ETH_CTRL_REGS + offset, value);
}

// DMA is handled entirely by FPGA - no HPS tracking needed

// Helper functions to access RTL8019 state structure in shared memory
// static struct rtl8019_state* get_eth_state() {
//     if (!eth_shmem || eth_shmem == (uint8_t *)-1) return NULL;
//     return (struct rtl8019_state*)(eth_shmem + ETH_RTL8019_STATE);
// }

static void write_eth_state(const struct rtl8019_state* state) {
    if (!eth_shmem || eth_shmem == (uint8_t *)-1) {
        eth_debug("write_eth_state: no eth_shmem!\n");
        return;
    }
    eth_write_shared_mem(ETH_RTL8019_STATE, state, sizeof(struct rtl8019_state));
}

static void read_eth_state(struct rtl8019_state* state) {
    if (!eth_shmem || eth_shmem == (uint8_t *)-1) {
        eth_debug("read_eth_state: no eth_shmem!\n");
        memset(state, 0, sizeof(struct rtl8019_state));
        return;
    }
    eth_read_shared_mem(ETH_RTL8019_STATE, state, sizeof(struct rtl8019_state));
}

// Initialize ethernet emulation
void minimig_eth_init()
{
    // Map the ethernet shared memory region
    eth_debug("ETH: Initializing ethernet emulation...\n");
    eth_shmem = (uint8_t *)shmem_map(ETH_SHMEM_ADDR, ETH_SHMEM_SIZE);
    if (!eth_shmem) {
        eth_debug("Failed to map ethernet shared memory!\n");
        return;
    } else {
        eth_debug("ETH: Successfully mapped shared memory at 0x%08X\n", ETH_SHMEM_ADDR);
    }

    eth_debug("Initializing RTL8019 ethernet emulation\n");
    
    // Initialize registers to reset state (this will initialize the shared memory state)
    minimig_eth_reset();
    
    // Initialize shared memory with proper values
    eth_debug("ETH: Initializing shared memory...\n");
    
    // Clear all shared memory
    memset(eth_shmem, 0, ETH_SHMEM_SIZE);
    
    // Write HPS signature
    eth_write_shared_u32(ETH_HPS_SIGNATURE, 0xCAFEBABE);
    
    // Write initial MAC address
    eth_write_shared_mem(ETH_CTRL_MAC, default_mac, 6);
    
    // Register state is already written by minimig_eth_reset() above
    
    // Initialize control flags
    eth_write_shared_u32(ETH_CTRL_FLAGS, ETH_FLAG_ENABLED);
    
    // Initialize packet buffers with pattern for testing (LSB format)
    //uint8_t test_pattern[16] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11,
    //                            0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99};
    //eth_write_shared_mem(ETH_RX_BUFFER, test_pattern, 16);
    
    eth_debug("ETH: Shared memory initialized with signature 0x%08X at %0X \n", 0xCAFEBABE, (ETH_SHMEM_ADDR + ETH_HPS_SIGNATURE));
    
    // Try to open raw socket for ethernet bridging
    eth_debug("ETH: Opening raw socket for bridging...\n");
    raw_socket = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (raw_socket < 0) {
        eth_debug("ETH: Warning: Could not open raw socket for ethernet bridging: %s\n", strerror(errno));
        return;
    }
    eth_debug("ETH: Raw socket opened successfully\n");
    
    // Find and bind to ethernet interface
    struct ifaddrs *ifaddr, *ifa;
    if (getifaddrs(&ifaddr) == -1) {
        eth_debug("Failed to get interface addresses\n");
        close(raw_socket);
        raw_socket = -1;
        return;
    }
    
    bool interface_found = false;
    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == NULL) continue;
        
        if (ifa->ifa_addr->sa_family == AF_PACKET && 
            strncmp(ifa->ifa_name, "eth", 3) == 0) {
            strcpy(bridge_interface, ifa->ifa_name);
            interface_found = true;
            break;
        }
    }
    freeifaddrs(ifaddr);
    
    if (!interface_found) {
        eth_debug("ETH: Warning: No ethernet interface found for bridging\n");
        close(raw_socket);
        raw_socket = -1;
        return;
    }
    eth_debug("ETH: Found ethernet interface: %s\n", bridge_interface);
    
    // Get interface index
    struct ifreq ifr;
    strncpy(ifr.ifr_name, bridge_interface, IFNAMSIZ-1);
    if (ioctl(raw_socket, SIOCGIFINDEX, &ifr) < 0) {
        eth_debug("Failed to get interface index: %s\n", strerror(errno));
        close(raw_socket);
        raw_socket = -1;
        return;
    }
    
    // Bind socket to interface
    memset(&sock_addr, 0, sizeof(sock_addr));
    sock_addr.sll_family = AF_PACKET;
    sock_addr.sll_protocol = htons(ETH_P_ALL);
    sock_addr.sll_ifindex = ifr.ifr_ifindex;
    
    if (bind(raw_socket, (struct sockaddr*)&sock_addr, sizeof(sock_addr)) < 0) {
        eth_debug("ETH: Failed to bind raw socket: %s\n", strerror(errno));
        close(raw_socket);
        raw_socket = -1;
        return;
    }
    eth_debug("ETH: Socket bound to interface successfully\n");
    
    // Print summary of ethernet initialization
    eth_debug("===============================================\n");
    eth_debug("ETH: X-Surf 100 Ethernet Initialized!\n");
    eth_debug("ETH: Amiga Address: 0x%06X\n", ETH_BOARD_ADDR);
    eth_debug("ETH: HPS Memory:    0x%08X\n", ETH_SHMEM_ADDR);
    eth_debug("ETH: MAC Address:   %02X:%02X:%02X:%02X:%02X:%02X\n",
           default_mac[0], default_mac[1], default_mac[2],
           default_mac[3], default_mac[4], default_mac[5]);
    eth_debug("ETH: Interface:     %s\n", bridge_interface);
    eth_debug("===============================================\n");
    
    // Set socket to non-blocking
    int flags = fcntl(raw_socket, F_GETFL, 0);
    fcntl(raw_socket, F_SETFL, flags | O_NONBLOCK);
    
    eth_debug("RTL8019 ethernet emulation initialized on interface %s\n", bridge_interface);
}

// Reset RTL8019 to initial state
void minimig_eth_reset()
{
    eth_debug("ETH: RTL8019 RESET triggered!\n");
    
    //minimig_eth_test();

    // Initialize RTL8019 state structure in shared memory
    struct rtl8019_state state;
    memset(&state, 0, sizeof(state));
    
    // Initialize page 0 registers directly in shared memory
    write_ne_register(0, 0x00, NE_CR_STP | NE_CR_RDMA_NOT);  // CR: Stop, no remote DMA
    write_ne_register(0, 0x07, NE_ISR_RST);                  // ISR: Reset complete
    write_ne_register(0, 0x0E, 0x48);                        // DCR: Normal operation, FIFO threshold
    write_ne_register(0, 0x0D, 0x00);                        // TCR: Normal operation
    write_ne_register(0, 0x0C, 0x00);                        // RCR: Reject all packets initially
    
    // Set up buffer boundaries
    write_ne_register(0, 0x01, NE_RX_START);                 // PSTART
    write_ne_register(0, 0x02, NE_RX_STOP);                  // PSTOP
    write_ne_register(0, 0x03, NE_RX_START);                 // BNRY
    write_ne_register(0, 0x04, NE_TX_START);                 // TPSR
    
    // Initialize MAC address
    memcpy(state.mac_addr, default_mac, 6);
    
    // Initialize page 1 registers (MAC address) directly in shared memory
    for (int i = 0; i < 6; i++) {
        write_ne_register(1, 0x01 + i, state.mac_addr[i]);  // PAR0-PAR5
    }
    write_ne_register(1, 0x07, NE_RX_START + 1);             // CURR
    
    // Initialize minimal HPS state (FPGA handles all register state)
    state.current_page = 0;
    state.enabled = false;
    state.link_up = false;
    
    // Packet statistics for HPS debugging only (FPGA has its own counters)
    state.tx_packets = 0;
    state.rx_packets = 0;
    state.tx_errors = 0;
    state.rx_errors = 0;
    
    // Write the complete state to shared memory
    write_eth_state(&state);
    
    // Update shared memory with reset state
    eth_write_shared_u32(ETH_CTRL_FLAGS, ETH_FLAG_RESET);
    
    // Write MAC address to shared memory
    eth_write_shared_mem(ETH_CTRL_MAC, state.mac_addr, 6);
    
    // Write HPS signature to show we're alive
    eth_write_shared_u32(ETH_HPS_SIGNATURE, 0xCAFEBABE);
}

// Read from RTL8019 register (called when Amiga accesses ethernet card)
uint8_t minimig_eth_read_reg(uint32_t addr)
{
    // Convert 32-bit aligned address to register number
    // addr is offset from register base, divide by 4 to get register number
    uint8_t reg = (addr >> 2) & 0x1F;  // Each register is 4 bytes apart
    uint8_t value = 0;
    
    static int read_count = 0;
    if (++read_count <= 10 || (read_count % 100) == 0) {
        eth_debug("ETH: Amiga READ  reg[0x%02X] = 0x%02X (count: %d)\n", reg, value, read_count);
    }
    
    if (reg == 0x1F) {  // NE_RESET register number
        // Reading reset port triggers reset
        minimig_eth_reset();
        return 0;
    }
    
    if (reg == 0x10) {  // NE_DATAPORT register number
        // Data port access
        return minimig_eth_read_data() & 0xFF;
    }
    
    // Regular register read - FPGA handles all register logic
    uint8_t cr_reg = read_ne_register(0, 0x00);  // Get CR register
    uint8_t page = (cr_reg & (NE_CR_PS0 | NE_CR_PS1)) >> 6;  // Get page from CR register
    
    if (reg < 16) {
        value = read_ne_register(page, reg);
        // FPGA handles all register logic, interrupts, and status updates
    }
    
    eth_debug("RTL8019 read reg[%d][0x%02X] = 0x%02X\n", page, reg, value);
    return value;
}

// Write to RTL8019 register (called when Amiga accesses ethernet card)
void minimig_eth_write_reg(uint32_t addr, uint8_t data)
{
    // Convert 32-bit aligned address to register number
    // addr is offset from register base, divide by 4 to get register number
    uint8_t reg = (addr >> 2) & 0x1F;  // Each register is 4 bytes apart
    
    static int write_count = 0;
    if (++write_count <= 10 || (write_count % 100) == 0) {
        eth_debug("ETH: Amiga WRITE reg[0x%02X] = 0x%02X (count: %d)\n", reg, data, write_count);
    }
    
    if (reg == 0x1F) {  // NE_RESET register number
        // Writing to reset port triggers reset
        minimig_eth_reset();
        return;
    }
    
    if (reg == 0x10) {  // NE_DATAPORT register number
        // Data port access
        minimig_eth_write_data(data);
        return;
    }
    
    // Get current page from CR register directly
    uint8_t cr_reg = read_ne_register(0, 0x00);  // Get CR register
    uint8_t page = (cr_reg & (NE_CR_PS0 | NE_CR_PS1)) >> 6;  // Get page from CR register
    
    // Get current state from shared memory for non-register state variables
    struct rtl8019_state state;
    read_eth_state(&state);
    
    eth_debug("RTL8019 write reg[%d][0x%02X] = 0x%02X\n", page, reg, data);
    
    if (reg < 16) {
        // FPGA handles all register logic including:
        // - Command register start/stop/transmit/DMA operations
        // - ISR register interrupt management
        // - DMA address/count tracking
        // - MAC address updates
        // HPS simply passes register writes to shared memory
        write_ne_register(page, reg, data);
        
        // Only track enabled state for packet filtering
        if (reg == 0x00) {  // Command register
            state.enabled = (data & NE_CR_STA) != 0;
            
            // Handle transmit packet command for HPS packet processing
            if (data & NE_CR_TXP) {
                eth_debug("RTL8019 transmit packet requested\n");
                uint32_t flags = eth_read_shared_u32(ETH_CTRL_FLAGS);
                eth_write_shared_u32(ETH_CTRL_FLAGS, flags | ETH_FLAG_TX_REQ);
            }
        }
        
        // Track MAC address updates for packet filtering
        if (page == 1 && reg >= 0x01 && reg <= 0x06) {  // PAR0-PAR5
            state.mac_addr[reg - 0x01] = data;
            eth_debug("ETH: MAC[%d] = 0x%02X\n", reg - 0x01, data);
        }
    }
    
    // Write minimal state back to shared memory
    write_eth_state(&state);
    
    // Update MAC address in shared memory control structure
    if (reg >= 0x01 && reg <= 0x06 && page == 1) {  // PAR0-PAR5
        eth_write_shared_mem(ETH_CTRL_MAC, state.mac_addr, 6);
    }
}

// Read data from RTL8019 data port
// FPGA handles all DMA address/count management automatically
uint16_t minimig_eth_read_data()
{
    static int data_read_count = 0;
    
    // Get current DMA state from registers (FPGA manages these)
    uint16_t dma_addr = read_ne_register(0, 0x08) | (read_ne_register(0, 0x09) << 8);  // CRDA0/CRDA1
    uint16_t dma_count = read_ne_register(0, 0x0A) | (read_ne_register(0, 0x0B) << 8); // RBCR0/RBCR1
    
    // Enhanced debug for FPGA data port access
    eth_debug("ETH: >>> Data port READ request (call #%d)\n", data_read_count + 1);
    eth_debug("ETH: FPGA DMA state - addr=0x%04X, count=%d\n", dma_addr, dma_count);
    uint8_t dcr_reg = read_ne_register(0, 0x0E);
    uint8_t cr_reg = read_ne_register(0, 0x00);
    eth_debug("ETH: DCR=0x%02X (%s-bit mode), CR=0x%02X (page %d)\n",
           dcr_reg, (dcr_reg & NE_DCR_WTS) ? "16" : "8",
           cr_reg, (cr_reg >> 6) & 3);
    
    if (dma_count == 0) {
        eth_debug("ETH: Data port read with zero DMA count!\n");
        return 0;
    }
    
    uint16_t addr = dma_addr;
    uint16_t data = 0;
    
    // Read from NE2000 memory
    if (addr < NE_MEM_SIZE) {
        data = read_ne_memory(addr);
        if (read_ne_register(0, 0x0E) & NE_DCR_WTS) {  // DCR register
            // 16-bit mode
            if (addr + 1 < NE_MEM_SIZE) {
                data |= (read_ne_memory(addr + 1) << 8);
            }
            // FPGA automatically handles DMA address increment and byte count decrement
            
            // Debug output for 16-bit reads
            if (++data_read_count <= 20 || (data_read_count % 100) == 0) {
                eth_debug("ETH: FPGA Data read [0x%04X] = 0x%04X (16-bit, count:%d) - FPGA handles DMA\n", 
                       addr, data, data_read_count);
                
                // Check if FPGA might be writing in LSB format to shared memory
                if (data == 0) {
                    // Data is zero - might indicate communication issue
                }
            }
        } else {
            // 8-bit mode
            // FPGA automatically handles DMA address increment and byte count decrement
            
            // Debug output for 8-bit reads
            if (data_read_count <= 20 || (data_read_count % 100) == 0) {
                eth_debug("ETH: FPGA Data read [0x%04X] = 0x%02X (8-bit, count:%d) - FPGA handles DMA\n", 
                       addr, data & 0xFF, ++data_read_count);
            }
        }
    } else {
        eth_debug("ETH: Data port read beyond memory bounds: addr=0x%04X\n", addr);
    }
    
    // FPGA automatically handles DMA completion and sets ISR_RDC interrupt
    // No HPS intervention needed for DMA management
    
    return data;
}

// Write data to RTL8019 data port
// FPGA handles all DMA address/count management automatically
void minimig_eth_write_data(uint16_t data)
{
    static int data_write_count = 0;
    
    // Get current DMA state from registers (FPGA manages these)
    uint16_t dma_addr = read_ne_register(0, 0x08) | (read_ne_register(0, 0x09) << 8);  // CRDA0/CRDA1
    uint16_t dma_count = read_ne_register(0, 0x0A) | (read_ne_register(0, 0x0B) << 8); // RBCR0/RBCR1
    
    // Enhanced debug for FPGA data port access
    eth_debug("ETH: >>> Data port WRITE request (call #%d), data=0x%04X\n", data_write_count + 1, data);
    eth_debug("ETH: FPGA DMA state - addr=0x%04X, count=%d\n", dma_addr, dma_count);
    uint8_t dcr_reg = read_ne_register(0, 0x0E);
    uint8_t cr_reg = read_ne_register(0, 0x00);
    eth_debug("ETH: DCR=0x%02X (%s-bit mode), CR=0x%02X (page %d)\n",
           dcr_reg, (dcr_reg & NE_DCR_WTS) ? "16" : "8",
           cr_reg, (cr_reg >> 6) & 3);
    
    if (dma_count == 0) {
        eth_debug("ETH: Data port write with zero DMA count! Data=0x%04X\n", data);
        return;
    }
    
    uint16_t addr = dma_addr;
    
    // Write to NE2000 memory
    if (addr < NE_MEM_SIZE) {
        write_ne_memory(addr, data & 0xFF);
        if (read_ne_register(0, 0x0E) & NE_DCR_WTS) {  // DCR register
            // 16-bit mode
            if (addr + 1 < NE_MEM_SIZE) {
                write_ne_memory(addr + 1, (data >> 8) & 0xFF);
            }
            // FPGA automatically handles DMA address increment and byte count decrement
            
            // Debug output for 16-bit writes
            if (++data_write_count <= 20 || (data_write_count % 100) == 0) {
                eth_debug("ETH: FPGA Data WRITE [0x%04X] = 0x%04X (16-bit, count:%d) - FPGA handles DMA\n", 
                       addr, data, data_write_count);
            }
        } else {
            // 8-bit mode
            // FPGA automatically handles DMA address increment and byte count decrement
            
            // Debug output for 8-bit writes
            if (data_write_count <= 20 || (data_write_count % 100) == 0) {
                eth_debug("ETH: FPGA Data write [0x%04X] = 0x%02X (8-bit, count:%d) - FPGA handles DMA\n", 
                       addr, data & 0xFF, ++data_write_count);
            }
        }
        
        // Hex dump for packet data writes (show first few bytes)
        if (data_write_count <= 50 && addr >= 0x2000) {  // TX buffer area
            eth_debug("ETH: TX Buffer [0x%04X]: %02X %02X\n", 
                   addr, read_ne_memory(addr), 
                   (addr + 1 < NE_MEM_SIZE) ? read_ne_memory(addr + 1) : 0);
        }
    } else {
        eth_debug("ETH: Data port write beyond memory bounds: addr=0x%04X, data=0x%04X\n", addr, data);
    }
    
    // FPGA automatically handles DMA completion and sets ISR_RDC interrupt
    // No HPS intervention needed for DMA management
}

// Transmit packet to host ethernet
void transmit_packet()
{
    // Get current state from shared memory
    struct rtl8019_state state;
    read_eth_state(&state);
    if (raw_socket < 0) return;
    
    uint8_t page = read_ne_register(0, 0x04);  // TPSR register
    uint16_t length = read_ne_register(0, 0x05) |     // TBCR0
                     (read_ne_register(0, 0x06) << 8);   // TBCR1
    
    if (length == 0 || length > 1500) {
        eth_debug("Invalid packet length: %d\n", length);
        return;
    }
    
    uint16_t offset = page * NE_PAGE_SIZE;
    if (offset + length > NE_MEM_SIZE) {
        eth_debug("Packet exceeds memory bounds\n");
        return;
    }
    
    // OPTIMIZATION: No need to copy to separate TX buffer - use NE2000 memory directly
    // Just update packet info for debugging
    eth_write_shared_mem(ETH_PACKET_INFO, &length, 2);
    eth_write_shared_mem(ETH_PACKET_INFO + 2, &offset, 2);
    
    // Debug: Show first 32 bytes of transmitted packet
    eth_debug("ETH: Transmitting packet from offset 0x%04X, length %d bytes:\n", offset, length);
    eth_debug("ETH: TX Data: ");
    for (int i = 0; i < 32 && i < length; i++) {
        uint8_t byte_val = read_ne_memory(offset + i);
        eth_debug("%02X ", byte_val);
        if ((i + 1) % 16 == 0) eth_debug("\nETH: TX Data: ");
    }
    eth_debug("\n");
    
    // Send packet via raw socket - read directly from shared memory
    uint8_t* packet_data = eth_shmem + ETH_NE_MEMORY + offset;
    if (send(raw_socket, packet_data, length, 0) < 0) {
        eth_debug("ETH: Failed to send packet: %s\n", strerror(errno));
        // FPGA handles ISR interrupt flags automatically
        state.tx_errors++;
    } else {
        eth_debug("ETH: Transmitted packet of %d bytes (total TX: %d)\n", length, state.tx_packets + 1);
        // FPGA handles ISR interrupt flags automatically
        state.tx_packets++;
    }
    
    // Write updated state back to shared memory
    write_eth_state(&state);
}

// Receive packet from host ethernet
void receive_packet()
{
    // Get current state from shared memory
    struct rtl8019_state state;
    read_eth_state(&state);
    if (raw_socket < 0 || !state.enabled) return;
    
    uint8_t buffer[1600];
    ssize_t len = recv(raw_socket, buffer, sizeof(buffer), 0);
    
    if (len <= 0) return;
    
    // Filter out our own transmitted packets and non-ethernet frames
    if (len < 14) return;
    
    // Check if packet is for us (broadcast, multicast, or our MAC)
    bool accept = false;
    
    // Broadcast
    if (memcmp(buffer, "\xFF\xFF\xFF\xFF\xFF\xFF", 6) == 0) {
        accept = (read_ne_register(0, 0x0C) & NE_RCR_AB) != 0;  // RCR register
    }
    // Multicast
    else if (buffer[0] & 0x01) {
        accept = (read_ne_register(0, 0x0C) & NE_RCR_AM) != 0;  // RCR register
    }
    // Unicast to our MAC
    else if (memcmp(buffer, state.mac_addr, 6) == 0) {
        accept = true;
    }
    // Promiscuous mode
    else if (read_ne_register(0, 0x0C) & NE_RCR_PRO) {  // RCR register
        accept = true;
    }
    
    if (!accept) return;
    
    // Check buffer space
    uint8_t curr = read_ne_register(1, 0x07);  // CURR register
    uint8_t bnry = read_ne_register(0, 0x03);  // BNRY register
    uint8_t next = curr + 1 + ((len + NE_PAGE_SIZE - 1) / NE_PAGE_SIZE);
    
    if (next >= NE_RX_STOP) {
        next = NE_RX_START + (next - NE_RX_STOP);
    }
    
    if (next == bnry) {
        // Buffer full
        uint8_t isr = read_ne_register(0, 0x07);
        write_ne_register(0, 0x07, isr | NE_ISR_OVW);  // ISR register
        return;
    }
    
    // Store packet in receive buffer
    uint16_t offset = curr * NE_PAGE_SIZE;
    struct ne_packet_header header;
    header.status = NE_RSR_PRX;
    header.next_page = next;
    header.length = len + 4;  // Include header
    
    // Write header
    if (offset + 4 <= NE_MEM_SIZE) {
        write_ne_memory_block(offset, &header, 4);
        offset += 4;
    }
    
    // Write packet data
    if (offset + len <= NE_MEM_SIZE) {
        write_ne_memory_block(offset, buffer, len);
    }
    
    // Update current pointer
    write_ne_register(1, 0x07, next);  // CURR register
    
    // Set packet received interrupt
    uint8_t isr = read_ne_register(0, 0x07);
    write_ne_register(0, 0x07, isr | NE_ISR_PRX);  // ISR register
    
    // Copy to shared memory for debugging
    if (len <= 1500) {
        eth_write_shared_mem(ETH_RX_BUFFER, buffer, len);
        eth_write_shared_mem(ETH_PACKET_INFO + 2, &len, 2);
        uint32_t flags = eth_read_shared_u32(ETH_CTRL_FLAGS);
        eth_write_shared_u32(ETH_CTRL_FLAGS, flags | ETH_FLAG_RX_AVAIL);
    }
    
    eth_debug("ETH: Received packet of %zd bytes (total RX: %d)\n", len, state.rx_packets + 1);
    
    // Debug: Show first 32 bytes of received packet
    eth_debug("ETH: RX Data: ");
    for (int i = 0; i < 32 && i < len; i++) {
        eth_debug("%02X ", buffer[i]);
        if ((i + 1) % 16 == 0) eth_debug("\nETH: RX Data: ");
    }
    eth_debug("\n");
    
    // Show where packet was stored in NE2000 memory
    eth_debug("ETH: Packet stored at NE2000 offset 0x%04X, next page: 0x%02X\n", offset, next);
    
    state.rx_packets++;
    
    // Write updated state back to shared memory
    write_eth_state(&state);
}

// Structure to track register changes within a single poll loop
struct register_snapshot {
    uint8_t cr, isr, imr, dcr;
    uint8_t pstart, pstop, bnry, tpsr;
    uint8_t tbcr0, tbcr1, rsar0, rsar1, rbcr0, rbcr1;
    uint8_t rcr, tcr, rsr;
    uint8_t par[6];  // Physical address registers (MAC)
    uint8_t curr;    // Current page register
    uint32_t control_flags;
    uint16_t dma_addr, byte_count;
};

// Main polling function
void minimig_eth_poll()
{
    static int poll_count = 0;
    static struct register_snapshot prev_snapshot;
    static bool snapshot_initialized = false;
    static int total_changes_detected = 0;
    
    // Initialize snapshot on first run
    if (!snapshot_initialized) {
        memset(&prev_snapshot, 0, sizeof(prev_snapshot));
        snapshot_initialized = true;
    }

	if (!eth_shmem)
	{
		eth_shmem = (uint8_t *)shmem_map(ETH_SHMEM_ADDR, ETH_SHMEM_SIZE);
		if (!eth_shmem) {
			eth_debug("ETH: Failed to map shared memory at 0x%08X\n", ETH_SHMEM_ADDR);
			eth_shmem = (uint8_t *)-1;
		} else {
			eth_debug("ETH: Successfully mapped shared memory at 0x%08X\n", ETH_SHMEM_ADDR);
		}
	}
	
	if(eth_shmem != (uint8_t *)-1)
	{
        // Take snapshot of current registers at start of poll loop
        // Read from shared memory at ETH_CTRL_REGS (offset 0x0004 from ETH_SHMEM_ADDR)
        struct register_snapshot current_snapshot;
        current_snapshot.cr = eth_read_shared_reg(ETH_CTRL_REGS + (0x00 * 4));     // CR register
        current_snapshot.isr = eth_read_shared_reg(ETH_CTRL_REGS + (0x07 * 4));    // ISR register  
        current_snapshot.imr = eth_read_shared_reg(ETH_CTRL_REGS + (0x0F * 4));    // IMR register
        current_snapshot.dcr = eth_read_shared_reg(ETH_CTRL_REGS + (0x0E * 4));    // DCR register
        current_snapshot.pstart = eth_read_shared_reg(ETH_CTRL_REGS + (0x01 * 4)); // PSTART register
        current_snapshot.pstop = eth_read_shared_reg(ETH_CTRL_REGS + (0x02 * 4));  // PSTOP register
        current_snapshot.bnry = eth_read_shared_reg(ETH_CTRL_REGS + (0x03 * 4));   // BNRY register
        current_snapshot.tpsr = eth_read_shared_reg(ETH_CTRL_REGS + (0x04 * 4));   // TPSR register
        current_snapshot.tbcr0 = eth_read_shared_reg(ETH_CTRL_REGS + (0x05 * 4));  // TBCR0 register
        current_snapshot.tbcr1 = eth_read_shared_reg(ETH_CTRL_REGS + (0x06 * 4));  // TBCR1 register
        current_snapshot.rsar0 = eth_read_shared_reg(ETH_CTRL_REGS + (0x08 * 4));  // RSAR0 register
        current_snapshot.rsar1 = eth_read_shared_reg(ETH_CTRL_REGS + (0x09 * 4));  // RSAR1 register
        current_snapshot.rbcr0 = eth_read_shared_reg(ETH_CTRL_REGS + (0x0A * 4));  // RBCR0 register
        current_snapshot.rbcr1 = eth_read_shared_reg(ETH_CTRL_REGS + (0x0B * 4));  // RBCR1 register
        current_snapshot.rcr = eth_read_shared_reg(ETH_CTRL_REGS + (0x0C * 4));    // RCR register
        current_snapshot.tcr = eth_read_shared_reg(ETH_CTRL_REGS + (0x0D * 4));    // TCR register
        current_snapshot.rsr = eth_read_shared_reg(ETH_CTRL_REGS + (0x0C * 4));    // RSR (same offset as RCR)
        current_snapshot.curr = eth_read_shared_reg(ETH_CTRL_REGS + (0x17 * 4));   // CURR register (page 1, reg 7)
        
        // Read MAC address from shared memory (page 1, PAR0-PAR5 at offsets 0x11-0x16)
        for (int i = 0; i < 6; i++) {
            current_snapshot.par[i] = eth_read_shared_reg(ETH_CTRL_REGS + ((0x11 + i) * 4));
        }
        
        current_snapshot.control_flags = eth_read_shared_u32(ETH_CTRL_FLAGS);
        current_snapshot.dma_addr = current_snapshot.rsar0 | (current_snapshot.rsar1 << 8);
        current_snapshot.byte_count = current_snapshot.rbcr0 | (current_snapshot.rbcr1 << 8);


        // Get current state from shared memory
        struct rtl8019_state state;
        read_eth_state(&state);
        
        // Check for control flags from FPGA via shared memory
        uint32_t flags = eth_read_shared_u32(ETH_CTRL_FLAGS);
        
        // Update heartbeat counter every poll
        hps_heartbeat_counter++;
        eth_write_shared_u32(ETH_HPS_HEARTBEAT, hps_heartbeat_counter);
        
        // OPTIMIZATION: Reduced monitoring - only check packet info for activity
        static uint16_t last_packet_info = 0;
        
        // Check packet info for new activity (both TX/RX use this)
        uint16_t packet_info_check = 0;
        eth_read_shared_mem(ETH_PACKET_INFO, &packet_info_check, 2);
        if (packet_info_check != last_packet_info) {
            eth_debug("ETH: Packet activity detected! Length: %d\n", packet_info_check);
            last_packet_info = packet_info_check;
        }
        
        // Increment poll count (but no regular debug output)
        poll_count++;
        
        // FPGA handles all register management directly via shared memory

        // Handle transmit request
        if (flags & ETH_FLAG_TX_REQ) {
            transmit_packet();
            flags &= ~ETH_FLAG_TX_REQ;
            eth_write_shared_u32(ETH_CTRL_FLAGS, flags);
        }
        
        // Handle reset request
        if (flags & ETH_FLAG_RESET) {
            minimig_eth_reset();
            flags &= ~ETH_FLAG_RESET;
            eth_write_shared_u32(ETH_CTRL_FLAGS, flags);
        }
        
        // Check for incoming packets
        receive_packet();
        
        // FPGA handles interrupt management automatically
        
        // Update minimal status
        eth_write_shared_u32(ETH_CTRL_FLAGS, flags);
        
        // Write updated state back to shared memory
        write_eth_state(&state);
        
        // Take final snapshot and compare changes that occurred during this poll loop
        // Read from shared memory at ETH_CTRL_REGS (offset 0x0004 from ETH_SHMEM_ADDR)
        struct register_snapshot final_snapshot;
        final_snapshot.cr = eth_read_shared_reg(ETH_CTRL_REGS + (0x00 * 4));     // CR register
        final_snapshot.isr = eth_read_shared_reg(ETH_CTRL_REGS + (0x07 * 4));    // ISR register  
        final_snapshot.imr = eth_read_shared_reg(ETH_CTRL_REGS + (0x0F * 4));    // IMR register
        final_snapshot.dcr = eth_read_shared_reg(ETH_CTRL_REGS + (0x0E * 4));    // DCR register
        final_snapshot.pstart = eth_read_shared_reg(ETH_CTRL_REGS + (0x01 * 4)); // PSTART register
        final_snapshot.pstop = eth_read_shared_reg(ETH_CTRL_REGS + (0x02 * 4));  // PSTOP register
        final_snapshot.bnry = eth_read_shared_reg(ETH_CTRL_REGS + (0x03 * 4));   // BNRY register
        final_snapshot.tpsr = eth_read_shared_reg(ETH_CTRL_REGS + (0x04 * 4));   // TPSR register
        final_snapshot.tbcr0 = eth_read_shared_reg(ETH_CTRL_REGS + (0x05 * 4));  // TBCR0 register
        final_snapshot.tbcr1 = eth_read_shared_reg(ETH_CTRL_REGS + (0x06 * 4));  // TBCR1 register
        final_snapshot.rsar0 = eth_read_shared_reg(ETH_CTRL_REGS + (0x08 * 4));  // RSAR0 register
        final_snapshot.rsar1 = eth_read_shared_reg(ETH_CTRL_REGS + (0x09 * 4));  // RSAR1 register
        final_snapshot.rbcr0 = eth_read_shared_reg(ETH_CTRL_REGS + (0x0A * 4));  // RBCR0 register
        final_snapshot.rbcr1 = eth_read_shared_reg(ETH_CTRL_REGS + (0x0B * 4));  // RBCR1 register
        final_snapshot.rcr = eth_read_shared_reg(ETH_CTRL_REGS + (0x0C * 4));    // RCR register
        final_snapshot.tcr = eth_read_shared_reg(ETH_CTRL_REGS + (0x0D * 4));    // TCR register
        final_snapshot.rsr = eth_read_shared_reg(ETH_CTRL_REGS + (0x0C * 4));    // RSR (same offset as RCR)
        final_snapshot.curr = eth_read_shared_reg(ETH_CTRL_REGS + (0x17 * 4));   // CURR register (page 1, reg 7)
        
        // Read MAC address from shared memory (page 1, PAR0-PAR5 at offsets 0x11-0x16)
        for (int i = 0; i < 6; i++) {
            final_snapshot.par[i] = eth_read_shared_reg(ETH_CTRL_REGS + ((0x11 + i) * 4));
        }
        
        final_snapshot.control_flags = eth_read_shared_u32(ETH_CTRL_FLAGS);
        final_snapshot.dma_addr = final_snapshot.rsar0 | (final_snapshot.rsar1 << 8);
        final_snapshot.byte_count = final_snapshot.rbcr0 | (final_snapshot.rbcr1 << 8);
        
        // Display register changes that occurred since last poll
        // Compare current vs previous to detect any changes between polls
        bool any_changes = false;
        static char change_buffer[2048];
        change_buffer[0] = '\0';
        
        #define CHECK_REG_CHANGE(reg, name) \
            if (snapshot_initialized && current_snapshot.reg != prev_snapshot.reg) { \
                if (!any_changes) { \
                    strcat(change_buffer, "ETH: Register changes detected: "); \
                    any_changes = true; \
                } else { \
                    strcat(change_buffer, ", "); \
                } \
                char temp[64]; \
                snprintf(temp, sizeof(temp), "%s(0x%02X->0x%02X)", name, prev_snapshot.reg, current_snapshot.reg); \
                strcat(change_buffer, temp); \
            }
        
        CHECK_REG_CHANGE(cr, "CR");
        CHECK_REG_CHANGE(isr, "ISR");  
        CHECK_REG_CHANGE(imr, "IMR");
        CHECK_REG_CHANGE(dcr, "DCR");
        CHECK_REG_CHANGE(pstart, "PSTART");
        CHECK_REG_CHANGE(pstop, "PSTOP");
        CHECK_REG_CHANGE(bnry, "BNRY");
        CHECK_REG_CHANGE(tpsr, "TPSR");
        CHECK_REG_CHANGE(tbcr0, "TBCR0");
        CHECK_REG_CHANGE(tbcr1, "TBCR1");
        CHECK_REG_CHANGE(rsar0, "RSAR0");
        CHECK_REG_CHANGE(rsar1, "RSAR1");
        CHECK_REG_CHANGE(rbcr0, "RBCR0");
        CHECK_REG_CHANGE(rbcr1, "RBCR1");
        CHECK_REG_CHANGE(rcr, "RCR");
        CHECK_REG_CHANGE(tcr, "TCR");
        CHECK_REG_CHANGE(curr, "CURR");
        
        // Check DMA address and byte count changes
        if (snapshot_initialized && current_snapshot.dma_addr != prev_snapshot.dma_addr) {
            if (!any_changes) {
                strcat(change_buffer, "ETH: Register changes detected: ");
                any_changes = true;
            } else {
                strcat(change_buffer, ", ");
            }
            char temp[64];
            snprintf(temp, sizeof(temp), "DMA_ADDR(0x%04X->0x%04X)", prev_snapshot.dma_addr, current_snapshot.dma_addr);
            strcat(change_buffer, temp);
        }
        
        if (snapshot_initialized && current_snapshot.byte_count != prev_snapshot.byte_count) {
            if (!any_changes) {
                strcat(change_buffer, "ETH: Register changes detected: ");
                any_changes = true;
            } else {
                strcat(change_buffer, ", ");
            }
            char temp[64];
            snprintf(temp, sizeof(temp), "BYTE_COUNT(%d->%d)", prev_snapshot.byte_count, current_snapshot.byte_count);
            strcat(change_buffer, temp);
        }
        
        // Check MAC address changes
        if (snapshot_initialized) {
            bool mac_changed = false;
            for (int i = 0; i < 6; i++) {
                if (current_snapshot.par[i] != prev_snapshot.par[i]) {
                    mac_changed = true;
                    break;
                }
            }
            
            if (mac_changed) {
                if (!any_changes) {
                    strcat(change_buffer, "ETH: Register changes detected: ");
                    any_changes = true;
                } else {
                    strcat(change_buffer, ", ");
                }
                char temp[128];
                snprintf(temp, sizeof(temp), "MAC(%02X:%02X:%02X:%02X:%02X:%02X->%02X:%02X:%02X:%02X:%02X:%02X)",
                       prev_snapshot.par[0], prev_snapshot.par[1], prev_snapshot.par[2],
                       prev_snapshot.par[3], prev_snapshot.par[4], prev_snapshot.par[5],
                       current_snapshot.par[0], current_snapshot.par[1], current_snapshot.par[2],
                       current_snapshot.par[3], current_snapshot.par[4], current_snapshot.par[5]);
                strcat(change_buffer, temp);
            }
        }
        
        // Check control flags changes
        if (snapshot_initialized && current_snapshot.control_flags != prev_snapshot.control_flags) {
            if (!any_changes) {
                strcat(change_buffer, "ETH: Register changes detected: ");
                any_changes = true;
            } else {
                strcat(change_buffer, ", ");
            }
            char temp[64];
            snprintf(temp, sizeof(temp), "FLAGS(0x%08X->0x%08X)", prev_snapshot.control_flags, current_snapshot.control_flags);
            strcat(change_buffer, temp);
        }
        
        if (any_changes) {
            strcat(change_buffer, "\n");
            eth_debug("%s", change_buffer);
            total_changes_detected++;
            
            // Only show detailed comparison when there are actual changes
            eth_debug("ETH: Changed from previous poll - CR prev:0x%02X now:0x%02X, ISR prev:0x%02X now:0x%02X\n",
                     prev_snapshot.cr, current_snapshot.cr, prev_snapshot.isr, current_snapshot.isr);
        }
        
        // Debug: Show a few register reads to verify memory mapping is working
        static int debug_count = 0;
        if (++debug_count <= 10) {
            uint8_t test_cr = eth_read_shared_reg(ETH_CTRL_REGS + (0x00 * 4));
            eth_debug("ETH: Debug #%d - Reading CR from offset 0x%04X = 0x%02X (shmem=%p, physical=0x%08X)\n", 
                     debug_count, ETH_CTRL_REGS + (0x00 * 4), test_cr, eth_shmem, ETH_SHMEM_ADDR + ETH_CTRL_REGS);
        }
        
        // No regular debug output - only show changes when they occur
        
        // Store current snapshot for next iteration comparison
        prev_snapshot = current_snapshot;
    }
}

// Test function to verify ethernet is working and write address-decodable patterns
void minimig_eth_test()
{
    eth_debug("\n===============================================\n");
    eth_debug("ETH: Starting Ethernet Test with Address Patterns...\n");
    eth_debug("===============================================\n");
    
    // Initialize shared memory if needed
    if (!eth_shmem) {
        eth_shmem = (uint8_t *)shmem_map(ETH_SHMEM_ADDR, ETH_SHMEM_SIZE);
        if (!eth_shmem) eth_shmem = (uint8_t *)-1;
    }
    
    if (eth_shmem == (uint8_t *)-1) {
        eth_debug("  FAIL: Cannot map shared memory at 0x%08X\n", ETH_SHMEM_ADDR);
        return;
    }
    
    eth_debug("ETH TEST: Reading previous values and writing address-decodable patterns\n");
    eth_debug("  Base HPS Address: 0x%08X\n", ETH_SHMEM_ADDR);
    //eth_debug("  Shared Offset:    0x%04X\n", ETH_SHMEM_OFFSET);
    eth_debug("  Amiga Base:       0x%08X\n", ETH_BOARD_ADDR);
    
    // Read previous values but don't display yet
    // We'll only show them if they differ from the new test patterns
    
    // Region 1: Control Flags
    uint32_t prev_flags = eth_read_shared_u32(ETH_CTRL_FLAGS);
    uint32_t expected_flags = 0x01000000 | ETH_CTRL_FLAGS;
    
    // Region 2: Control Registers - Store first 8 and last 2 for comparison
    uint32_t prev_regs[18];
    for (int i = 0; i < 18; i++) {
        prev_regs[i] = eth_read_shared_u32(ETH_CTRL_REGS + i*4);
    }
    
    // Region 3: MAC Address
    uint8_t prev_mac[6];
    eth_read_shared_mem(ETH_CTRL_MAC, prev_mac, 6);
    uint8_t expected_mac[6] = {0x03, 0x10, 0x4C, 0xDE, 0xAD, 0xBE};
    
    // Region 4: Heartbeat and Signature
    uint32_t prev_heartbeat = eth_read_shared_u32(ETH_HPS_HEARTBEAT);
    uint32_t prev_signature = eth_read_shared_u32(ETH_HPS_SIGNATURE);
    uint32_t expected_heartbeat = 0x04000000 | ETH_HPS_HEARTBEAT;
    uint32_t expected_signature = 0xCAFEBABE;
    
    // Region 5-7: Buffer patterns
    uint32_t prev_tx = eth_read_shared_u32(ETH_TX_BUFFER);
    uint32_t prev_rx = eth_read_shared_u32(ETH_RX_BUFFER);
    uint32_t prev_ne = eth_read_shared_u32(ETH_NE_MEMORY);
    uint32_t expected_tx = 0x05000000 | ETH_TX_BUFFER;
    uint32_t expected_rx = 0x06000000 | ETH_RX_BUFFER;
    uint32_t expected_ne = 0x07000000 | ETH_NE_MEMORY;
    
    // Check if any values need to be shown (differ from expected)
    bool has_changes = false;
    has_changes |= (prev_flags != expected_flags);
    has_changes |= (prev_signature != expected_signature);
    has_changes |= (prev_tx != expected_tx);
    has_changes |= (prev_rx != expected_rx);
    has_changes |= (prev_ne != expected_ne);
    has_changes |= (memcmp(prev_mac, expected_mac, 6) != 0);
    
    // Check registers for changes
    for (int i = 0; i < 18; i++) {
        uint32_t expected_reg = 0x02000000 | (i << 16) | (ETH_CTRL_REGS + i*4);
        has_changes |= (prev_regs[i] != expected_reg);
    }
    
    if (has_changes) {
        eth_debug("\n--- PREVIOUS VALUES (showing only changed items) ---\n");
        
        // Show only changed values
        if (prev_flags != expected_flags) {
            eth_debug("  [0x%04X] Control Flags     = 0x%08X (was) -> 0x%08X (new)\n", 
                   ETH_CTRL_FLAGS, prev_flags, expected_flags);
        }
        
        // Show changed registers
        bool reg_header_shown = false;
        for (int i = 0; i < 18; i++) {
            uint32_t expected_reg = 0x02000000 | (i << 16) | (ETH_CTRL_REGS + i*4);
            if (prev_regs[i] != expected_reg) {
                if (!reg_header_shown) {
                    eth_debug("  Control Registers (changed):\n");
                    reg_header_shown = true;
                }
                eth_debug("    [0x%04X] Reg[%02d] = 0x%08X -> 0x%08X\n", 
                       ETH_CTRL_REGS + i*4, i, prev_regs[i], expected_reg);
            }
        }
        
        // Show MAC if changed
        if (memcmp(prev_mac, expected_mac, 6) != 0) {
            eth_debug("  [0x%04X] MAC Address       = %02X:%02X:%02X:%02X:%02X:%02X -> %02X:%02X:%02X:%02X:%02X:%02X\n",
                   ETH_CTRL_MAC, 
                   prev_mac[0], prev_mac[1], prev_mac[2], prev_mac[3], prev_mac[4], prev_mac[5],
                   expected_mac[0], expected_mac[1], expected_mac[2], expected_mac[3], expected_mac[4], expected_mac[5]);
        }
        
        // Show heartbeat/signature if changed
        if (prev_heartbeat != expected_heartbeat) {
            eth_debug("  [0x%04X] Heartbeat         = 0x%08X -> 0x%08X\n", 
                   ETH_HPS_HEARTBEAT, prev_heartbeat, expected_heartbeat);
        }
        if (prev_signature != expected_signature) {
            eth_debug("  [0x%04X] Signature         = 0x%08X -> 0x%08X\n", 
                   ETH_HPS_SIGNATURE, prev_signature, expected_signature);
        }
        
        // Show buffers if changed
        if (prev_tx != expected_tx) {
            eth_debug("  [0x%04X] TX Buffer         = 0x%08X -> 0x%08X\n", 
                   ETH_TX_BUFFER, prev_tx, expected_tx);
        }
        if (prev_rx != expected_rx) {
            eth_debug("  [0x%04X] RX Buffer         = 0x%08X -> 0x%08X\n", 
                   ETH_RX_BUFFER, prev_rx, expected_rx);
        }
        if (prev_ne != expected_ne) {
            eth_debug("  [0x%04X] NE2000 Memory     = 0x%08X -> 0x%08X\n", 
                   ETH_NE_MEMORY, prev_ne, expected_ne);
        }
    } else {
        eth_debug("\n--- NO CHANGES DETECTED (test patterns already present) ---\n");
    }
    
    eth_debug("\n--- WRITING NEW TEST PATTERNS ---\n");
    
    // Write recognizable patterns to each memory region
    // Pattern: 0xAABBCCDD where AA=region_id, BB=subregion, CC/DD=offset_encoded
    
    // Region 1: Control Flags (0x1000)
    uint32_t pattern = 0x01000000 | ETH_CTRL_FLAGS;  // Region 1, offset encoded
    eth_write_shared_u32(ETH_CTRL_FLAGS, pattern);
    eth_debug("  [0x%04X] Control Flags     = 0x%08X (new) (Amiga: 0x%06X)\n", 
           ETH_CTRL_FLAGS, pattern, ETH_BOARD_ADDR +ETH_CTRL_FLAGS);
    
    // Region 2: Control Registers (0x1004-0x104B) - Now 18 registers (72 bytes)
    eth_debug("  Writing 18 control registers (72 bytes):\n");
    for (int i = 0; i < 18; i++) {
        pattern = 0x02000000 | (i << 16) | (ETH_CTRL_REGS + i*4);
        eth_write_shared_u32(ETH_CTRL_REGS + i*4, pattern);
        if (i < 8 || i >= 16) {  // Show first 8 and last 2 (extended registers)
            eth_debug("    [0x%04X] Reg[%02d] = 0x%08X (new)\n", 
                   ETH_CTRL_REGS + i*4, i, pattern);
        } else if (i == 8) {
            eth_debug("    ... (regs 8-15) ...\n");
        }
    }
    eth_debug("  [0x%04X] Control Regs[0-17] = 72 bytes total (Amiga: 0x%06X)\n", 
           ETH_CTRL_REGS, ETH_BOARD_ADDR +ETH_CTRL_REGS);
    
    // Region 3: MAC Address (0x1024)
    uint8_t test_mac[6] = {0x03, 0x10, 0x4C, 0xDE, 0xAD, 0xBE};
    eth_write_shared_mem(ETH_CTRL_MAC, test_mac, 6);
    eth_debug("  [0x%04X] MAC Address       = 03:10:4C:DE:AD:BE (Amiga: 0x%06X)\n",
           ETH_CTRL_MAC, ETH_BOARD_ADDR +ETH_CTRL_MAC);
    
    // Region 4: Heartbeat and Signature
    pattern = 0x04000000 | ETH_HPS_HEARTBEAT;
    eth_write_shared_u32(ETH_HPS_HEARTBEAT, pattern);
    eth_write_shared_u32(ETH_HPS_SIGNATURE, 0xCAFEBABE);
    eth_debug("  [0x%04X] Heartbeat         = 0x%08X (Amiga: 0x%06X)\n",
           ETH_HPS_HEARTBEAT, pattern, ETH_BOARD_ADDR +ETH_HPS_HEARTBEAT);
    eth_debug("  [0x%04X] Signature         = 0xCAFEBABE (Amiga: 0x%06X)\n",
           ETH_HPS_SIGNATURE, ETH_BOARD_ADDR +ETH_HPS_SIGNATURE);
    
    // Region 5: TX Buffer (0x2000) - Write address pattern every 256 bytes
    eth_debug("  [0x%04X] TX Buffer patterns...\n", ETH_TX_BUFFER);
    for (int i = 0; i < 1500; i += 256) {
        pattern = 0x05000000 | (ETH_TX_BUFFER + i);
        eth_write_shared_u32(ETH_TX_BUFFER + i, pattern);
        if (i < 512) {  // Only show first few
            eth_debug("    [+0x%04X] = 0x%08X (Amiga: 0x%06X)\n", 
                   i, pattern, ETH_BOARD_ADDR +ETH_TX_BUFFER + i);
        }
    }
    
    // Region 6: RX Buffer (0x2600) - Write address pattern every 256 bytes
    eth_debug("  [0x%04X] RX Buffer patterns...\n", ETH_RX_BUFFER);
    for (int i = 0; i < 1500; i += 256) {
        pattern = 0x06000000 | (ETH_RX_BUFFER + i);
        eth_write_shared_u32(ETH_RX_BUFFER + i, pattern);
        if (i < 512) {  // Only show first few
            eth_debug("    [+0x%04X] = 0x%08X (Amiga: 0x%06X)\n", 
                   i, pattern, ETH_BOARD_ADDR +ETH_RX_BUFFER + i);
        }
    }
    
    // Region 7: NE2000 Memory (0x3000) - Write pattern every 1KB
    eth_debug("  [0x%04X] NE2000 Memory patterns...\n", ETH_NE_MEMORY);
    for (int i = 0; i < 16384; i += 1024) {
        pattern = 0x07000000 | ((ETH_NE_MEMORY + i) & 0xFFFF);
        eth_write_shared_u32(ETH_NE_MEMORY + i, pattern);
        if (i < 4096) {  // Only show first 4KB
            eth_debug("    [+0x%04X] = 0x%08X (Amiga: 0x%06X)\n", 
                   i, pattern, ETH_BOARD_ADDR +ETH_NE_MEMORY + i);
        }
    }
    
    // Region 8: Debug Info (0x7000)
    pattern = 0x08000000 | ETH_DEBUG_INFO;
    eth_write_shared_u32(ETH_DEBUG_INFO, pattern);
    eth_debug("  [0x%04X] Debug Info        = 0x%08X (Amiga: 0x%06X)\n",
           ETH_DEBUG_INFO, pattern, ETH_BOARD_ADDR +ETH_DEBUG_INFO);
    
    // Region 9: Future Use (0x9000)
    pattern = 0x09000000 | ETH_FUTURE_USE;
    eth_write_shared_u32(ETH_FUTURE_USE, pattern);
    eth_debug("  [0x%04X] Future Use        = 0x%08X (Amiga: 0x%06X)\n",
           ETH_FUTURE_USE, pattern, ETH_BOARD_ADDR +ETH_FUTURE_USE);
    
    eth_debug("\n--- VERIFICATION ---\n");
    
    // Verify patterns were written correctly
    bool all_pass = true;
    
    // Verify Control Flags
    uint32_t readback = eth_read_shared_u32(ETH_CTRL_FLAGS);
    if (readback != expected_flags) {
        eth_debug("  Control Flags: FAIL (expected 0x%08X, got 0x%08X)\n", expected_flags, readback);
        all_pass = false;
    }
    
    // Verify Registers
    bool reg_fail = false;
    for (int i = 0; i < 18; i++) {
        uint32_t reg_val = eth_read_shared_u32(ETH_CTRL_REGS + i*4);
        uint32_t expected = 0x02000000 | (i << 16) | (ETH_CTRL_REGS + i*4);
        if (reg_val != expected) {
            if (!reg_fail) {
                eth_debug("  Register verification failures:\n");
                reg_fail = true;
            }
            eth_debug("    Reg[%02d]: FAIL (expected 0x%08X, got 0x%08X)\n", i, expected, reg_val);
            all_pass = false;
        }
    }
    
    // Verify MAC address
    uint8_t new_mac[6];
    eth_read_shared_mem(ETH_CTRL_MAC, new_mac, 6);
    if (memcmp(new_mac, expected_mac, 6) != 0) {
        eth_debug("  MAC Address: FAIL (expected %02X:%02X:%02X:%02X:%02X:%02X, got %02X:%02X:%02X:%02X:%02X:%02X)\n",
               expected_mac[0], expected_mac[1], expected_mac[2], expected_mac[3], expected_mac[4], expected_mac[5],
               new_mac[0], new_mac[1], new_mac[2], new_mac[3], new_mac[4], new_mac[5]);
        all_pass = false;
    }
    
    // Verify Signature
    readback = eth_read_shared_u32(ETH_HPS_SIGNATURE);
    if (readback != expected_signature) {
        eth_debug("  Signature: FAIL (expected 0x%08X, got 0x%08X)\n", expected_signature, readback);
        all_pass = false;
    }
    
    // Verify Buffers
    uint32_t new_tx = eth_read_shared_u32(ETH_TX_BUFFER);
    uint32_t new_rx = eth_read_shared_u32(ETH_RX_BUFFER);
    uint32_t new_ne = eth_read_shared_u32(ETH_NE_MEMORY);
    
    if (new_tx != expected_tx) {
        eth_debug("  TX Buffer: FAIL (expected 0x%08X, got 0x%08X)\n", expected_tx, new_tx);
        all_pass = false;
    }
    if (new_rx != expected_rx) {
        eth_debug("  RX Buffer: FAIL (expected 0x%08X, got 0x%08X)\n", expected_rx, new_rx);
        all_pass = false;
    }
    if (new_ne != expected_ne) {
        eth_debug("  NE Memory: FAIL (expected 0x%08X, got 0x%08X)\n", expected_ne, new_ne);
        all_pass = false;
    }
    
    if (all_pass) {
        eth_debug("  All memory regions verified successfully - PASS\n");
    } else {
        eth_debug("  Some memory regions failed verification - see above\n");
    }
    
    // Test 2: Check register initialization - read from shared memory
    eth_debug("\nETH TEST 2: Register check\n");
    uint8_t cr_reg = eth_read_shared_reg(ETH_CTRL_REGS + (0x00 * 4));
    uint8_t isr_reg = eth_read_shared_reg(ETH_CTRL_REGS + (0x07 * 4));
    eth_debug("  CR  (0x00) = 0x%02X (pattern: 0x%08X)\n", cr_reg, 0x02000000 | ETH_CTRL_REGS);
    eth_debug("  ISR (0x07) = 0x%02X (pattern: 0x%08X)\n", isr_reg, 0x02070000 | (ETH_CTRL_REGS + 0x1C));
    
    // Read MAC address from shared memory
    uint8_t shared_mac[6];
    eth_read_shared_mem(ETH_CTRL_MAC, shared_mac, 6);
    eth_debug("  MAC = %02X:%02X:%02X:%02X:%02X:%02X\n",
           shared_mac[0], shared_mac[1], shared_mac[2],
           shared_mac[3], shared_mac[4], shared_mac[5]);
    
    // Test 3: Check control flags
    eth_debug("\nETH TEST 3: Control flags\n");
    uint32_t flags = eth_read_shared_u32(ETH_CTRL_FLAGS);
    eth_debug("  Flags = 0x%08X\n", flags);
    eth_debug("  - Reset:    %s\n", (flags & ETH_FLAG_RESET) ? "YES" : "NO");
    eth_debug("  - TX Req:   %s\n", (flags & ETH_FLAG_TX_REQ) ? "YES" : "NO");
    eth_debug("  - RX Avail: %s\n", (flags & ETH_FLAG_RX_AVAIL) ? "YES" : "NO");
    eth_debug("  - IRQ:      %s\n", (flags & ETH_FLAG_IRQ) ? "YES" : "NO");
    eth_debug("  - Enabled:  %s\n", (flags & ETH_FLAG_ENABLED) ? "YES" : "NO");
    
    // Test 4: Socket status
    eth_debug("ETH TEST 4: Network socket\n");
    if (raw_socket >= 0) {
        eth_debug("  PASS: Raw socket is open (fd=%d)\n", raw_socket);
        eth_debug("  Interface: %s\n", bridge_interface);
    } else {
        eth_debug("  FAIL: No raw socket available\n");
    }
    
    eth_debug("===============================================\n");
    eth_debug("ETH: Test Complete\n");
    eth_debug("===============================================\n\n");
}