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
    if (!eth_shmem || eth_shmem == (uint8_t *)-1) return;
    
    if (offset + size > ETH_SHMEM_SIZE) {
        printf("ETH: Write beyond shared memory bounds: offset=0x%X, size=%d\n", offset, size);
        return;
    }
    
    memcpy(eth_shmem + offset, data, size);
}

static void eth_read_shared_mem(uint32_t offset, void *data, uint32_t size)
{
    if (!eth_shmem || eth_shmem == (uint8_t *)-1) {
        memset(data, 0, size);
        return;
    }
    
    if (offset + size > ETH_SHMEM_SIZE) {
        printf("ETH: Read beyond shared memory bounds: offset=0x%X, size=%d\n", offset, size);
        memset(data, 0, size);
        return;
    }
    
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

// Helper functions to access RTL8019 state structure in shared memory
// static struct rtl8019_state* get_eth_state() {
//     if (!eth_shmem || eth_shmem == (uint8_t *)-1) return NULL;
//     return (struct rtl8019_state*)(eth_shmem + ETH_SHM_OFFSET + ETH_RTL8019_STATE);
// }

static void write_eth_state(const struct rtl8019_state* state) {
    if (!eth_shmem || eth_shmem == (uint8_t *)-1) return;
    eth_write_shared_mem(ETH_SHM_OFFSET + ETH_RTL8019_STATE, state, sizeof(struct rtl8019_state));
}

static void read_eth_state(struct rtl8019_state* state) {
    if (!eth_shmem || eth_shmem == (uint8_t *)-1) {
        memset(state, 0, sizeof(struct rtl8019_state));
        return;
    }
    eth_read_shared_mem(ETH_SHM_OFFSET + ETH_RTL8019_STATE, state, sizeof(struct rtl8019_state));
}

// Initialize ethernet emulation
void minimig_eth_init()
{
    // Map the ethernet shared memory region
    printf("ETH: Initializing ethernet emulation...\n");
    eth_shmem = (uint8_t *)shmem_map(ETH_SHMEM_ADDR, ETH_SHMEM_SIZE);
    if (!eth_shmem) {
        printf("Failed to map ethernet shared memory!\n");
        return;
    }

    eth_debug("Initializing RTL8019 ethernet emulation\n");
    
    // Initialize registers to reset state (this will initialize the shared memory state)
    minimig_eth_reset();
    
    // Initialize shared memory with proper values
    printf("ETH: Initializing shared memory...\n");
    
    // Clear all shared memory
    memset(eth_shmem, 0, ETH_SHMEM_SIZE);
    
    // Write HPS signature
    eth_write_shared_u32(ETH_SHM_OFFSET + ETH_HPS_SIGNATURE, 0xCAFEBABE);
    
    // Write initial MAC address
    eth_write_shared_mem(ETH_SHM_OFFSET + ETH_CTRL_MAC, default_mac, 6);
    
    // Register state is already written by minimig_eth_reset() above
    
    // Initialize control flags
    eth_write_shared_u32(ETH_SHM_OFFSET + ETH_CTRL_FLAGS, ETH_FLAG_ENABLED);
    
    // Initialize packet buffers with pattern for testing (LSB format)
    uint8_t test_pattern[16] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11,
                                0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99};
    eth_write_shared_mem(ETH_SHM_OFFSET + ETH_RX_BUFFER, test_pattern, 16);
    
    printf("ETH: Shared memory initialized with signature 0x%08X\n", 0xCAFEBABE);
    
    // Try to open raw socket for ethernet bridging
    printf("ETH: Opening raw socket for bridging...\n");
    raw_socket = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (raw_socket < 0) {
        printf("ETH: Warning: Could not open raw socket for ethernet bridging: %s\n", strerror(errno));
        return;
    }
    printf("ETH: Raw socket opened successfully\n");
    
    // Find and bind to ethernet interface
    struct ifaddrs *ifaddr, *ifa;
    if (getifaddrs(&ifaddr) == -1) {
        printf("Failed to get interface addresses\n");
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
        printf("ETH: Warning: No ethernet interface found for bridging\n");
        close(raw_socket);
        raw_socket = -1;
        return;
    }
    printf("ETH: Found ethernet interface: %s\n", bridge_interface);
    
    // Get interface index
    struct ifreq ifr;
    strncpy(ifr.ifr_name, bridge_interface, IFNAMSIZ-1);
    if (ioctl(raw_socket, SIOCGIFINDEX, &ifr) < 0) {
        printf("Failed to get interface index: %s\n", strerror(errno));
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
        printf("ETH: Failed to bind raw socket: %s\n", strerror(errno));
        close(raw_socket);
        raw_socket = -1;
        return;
    }
    printf("ETH: Socket bound to interface successfully\n");
    
    // Print summary of ethernet initialization
    printf("===============================================\n");
    printf("ETH: X-Surf 100 Ethernet Initialized!\n");
    printf("ETH: Amiga Address: 0x%06X\n", ETH_BASE_ADDR);
    printf("ETH: HPS Memory:    0x%08X\n", ETH_SHMEM_ADDR);
    printf("ETH: MAC Address:   %02X:%02X:%02X:%02X:%02X:%02X\n",
           default_mac[0], default_mac[1], default_mac[2],
           default_mac[3], default_mac[4], default_mac[5]);
    printf("ETH: Interface:     %s\n", bridge_interface);
    printf("===============================================\n");
    
    // Set socket to non-blocking
    int flags = fcntl(raw_socket, F_GETFL, 0);
    fcntl(raw_socket, F_SETFL, flags | O_NONBLOCK);
    
    eth_debug("RTL8019 ethernet emulation initialized on interface %s\n", bridge_interface);
}

// Reset RTL8019 to initial state
void minimig_eth_reset()
{
    printf("ETH: RTL8019 RESET triggered!\n");
    
    // Initialize RTL8019 state structure in shared memory
    struct rtl8019_state state;
    memset(&state, 0, sizeof(state));
    
    // Initialize page 0 registers (use logical register numbers, not physical offsets)
    state.regs[0][0x00] = NE_CR_STP | NE_CR_RDMA_NOT;  // CR: Stop, no remote DMA
    state.regs[0][0x07] = NE_ISR_RST;                  // ISR: Reset complete
    state.regs[0][0x0E] = 0x48;                        // DCR: Normal operation, FIFO threshold
    state.regs[0][0x0D] = 0x00;                        // TCR: Normal operation
    state.regs[0][0x0C] = 0x00;                        // RCR: Reject all packets initially
    
    // Set up buffer boundaries
    state.regs[0][0x01] = NE_RX_START;                 // PSTART
    state.regs[0][0x02] = NE_RX_STOP;                  // PSTOP
    state.regs[0][0x03] = NE_RX_START;                 // BNRY
    state.regs[0][0x04] = NE_TX_START;                 // TPSR
    
    // Initialize MAC address
    memcpy(state.mac_addr, default_mac, 6);
    
    // Initialize page 1 registers (MAC address)
    for (int i = 0; i < 6; i++) {
        state.regs[1][0x01 + i] = state.mac_addr[i];  // PAR0-PAR5
    }
    state.regs[1][0x07] = NE_RX_START + 1;             // CURR
    
    // Reset state variables
    state.current_page = 0;
    state.remote_dma_addr = 0;
    state.remote_dma_count = 0;
    state.boundary_ptr = NE_RX_START;
    state.current_ptr = NE_RX_START + 1;
    state.enabled = false;
    state.link_up = false;
    state.tx_packets = 0;
    state.rx_packets = 0;
    state.tx_errors = 0;
    state.rx_errors = 0;
    
    // Write the complete state to shared memory
    write_eth_state(&state);
    
    // Update shared memory with reset state
    eth_write_shared_u32(ETH_SHM_OFFSET + ETH_CTRL_FLAGS, ETH_FLAG_RESET);
    
    // Write registers to shared memory (32-bit aligned, LSB format)
    for (int i = 0; i < 16; i++) {
        eth_write_shared_reg(ETH_SHM_OFFSET + ETH_CTRL_REGS + (i * 4), state.regs[0][i]);
    }
    eth_write_shared_mem(ETH_SHM_OFFSET + ETH_CTRL_MAC, state.mac_addr, 6);
    
    // Write HPS signature to show we're alive
    eth_write_shared_u32(ETH_SHM_OFFSET + ETH_HPS_SIGNATURE, 0xCAFEBABE);
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
        printf("ETH: Amiga READ  reg[0x%02X] = 0x%02X (count: %d)\n", reg, value, read_count);
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
    
    // Regular register read - get state from shared memory
    struct rtl8019_state state;
    read_eth_state(&state);
    uint8_t page = (state.regs[0][0x00] & (NE_CR_PS0 | NE_CR_PS1)) >> 6;  // Get page from CR register
    
    if (reg < 16) {
        value = state.regs[page][reg];
        
        // Special handling for certain registers
        switch (reg) {
            case NE_P0_ISR:
                // Reading ISR doesn't clear bits automatically
                break;
                
            case NE_P0_TSR:
                if (page == 0) {
                    // Return transmit status
                    value = NE_TSR_PTX;  // Always indicate successful transmission
                }
                break;
                
            case NE_P0_RSR:
                if (page == 0) {
                    // Return receive status for last packet
                    value = NE_RSR_PRX;  // Packet received intact
                }
                break;
                
            default:
                break;
        }
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
        printf("ETH: Amiga WRITE reg[0x%02X] = 0x%02X (count: %d)\n", reg, data, write_count);
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
    
    // Get current state from shared memory
    struct rtl8019_state state;
    read_eth_state(&state);
    uint8_t page = (state.regs[0][0x00] & (NE_CR_PS0 | NE_CR_PS1)) >> 6;  // Get page from CR register
    
    eth_debug("RTL8019 write reg[%d][0x%02X] = 0x%02X\n", page, reg, data);
    
    if (reg < 16) {
        // Special handling for command register
        if (reg == 0x00) {  // Command register
            uint8_t old_cr = state.regs[0][0x00];
            state.regs[0][0x00] = data;
            
            // Handle start/stop
            if ((data & NE_CR_STA) && !(old_cr & NE_CR_STA)) {
                state.enabled = true;
                eth_debug("RTL8019 started\n");
            } else if (!(data & NE_CR_STA) && (old_cr & NE_CR_STA)) {
                state.enabled = false;
                eth_debug("RTL8019 stopped\n");
            }
            
            // Handle transmit packet command
            if (data & NE_CR_TXP) {
                eth_debug("RTL8019 transmit packet requested\n");
                // Trigger packet transmission via shared memory
                uint32_t flags = eth_read_shared_u32(ETH_SHM_OFFSET + ETH_CTRL_FLAGS);
                eth_write_shared_u32(ETH_SHM_OFFSET + ETH_CTRL_FLAGS, flags | ETH_FLAG_TX_REQ);
                // Clear TXP bit (auto-cleared after transmission)
                state.regs[0][NE_P0_CR] &= ~NE_CR_TXP;
            }
            
            // Handle remote DMA commands
            uint8_t rdma = data & (NE_CR_RD0 | NE_CR_RD1 | NE_CR_RD2);
            if (rdma == NE_CR_RDMA_READ || rdma == NE_CR_RDMA_WRITE) {
                // Set up remote DMA
                state.remote_dma_addr = 
                    state.regs[0][0x08] |   // RSAR0
                    (state.regs[0][0x09] << 8);  // RSAR1
                state.remote_dma_count = 
                    state.regs[0][0x0A] |   // RBCR0
                    (state.regs[0][0x0B] << 8);  // RBCR1
                    
                const char* dma_type = (rdma == NE_CR_RDMA_READ) ? "READ" : "WRITE";
                printf("ETH: FPGA Remote DMA %s setup - addr:0x%04X count:%d DCR:0x%02X\n", 
                       dma_type, state.remote_dma_addr, state.remote_dma_count,
                       state.regs[0][0x0E]);
            } else if (rdma == NE_CR_RDMA_SEND) {
                printf("ETH: FPGA Remote DMA SEND packet command\n");
            }
        }
        // Handle ISR register writes (clear interrupts)
        else if (reg == 0x07 && page == 0) {  // ISR register
            // Clear bits that are set in the write data
            state.regs[0][0x07] &= ~data;
        }
        // Handle other registers
        else {
            state.regs[page][reg] = data;
            
            // Debug important DMA setup registers
            if (page == 0) {
                switch (reg) {
                    case 0x08:  // RSAR0
                        printf("ETH: FPGA wrote RSAR0 = 0x%02X (DMA addr low)\n", data);
                        break;
                    case 0x09:  // RSAR1  
                        printf("ETH: FPGA wrote RSAR1 = 0x%02X (DMA addr high)\n", data);
                        break;
                    case 0x0A:  // RBCR0
                        printf("ETH: FPGA wrote RBCR0 = 0x%02X (DMA count low)\n", data);
                        break;
                    case 0x0B:  // RBCR1
                        printf("ETH: FPGA wrote RBCR1 = 0x%02X (DMA count high)\n", data);
                        break;
                    case 0x0E:  // DCR
                        printf("ETH: FPGA wrote DCR = 0x%02X (data config: %s-bit mode)\n", 
                               data, (data & NE_DCR_WTS) ? "16" : "8");
                        break;
                }
            }
            
            // Update MAC address in page 1
            if (page == 1 && reg >= 0x01 && reg <= 0x06) {  // PAR0-PAR5
                state.mac_addr[reg - 0x01] = data;
                printf("ETH: FPGA updated MAC[%d] = 0x%02X -> %02X:%02X:%02X:%02X:%02X:%02X\n",
                       reg - 0x01, data,
                       state.mac_addr[0], state.mac_addr[1], state.mac_addr[2],
                       state.mac_addr[3], state.mac_addr[4], state.mac_addr[5]);
            }
        }
    }
    
    // Write updated state back to shared memory
    write_eth_state(&state);
    
    // Update shared memory with current register state (32-bit aligned, LSB format)
    eth_write_shared_reg(ETH_SHM_OFFSET + ETH_CTRL_REGS + (reg * 4), state.regs[page][reg]);
    if (reg >= 0x01 && reg <= 0x06 && page == 1) {  // PAR0-PAR5
        eth_write_shared_mem(ETH_SHM_OFFSET + ETH_CTRL_MAC, state.mac_addr, 6);
    }
}

// Read data from RTL8019 data port
uint16_t minimig_eth_read_data()
{
    static int data_read_count = 0;
    
    // Get current state from shared memory
    struct rtl8019_state state;
    read_eth_state(&state);
    
    // Enhanced debug for FPGA data port access
    printf("ETH: >>> Data port READ request (call #%d)\n", data_read_count + 1);
    printf("ETH: Current remote_dma_addr=0x%04X, remote_dma_count=%d\n", 
           state.remote_dma_addr, state.remote_dma_count);
    printf("ETH: DCR=0x%02X (%s-bit mode), CR=0x%02X (page %d)\n",
           state.regs[0][0x0E], (state.regs[0][0x0E] & NE_DCR_WTS) ? "16" : "8",
           state.regs[0][0x00], (state.regs[0][0x00] >> 6) & 3);
    
    if (state.remote_dma_count == 0) {
        printf("ETH: Data port read with zero DMA count!\n");
        return 0;
    }
    
    uint16_t addr = state.remote_dma_addr;
    uint16_t data = 0;
    
    // Read from NE2000 memory
    if (addr < NE_MEM_SIZE) {
        data = state.memory[addr];
        if (state.regs[0][0x0E] & NE_DCR_WTS) {  // DCR register
            // 16-bit mode
            if (addr + 1 < NE_MEM_SIZE) {
                data |= (state.memory[addr + 1] << 8);
            }
            state.remote_dma_addr += 2;
            if (state.remote_dma_count >= 2) {
                state.remote_dma_count -= 2;
            } else {
                state.remote_dma_count = 0;
            }
            
            // Debug output for 16-bit reads
            if (++data_read_count <= 20 || (data_read_count % 100) == 0) {
                printf("ETH: FPGA Data READ [0x%04X] = 0x%04X (16-bit, count:%d, remaining:%d)\n", 
                       addr, data, data_read_count, state.remote_dma_count);
                
                // Check if FPGA might be writing in LSB format to shared memory
                if (data == 0) {
                    // Data is zero - might indicate communication issue
                }
            }
        } else {
            // 8-bit mode
            state.remote_dma_addr += 1;
            state.remote_dma_count -= 1;
            
            // Debug output for 8-bit reads
            if (data_read_count <= 20 || (data_read_count % 100) == 0) {
                printf("ETH: FPGA Data read [0x%04X] = 0x%02X (8-bit, count:%d, remaining:%d)\n", 
                       addr, data & 0xFF, ++data_read_count, state.remote_dma_count);
            }
        }
    } else {
        printf("ETH: Data port read beyond memory bounds: addr=0x%04X\n", addr);
    }
    
    // Check if remote DMA is complete
    if (state.remote_dma_count == 0) {
        state.regs[0][0x07] |= NE_ISR_RDC;  // ISR register
        printf("ETH: Remote DMA READ complete\n");
    }
    
    // Write updated state back to shared memory
    write_eth_state(&state);
    
    return data;
}

// Write data to RTL8019 data port
void minimig_eth_write_data(uint16_t data)
{
    static int data_write_count = 0;
    
    // Get current state from shared memory
    struct rtl8019_state state;
    read_eth_state(&state);
    
    // Enhanced debug for FPGA data port access
    printf("ETH: >>> Data port WRITE request (call #%d), data=0x%04X\n", data_write_count + 1, data);
    printf("ETH: Current remote_dma_addr=0x%04X, remote_dma_count=%d\n", 
           state.remote_dma_addr, state.remote_dma_count);
    printf("ETH: DCR=0x%02X (%s-bit mode), CR=0x%02X (page %d)\n",
           state.regs[0][0x0E], (state.regs[0][0x0E] & NE_DCR_WTS) ? "16" : "8",
           state.regs[0][0x00], (state.regs[0][0x00] >> 6) & 3);
    
    if (state.remote_dma_count == 0) {
        printf("ETH: Data port write with zero DMA count! Data=0x%04X\n", data);
        return;
    }
    
    uint16_t addr = state.remote_dma_addr;
    
    // Write to NE2000 memory
    if (addr < NE_MEM_SIZE) {
        state.memory[addr] = data & 0xFF;
        if (state.regs[0][0x0E] & NE_DCR_WTS) {  // DCR register
            // 16-bit mode
            if (addr + 1 < NE_MEM_SIZE) {
                state.memory[addr + 1] = (data >> 8) & 0xFF;
            }
            state.remote_dma_addr += 2;
            if (state.remote_dma_count >= 2) {
                state.remote_dma_count -= 2;
            } else {
                state.remote_dma_count = 0;
            }
            
            // Debug output for 16-bit writes
            if (++data_write_count <= 20 || (data_write_count % 100) == 0) {
                printf("ETH: FPGA Data WRITE [0x%04X] = 0x%04X (16-bit, count:%d, remaining:%d)\n", 
                       addr, data, data_write_count, state.remote_dma_count);
            }
        } else {
            // 8-bit mode
            state.remote_dma_addr += 1;
            state.remote_dma_count -= 1;
            
            // Debug output for 8-bit writes
            if (data_write_count <= 20 || (data_write_count % 100) == 0) {
                printf("ETH: FPGA Data write [0x%04X] = 0x%02X (8-bit, count:%d, remaining:%d)\n", 
                       addr, data & 0xFF, ++data_write_count, state.remote_dma_count);
            }
        }
        
        // Hex dump for packet data writes (show first few bytes)
        if (data_write_count <= 50 && addr >= 0x2000) {  // TX buffer area
            printf("ETH: TX Buffer [0x%04X]: %02X %02X\n", 
                   addr, state.memory[addr], 
                   (addr + 1 < NE_MEM_SIZE) ? state.memory[addr + 1] : 0);
        }
    } else {
        printf("ETH: Data port write beyond memory bounds: addr=0x%04X, data=0x%04X\n", addr, data);
    }
    
    // Check if remote DMA is complete
    if (state.remote_dma_count == 0) {
        state.regs[0][0x07] |= NE_ISR_RDC;  // ISR register
        printf("ETH: Remote DMA WRITE complete\n");
    }
    
    // Write updated state back to shared memory
    write_eth_state(&state);
}

// Transmit packet to host ethernet
void transmit_packet()
{
    // Get current state from shared memory
    struct rtl8019_state state;
    read_eth_state(&state);
    if (raw_socket < 0) return;
    
    uint8_t page = state.regs[0][0x04];  // TPSR register
    uint16_t length = state.regs[0][0x05] |     // TBCR0
                     (state.regs[0][0x06] << 8);   // TBCR1
    
    if (length == 0 || length > 1500) {
        eth_debug("Invalid packet length: %d\n", length);
        return;
    }
    
    uint16_t offset = page * NE_PAGE_SIZE;
    if (offset + length > NE_MEM_SIZE) {
        eth_debug("Packet exceeds memory bounds\n");
        return;
    }
    
    // Copy packet to shared memory for debugging
    eth_write_shared_mem(ETH_SHM_OFFSET + ETH_TX_BUFFER, &state.memory[offset], length);
    eth_write_shared_mem(ETH_SHM_OFFSET + ETH_PACKET_INFO, &length, 2);
    
    // Debug: Show first 32 bytes of transmitted packet
    printf("ETH: Transmitting packet from offset 0x%04X, length %d bytes:\n", offset, length);
    printf("ETH: TX Data: ");
    for (int i = 0; i < 32 && i < length; i++) {
        printf("%02X ", state.memory[offset + i]);
        if ((i + 1) % 16 == 0) printf("\nETH: TX Data: ");
    }
    printf("\n");
    
    // Send packet via raw socket
    if (send(raw_socket, &state.memory[offset], length, 0) < 0) {
        printf("ETH: Failed to send packet: %s\n", strerror(errno));
        state.regs[0][0x07] |= NE_ISR_TXE;  // ISR register
        state.tx_errors++;
    } else {
        printf("ETH: Transmitted packet of %d bytes (total TX: %d)\n", length, state.tx_packets + 1);
        state.regs[0][0x07] |= NE_ISR_PTX;  // ISR register
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
        accept = (state.regs[0][0x0C] & NE_RCR_AB) != 0;  // RCR register
    }
    // Multicast
    else if (buffer[0] & 0x01) {
        accept = (state.regs[0][0x0C] & NE_RCR_AM) != 0;  // RCR register
    }
    // Unicast to our MAC
    else if (memcmp(buffer, state.mac_addr, 6) == 0) {
        accept = true;
    }
    // Promiscuous mode
    else if (state.regs[0][0x0C] & NE_RCR_PRO) {  // RCR register
        accept = true;
    }
    
    if (!accept) return;
    
    // Check buffer space
    uint8_t curr = state.regs[1][0x07];  // CURR register
    uint8_t bnry = state.regs[0][0x03];  // BNRY register
    uint8_t next = curr + 1 + ((len + NE_PAGE_SIZE - 1) / NE_PAGE_SIZE);
    
    if (next >= NE_RX_STOP) {
        next = NE_RX_START + (next - NE_RX_STOP);
    }
    
    if (next == bnry) {
        // Buffer full
        state.regs[0][0x07] |= NE_ISR_OVW;  // ISR register
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
        memcpy(&state.memory[offset], &header, 4);
        offset += 4;
    }
    
    // Write packet data
    if (offset + len <= NE_MEM_SIZE) {
        memcpy(&state.memory[offset], buffer, len);
    }
    
    // Update current pointer
    state.regs[1][0x07] = next;  // CURR register
    
    // Set packet received interrupt
    state.regs[0][0x07] |= NE_ISR_PRX;  // ISR register
    
    // Copy to shared memory for debugging
    if (len <= 1500) {
        eth_write_shared_mem(ETH_SHM_OFFSET + ETH_RX_BUFFER, buffer, len);
        eth_write_shared_mem(ETH_SHM_OFFSET + ETH_PACKET_INFO + 2, &len, 2);
        uint32_t flags = eth_read_shared_u32(ETH_SHM_OFFSET + ETH_CTRL_FLAGS);
        eth_write_shared_u32(ETH_SHM_OFFSET + ETH_CTRL_FLAGS, flags | ETH_FLAG_RX_AVAIL);
    }
    
    printf("ETH: Received packet of %zd bytes (total RX: %d)\n", len, state.rx_packets + 1);
    
    // Debug: Show first 32 bytes of received packet
    printf("ETH: RX Data: ");
    for (int i = 0; i < 32 && i < len; i++) {
        printf("%02X ", buffer[i]);
        if ((i + 1) % 16 == 0) printf("\nETH: RX Data: ");
    }
    printf("\n");
    
    // Show where packet was stored in NE2000 memory
    printf("ETH: Packet stored at NE2000 offset 0x%04X, next page: 0x%02X\n", offset, next);
    
    state.rx_packets++;
    
    // Write updated state back to shared memory
    write_eth_state(&state);
}

// Main polling function
void minimig_eth_poll()
{
    static int poll_count = 0;

	if (!eth_shmem)
	{
		eth_shmem = (uint8_t *)shmem_map(ETH_SHMEM_ADDR, ETH_SHMEM_SIZE);
		if (!eth_shmem) eth_shmem = (uint8_t *)-1;
	}
	else if(eth_shmem != (uint8_t *)-1)
	{
        // Get current state from shared memory
        struct rtl8019_state state;
        read_eth_state(&state);
        
        // Check for control flags from FPGA via shared memory
        uint32_t flags = eth_read_shared_u32(ETH_SHM_OFFSET + ETH_CTRL_FLAGS);
        
        // Update heartbeat counter every poll
        hps_heartbeat_counter++;
        eth_write_shared_u32(ETH_SHM_OFFSET + ETH_HPS_HEARTBEAT, hps_heartbeat_counter);
        
        // Monitor shared memory buffer activity for FPGA data port access
        static uint16_t last_tx_check = 0;
        static uint16_t last_rx_check = 0;
        
        // Check if FPGA has written to TX buffer (data port writes)
        uint16_t tx_buffer_check = 0;
        eth_read_shared_mem(ETH_SHM_OFFSET + ETH_TX_BUFFER, &tx_buffer_check, 2);
        if (tx_buffer_check != last_tx_check) {
            printf("ETH: FPGA wrote to TX buffer! First word changed: 0x%04X -> 0x%04X\n", 
                   last_tx_check, tx_buffer_check);
            
            // Show first 16 bytes of TX buffer
            uint8_t tx_data[16];
            eth_read_shared_mem(ETH_SHM_OFFSET + ETH_TX_BUFFER, tx_data, 16);
            printf("ETH: TX Buffer data: ");
            for (int i = 0; i < 16; i++) {
                printf("%02X ", tx_data[i]);
            }
            printf("\n");
            
            last_tx_check = tx_buffer_check;
        }
        
        // Check if HPS should provide data to RX buffer (data port reads)  
        // FPGA expects 16-bit data in LSB format
        uint16_t rx_buffer_check = 0;
        eth_read_shared_mem(ETH_SHM_OFFSET + ETH_RX_BUFFER, &rx_buffer_check, 2);
        
        if (rx_buffer_check != last_rx_check) {
            printf("ETH: RX buffer changed! First word: 0x%04X -> 0x%04X\n", 
                   last_rx_check, rx_buffer_check);
            last_rx_check = rx_buffer_check;
        }
        
        // Debug output every 10000 polls (reduced frequency)
        if (++poll_count % 10000 == 0) {
            // Read enabled status from shared memory
            uint32_t status_flags = eth_read_shared_u32(ETH_SHM_OFFSET + ETH_CTRL_FLAGS);
            bool enabled = (status_flags & ETH_FLAG_ENABLED) != 0;
            
            printf("ETH: Poll #%d, flags=0x%08X, enabled=%d, TX:%d, RX:%d, HB:%d\n", 
                   poll_count, flags, enabled, 
                   state.tx_packets, state.rx_packets, hps_heartbeat_counter);
        }
        
        if (flags & ETH_FLAG_REG_DIRTY) {
            // FPGA has updated registers, sync our state (32-bit aligned, LSB format)
            printf("ETH: FPGA updated registers, syncing HPS state...\n");
            
            // Store old values for comparison
            uint8_t old_rsar0 = state.regs[0][0x08];
            uint8_t old_rsar1 = state.regs[0][0x09];
            uint8_t old_rbcr0 = state.regs[0][0x0A];
            uint8_t old_rbcr1 = state.regs[0][0x0B];
            uint8_t old_cr = state.regs[0][0x00];
            
            for (int i = 0; i < 16; i++) {
                state.regs[0][i] = eth_read_shared_reg(ETH_SHM_OFFSET + ETH_CTRL_REGS + (i * 4));
            }
            
            // Debug important DMA-related register changes
            if (old_rsar0 != state.regs[0][0x08] || old_rsar1 != state.regs[0][0x09]) {
                uint16_t old_addr = old_rsar0 | (old_rsar1 << 8);
                uint16_t new_addr = state.regs[0][0x08] | (state.regs[0][0x09] << 8);
                printf("ETH: Remote DMA address changed: 0x%04X -> 0x%04X\n", old_addr, new_addr);
                state.remote_dma_addr = new_addr;
            }
            
            if (old_rbcr0 != state.regs[0][0x0A] || old_rbcr1 != state.regs[0][0x0B]) {
                uint16_t old_count = old_rbcr0 | (old_rbcr1 << 8);
                uint16_t new_count = state.regs[0][0x0A] | (state.regs[0][0x0B] << 8);
                printf("ETH: Remote DMA count changed: %d -> %d\n", old_count, new_count);
                state.remote_dma_count = new_count;
            }
            
            if (old_cr != state.regs[0][0x00]) {
                printf("ETH: Command register changed: 0x%02X -> 0x%02X\n", old_cr, state.regs[0][0x00]);
                if ((state.regs[0][0x00] & 0x38) != 0) {
                    printf("ETH: Remote DMA command: %s\n", 
                           (state.regs[0][0x00] & 0x38) == 0x08 ? "READ" :
                           (state.regs[0][0x00] & 0x38) == 0x10 ? "write" :
                           (state.regs[0][0x00] & 0x38) == 0x18 ? "send packet" : "unknown");
                }
            }
            
            // Clear the dirty flag
            flags &= ~ETH_FLAG_REG_DIRTY;
            eth_write_shared_u32(ETH_SHM_OFFSET + ETH_CTRL_FLAGS, flags);
        }

        // Handle transmit request
        if (flags & ETH_FLAG_TX_REQ) {
            transmit_packet();
            flags &= ~ETH_FLAG_TX_REQ;
            eth_write_shared_u32(ETH_SHM_OFFSET + ETH_CTRL_FLAGS, flags);
        }
        
        // Handle reset request
        if (flags & ETH_FLAG_RESET) {
            minimig_eth_reset();
            flags &= ~ETH_FLAG_RESET;
            eth_write_shared_u32(ETH_SHM_OFFSET + ETH_CTRL_FLAGS, flags);
        }
        
        // Check for incoming packets
        receive_packet();
        
        // Update interrupt status
        uint8_t isr = state.regs[0][0x07];  // ISR register
        uint8_t imr = state.regs[0][0x0F];  // IMR register
        bool irq_active = (isr & imr) != 0;
        
        if (irq_active) {
            flags |= ETH_FLAG_IRQ;
        } else {
            flags &= ~ETH_FLAG_IRQ;
        }
        
        // Update status in shared memory
        uint16_t status = (state.enabled ? 0x01 : 0x00) |
                        (state.link_up ? 0x02 : 0x00);
        
        eth_write_shared_u32(ETH_SHM_OFFSET + ETH_CTRL_FLAGS, flags);
        eth_write_shared_mem(ETH_SHM_OFFSET + ETH_CTRL_STATUS, &status, 2);
        
        // Write updated state back to shared memory
        write_eth_state(&state);
    }
}

// Test function to verify ethernet is working
void minimig_eth_test()
{
    printf("\n===============================================\n");
    printf("ETH: Starting Ethernet Test...\n");
    printf("===============================================\n");
    
    // Test 1: Check shared memory mapping
    printf("ETH TEST 1: Shared memory test\n");
    uint32_t test_val = 0xDEADBEEF;
    eth_write_shared_u32(ETH_SHM_OFFSET + ETH_DEBUG_INFO, test_val);
    uint32_t read_val = eth_read_shared_u32(ETH_SHM_OFFSET + ETH_DEBUG_INFO);
    if (read_val == test_val) {
        printf("  PASS: Shared memory read/write OK (0x%08X)\n", read_val);
    } else {
        printf("  FAIL: Expected 0x%08X, got 0x%08X\n", test_val, read_val);
    }
    
    // Test 2: Check register initialization - read from shared memory
    printf("ETH TEST 2: Register check\n");
    uint8_t cr_reg = eth_read_shared_reg(ETH_SHM_OFFSET + ETH_CTRL_REGS + (0x00 * 4));
    uint8_t isr_reg = eth_read_shared_reg(ETH_SHM_OFFSET + ETH_CTRL_REGS + (0x07 * 4));
    printf("  CR  (0x00) = 0x%02X (expect 0x21)\n", cr_reg);
    printf("  ISR (0x07) = 0x%02X (expect 0x80)\n", isr_reg);
    
    // Read MAC address from shared memory
    uint8_t shared_mac[6];
    eth_read_shared_mem(ETH_SHM_OFFSET + ETH_CTRL_MAC, shared_mac, 6);
    printf("  MAC = %02X:%02X:%02X:%02X:%02X:%02X\n",
           shared_mac[0], shared_mac[1], shared_mac[2],
           shared_mac[3], shared_mac[4], shared_mac[5]);
    
    // Test 3: Check control flags
    printf("ETH TEST 3: Control flags\n");
    uint32_t flags = eth_read_shared_u32(ETH_SHM_OFFSET + ETH_CTRL_FLAGS);
    printf("  Flags = 0x%08X\n", flags);
    printf("  - Reset:    %s\n", (flags & ETH_FLAG_RESET) ? "YES" : "NO");
    printf("  - TX Req:   %s\n", (flags & ETH_FLAG_TX_REQ) ? "YES" : "NO");
    printf("  - RX Avail: %s\n", (flags & ETH_FLAG_RX_AVAIL) ? "YES" : "NO");
    printf("  - IRQ:      %s\n", (flags & ETH_FLAG_IRQ) ? "YES" : "NO");
    printf("  - Enabled:  %s\n", (flags & ETH_FLAG_ENABLED) ? "YES" : "NO");
    
    // Test 4: Socket status
    printf("ETH TEST 4: Network socket\n");
    if (raw_socket >= 0) {
        printf("  PASS: Raw socket is open (fd=%d)\n", raw_socket);
        printf("  Interface: %s\n", bridge_interface);
    } else {
        printf("  FAIL: No raw socket available\n");
    }
    
    printf("===============================================\n");
    printf("ETH: Test Complete\n");
    printf("===============================================\n\n");
}
