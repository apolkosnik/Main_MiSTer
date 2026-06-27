#ifndef ETH_GSO_H
#define ETH_GSO_H

// Software GSO (generic segmentation offload) for the RX intake.
//
// With host receive offload (GRO/LRO) enabled, an AF_PACKET tap is handed frames
// AFTER the kernel has coalesced several in-order TCP segments into one oversized
// skb carrying a single, coherent IPv4/TCP header whose total-length covers the
// merged payload. A 10 Mbit NE2000 / the Amiga TCP stack cannot accept such a
// frame, and a real X-Surf NIC never sees one. Rather than disable the offload
// (which throws away the host's batching), we do what the kernel itself does when
// such a packet leaves a non-GRO device: re-segment it in software back into
// <=MTU TCP segments, fixing each segment's sequence number, IP total-length,
// IP identification, PSH/FIN flags, and recomputing the IPv4 + TCP checksums.
//
// Header-only / static so the daemon and the unit test build the identical code.

#include <stdint.h>
#include <string.h>

// Max emitted frame: L2 (incl. VLAN) + a full 1500-byte IP packet = <=1518.
#define ETH_GSO_MTU        1500
#define ETH_GSO_MAX_FRAME  1600

// Caller-supplied sink for each produced <=MTU frame, in emission (sequence) order.
typedef void (*eth_gso_emit_fn)(void* ctx, const uint8_t* frame, int len);

// RFC 1071 internet checksum, accumulating big-endian 16-bit words.
static inline uint32_t eth_gso_csum_add(const uint8_t* data, int len, uint32_t sum)
{
    int i = 0;
    for (; i + 1 < len; i += 2) {
        sum += (uint32_t)((data[i] << 8) | data[i + 1]);
    }
    if (i < len) {
        sum += (uint32_t)(data[i] << 8);   // final odd byte occupies the high lane
    }
    return sum;
}

static inline uint16_t eth_gso_csum_fold(uint32_t sum)
{
    while (sum >> 16) {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    return (uint16_t)(~sum);
}

// Re-segment one received frame. For an oversized, in-order IPv4/TCP data frame
// it emits ceil(payload / MSS) <=MTU segments and returns their count. Returns 0
// (emitting nothing) when the frame is not a coalesced IPv4/TCP data segment the
// caller should re-split (non-IPv4, non-TCP, fragmented, SYN/RST/URG, no payload)
// -- the caller then drops it. A frame that already fits one segment yields 1.
static inline int eth_gso_resegment(const uint8_t* in, int in_len,
                                    eth_gso_emit_fn emit, void* ctx)
{
    if (in_len < 14) return 0;

    int l2 = 14;
    int ethertype = (in[12] << 8) | in[13];
    if (ethertype == 0x8100) {                 // 802.1Q VLAN tag
        if (in_len < 18) return 0;
        l2 = 18;
        ethertype = (in[16] << 8) | in[17];
    }
    if (ethertype != 0x0800) return 0;         // IPv4 only

    const uint8_t* ip = in + l2;
    if (in_len < l2 + 20) return 0;
    if ((ip[0] >> 4) != 4) return 0;
    int ihl = (ip[0] & 0x0F) * 4;
    if (ihl < 20 || in_len < l2 + ihl) return 0;
    if (ip[9] != 6) return 0;                  // TCP only

    int ip_total = (ip[2] << 8) | ip[3];
    if (ip_total < ihl) return 0;
    int mf       = (ip[6] & 0x20) != 0;        // more-fragments
    int frag_off = ((ip[6] & 0x1F) << 8) | ip[7];
    if (mf || frag_off) return 0;              // never re-segment an IP fragment

    const uint8_t* tcp = ip + ihl;
    if (in_len < l2 + ihl + 20) return 0;
    int thl = ((tcp[12] >> 4) & 0x0F) * 4;
    if (thl < 20 || in_len < l2 + ihl + thl) return 0;

    uint8_t flags = tcp[13];                   // FIN1 SYN2 RST4 PSH8 ACK16 URG32
    if (flags & (0x02 | 0x04 | 0x20)) return 0; // SYN/RST/URG: not coalesced data

    int avail = in_len - (l2 + ihl + thl);
    int payload_total = ip_total - ihl - thl;
    // Some AF_PACKET/GRO paths expose the coalesced payload bytes while leaving
    // the IPv4 total-length at the first segment's value.  When the captured
    // frame is already larger than a wire-MTU packet, trust the captured payload
    // length and split all bytes instead of silently emitting only the first MSS.
    if (in_len > l2 + ETH_GSO_MTU && avail > payload_total) payload_total = avail;
    if (payload_total > avail) payload_total = avail;
    if (payload_total <= 0) return 0;

    const uint8_t* payload = tcp + thl;
    uint32_t seq0  = ((uint32_t)tcp[4] << 24) | ((uint32_t)tcp[5] << 16) |
                     ((uint32_t)tcp[6] << 8)  | (uint32_t)tcp[7];
    uint16_t ip_id0 = (uint16_t)((ip[4] << 8) | ip[5]);

    int max_payload = ETH_GSO_MTU - ihl - thl;
    if (max_payload <= 0) return 0;

    int hdr_len = l2 + ihl + thl;
    int nseg = 0;
    int off = 0;
    while (off < payload_total) {
        int chunk = payload_total - off;
        if (chunk > max_payload) chunk = max_payload;
        int last = (off + chunk >= payload_total);

        uint8_t seg[ETH_GSO_MAX_FRAME];
        memcpy(seg, in, (size_t)hdr_len);                       // L2 + IP + TCP (incl. options/VLAN)
        memcpy(seg + hdr_len, payload + off, (size_t)chunk);
        int seg_total = hdr_len + chunk;

        // --- IPv4 header fix-ups ---
        uint8_t* sip = seg + l2;
        int sip_total = ihl + thl + chunk;
        sip[2] = (uint8_t)(sip_total >> 8);
        sip[3] = (uint8_t)(sip_total & 0xFF);
        uint16_t sid = (uint16_t)(ip_id0 + nseg);               // GSO increments IP id per segment
        sip[4] = (uint8_t)(sid >> 8);
        sip[5] = (uint8_t)(sid & 0xFF);
        sip[10] = 0; sip[11] = 0;
        uint16_t ipck = eth_gso_csum_fold(eth_gso_csum_add(sip, ihl, 0));
        sip[10] = (uint8_t)(ipck >> 8);
        sip[11] = (uint8_t)(ipck & 0xFF);

        // --- TCP header fix-ups ---
        uint8_t* stcp = seg + l2 + ihl;
        uint32_t sseq = seq0 + (uint32_t)off;                   // mod-2^32 wrap is correct
        stcp[4] = (uint8_t)(sseq >> 24);
        stcp[5] = (uint8_t)(sseq >> 16);
        stcp[6] = (uint8_t)(sseq >> 8);
        stcp[7] = (uint8_t)(sseq);
        // PSH/FIN belong only on the final segment; ACK and the rest are replicated.
        uint8_t sflags = (uint8_t)(flags & ~(0x08 | 0x01));
        if (last) sflags |= (uint8_t)(flags & (0x08 | 0x01));
        stcp[13] = sflags;
        stcp[16] = 0; stcp[17] = 0;
        uint32_t sum = eth_gso_csum_add(sip + 12, 8, 0);        // pseudo-header: src + dst IP
        sum += 6;                                               // pseudo-header: protocol = TCP
        sum += (uint32_t)(thl + chunk);                         // pseudo-header: TCP length
        sum = eth_gso_csum_add(stcp, thl + chunk, sum);
        uint16_t tck = eth_gso_csum_fold(sum);
        stcp[16] = (uint8_t)(tck >> 8);
        stcp[17] = (uint8_t)(tck & 0xFF);

        emit(ctx, seg, seg_total);
        nseg++;
        off += chunk;
    }
    return nseg;
}

#endif // ETH_GSO_H
