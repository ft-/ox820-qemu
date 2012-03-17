/*
 * ox820-rps unit
 *
 * Written by Sven Bormann
 *
 * This code is licensed under the GPL.
 */

#include "sysbus.h"
#include "net.h"

typedef struct {
    SysBusDevice    busdev;
    MemoryRegion    iomem;
    MemoryRegion    iomem_mac;
    MemoryRegion    iomem_dma;
    MemoryRegion    iomem_netoe;

    qemu_irq        mac_irq;
    qemu_irq        pmt_irq;

    uint32_t        cken;
    uint32_t        rsten;
    uint32_t        phyaddr;

    NICState *nic;
    NICConf conf;

    struct {
        uint32_t        mac_config;                 /* +0x00000 */
        uint32_t        mac_frame_filter;           /* +0x00004 */
        uint32_t        mac_table_high;             /* +0x00008 */
        uint32_t        mac_table_low;              /* +0x0000C */
        uint32_t        gmii_address;               /* +0x00010 */
        uint32_t        gmii_data;                  /* +0x00014 */
        uint32_t        flow_control;               /* +0x00018 */
        uint32_t        vlan_tag;                   /* +0x0001C */
        uint32_t        debug;                      /* +0x00024 */
        uint32_t        wakeup_frame_filter;        /* +0x00028 */
        uint32_t        pmt_control_status;         /* +0x0002C */
        uint32_t        interrupt;                  /* +0x00038 */
        uint32_t        interrupt_mask;             /* +0x0003C */
        struct                                      /* +0x00040 */
        {
            uint32_t    high;
            uint32_t    low;
        } macaddress[32];
        uint32_t        mii_status;                 /* +0x000D8 */
        uint32_t        mmc_cntrl;                  /* +0x00100 */
        uint32_t        mmc_intr_rx;                /* +0x00104 */
        uint32_t        mmc_intr_tx;                /* +0x00108 */
        uint32_t        mmc_intr_mask_rx;           /* +0x0010C */
        uint32_t        mmc_intr_mask_tx;           /* +0x00110 */
        uint32_t        txoctectcount_gb;           /* +0x00114 */
        uint32_t        txframecount_gb;            /* +0x00118 */
        uint32_t        txunderflowerror;           /* +0x00148 */
        uint32_t        txpauseframes;              /* +0x00170 */
        uint32_t        rxframecount_gb;            /* +0x00180 */
        uint32_t        rxoctectcount_gb;           /* +0x00188 */
        uint32_t        rxpauseframes;              /* +0x001D0 */
        uint32_t        timestampcontrol;           /* +0x00700 */
        uint32_t        subsecond_inc;              /* +0x00704 */
        uint32_t        timestamp_low;              /* +0x00708 */
        uint32_t        timestamp_high;             /* +0x0070C */
        uint32_t        timestamp_update_high;      /* +0x00710 */
        uint32_t        timestamp_update_low;       /* +0x00714 */
        uint32_t        timestamp_addend;           /* +0x00718 */
        uint32_t        targettimehigh;             /* +0x0071C */
        uint32_t        targettimelow;              /* +0x00720 */
        uint32_t        timestamp_status;           /* +0x00728 */
        uint32_t        pps_control;                /* +0x0072C */
    } mac;
    struct
    {
        uint32_t        bus_mode;                   /* +0x01000 */
        uint32_t        transmit_poll_demand;       /* +0x01004 */
        uint32_t        receive_poll_demand;        /* +0x01008 */
        uint32_t        receive_desc_list;          /* +0x0100C */
        uint32_t        transmit_desc_list;         /* +0x01010 */
        uint32_t        status;                     /* +0x01014 */
        uint32_t        opmode;                     /* +0x01018 */
        uint32_t        intenable;                  /* +0x0101C */
        uint32_t        miss_frame_and_buf_ovl_cnt; /* +0x01020 */
        uint32_t        rx_int_watchdog_timer;      /* +0x01024 */
        uint32_t        cur_host_tx_desc;           /* +0x01048 */
        uint32_t        cur_host_rx_desc;           /* +0x0104C */
        uint32_t        cur_host_tx_bufaddr;        /* +0x01050 */
        uint32_t        cur_host_rx_bufaddr;        /* +0x01054 */
    } dma;
    struct
    {
        uint32_t        job_queue_baseaddr;         /* +0x10000 */
        uint32_t        job_queue_fill_level;       /* +0x10004 */
        uint32_t        job_queue_read_ptr;         /* +0x10008 */
        uint32_t        job_queue_write_ptr;        /* +0x1000C */
        uint32_t        status;                     /* +0x10018 */
        uint32_t        jobs_completed;             /* +0x1001C */
        uint32_t        bytes_transmitted;          /* +0x10020 */
        uint32_t        packets_transmitted;        /* +0x10024 */
        uint32_t        tx_aborts;                  /* +0x10028 */
        uint32_t        tx_collisions;              /* +0x1002C */
        uint32_t        tx_carrier_errors;          /* +0x10030 */
    } netoe;
} ox820_gmac_state;

static void ox820_gmac_irq_update(ox820_gmac_state* s)
{
    int irqset = 0;
    qemu_set_irq(s->mac_irq, irqset);
    qemu_set_irq(s->pmt_irq, irqset);
}

static void ox820_gmac_mac_phy_address(ox820_gmac_state* s)
{
    if(s->mac.gmii_address & 2)
    {
        /* GMII Write */
    }
    else
    {
        /* GMII read */
        s->mac.gmii_data = 0;
    }
}

static uint64_t ox820_gmac_mac_read(void *opaque, target_phys_addr_t offset,
                                    unsigned size)
{
    ox820_gmac_state *s = (ox820_gmac_state *)opaque;
    uint32_t c = 0;

    offset &= 0xFFF;
    switch (offset >> 2) {
    case 0x0000 >> 2:
        c = s->mac.mac_config;
        break;
    case 0x0004 >> 2:
        c = s->mac.mac_frame_filter;
        break;

    case 0x0008 >> 2:
        c = s->mac.mac_table_high;
        break;

    case 0x000C >> 2:
        c = s->mac.mac_table_low;
        break;

    case 0x0010 >> 2:
        c = s->mac.gmii_address;
        break;

    case 0x0014 >> 2:
        c = s->mac.gmii_data;
        break;

    case 0x0018 >> 2:
        c = s->mac.flow_control;
        break;

    case 0x001C >> 2:
        c = s->mac.vlan_tag;
        break;

    case 0x0020 >> 2:
        c = 0x35;
        break;

    case 0x0024 >> 2:
        c = s->mac.debug;
        break;

    case 0x0028 >> 2:
        c = s->mac.wakeup_frame_filter;
        break;

    case 0x002C >> 2:
        c = s->mac.pmt_control_status;
        break;

    case 0x0038 >> 2:
        c = s->mac.interrupt;
        break;

    case 0x003C >> 2:
        c = s->mac.interrupt_mask;
        break;

    case 0x0040 >> 2: case 0x0048 >> 2: case 0x0050 >> 2: case 0x0058 >> 2:
    case 0x0060 >> 2: case 0x0068 >> 2: case 0x0070 >> 2: case 0x0078 >> 2:
    case 0x0080 >> 2: case 0x0088 >> 2: case 0x0090 >> 2: case 0x0098 >> 2:
    case 0x00A0 >> 2: case 0x00A8 >> 2: case 0x00B0 >> 2: case 0x00B8 >> 2:
        c = s->mac.macaddress[offset >> 3].high;
        break;

    case 0x0044 >> 2: case 0x004C >> 2: case 0x0054 >> 2: case 0x005C >> 2:
    case 0x0064 >> 2: case 0x006C >> 2: case 0x0074 >> 2: case 0x007C >> 2:
    case 0x0084 >> 2: case 0x008C >> 2: case 0x0094 >> 2: case 0x009C >> 2:
    case 0x00A4 >> 2: case 0x00AC >> 2: case 0x00B4 >> 2: case 0x00BC >> 2:
        c = s->mac.macaddress[offset >> 3].low;
        break;

    case 0x00D8 >> 2:
        c = s->mac.mii_status;
        break;

    case 0x0100 >> 2:
        c = s->mac.mmc_cntrl;
        break;

    case 0x0104 >> 2:
        c = s->mac.mmc_intr_rx;
        break;

    case 0x0108 >> 2:
        c = s->mac.mmc_intr_tx;
        break;

    case 0x010C >> 2:
        c = s->mac.mmc_intr_mask_rx;
        break;

    case 0x0110 >> 2:
        c = s->mac.mmc_intr_mask_tx;
        break;

    case 0x0114 >> 2:
        c = s->mac.txoctectcount_gb;
        break;

    case 0x0118 >> 2:
        c = s->mac.txframecount_gb;
        break;

    case 0x0148 >> 2:
        c = s->mac.txunderflowerror;
        break;

    case 0x0170 >> 2:
        c = s->mac.txpauseframes;
        break;

    case 0x0180 >> 2:
        c = s->mac.rxframecount_gb;
        break;

    case 0x0188 >> 2:
        c = s->mac.rxoctectcount_gb;
        break;

    case 0x01D0 >> 2:
        c = s->mac.rxpauseframes;
        break;

    case 0x0700 >> 2:
        c = s->mac.timestampcontrol;
        break;

    case 0x0704 >> 2:
        c = s->mac.subsecond_inc;
        break;

    case 0x0708 >> 2:
        c = s->mac.timestamp_low;
        break;

    case 0x070C >> 2:
        c = s->mac.timestamp_high;
        break;

    case 0x0710 >> 2:
        c = s->mac.timestamp_update_high;
        break;

    case 0x0714 >> 2:
        c = s->mac.timestamp_update_low;
        break;

    case 0x0718 >> 2:
        c = s->mac.timestamp_addend;
        break;

    case 0x071C >> 2:
        c = s->mac.targettimehigh;
        break;

    case 0x0720 >> 2:
        c = s->mac.targettimelow;
        break;

    case 0x0728 >> 2:
        c = s->mac.timestamp_status;
        break;

    case 0x072C >> 2:
        c = s->mac.pps_control;
        break;

    case 0x0800 >> 2: case 0x0808 >> 2: case 0x0810 >> 2: case 0x0818 >> 2:
    case 0x0820 >> 2: case 0x0828 >> 2: case 0x0830 >> 2: case 0x0838 >> 2:
    case 0x0840 >> 2: case 0x0848 >> 2: case 0x0850 >> 2: case 0x0858 >> 2:
    case 0x0860 >> 2: case 0x0868 >> 2: case 0x0870 >> 2: case 0x0878 >> 2:
        c = s->mac.macaddress[(offset >> 3) - 240].high;
        break;

    case 0x0804 >> 2: case 0x080C >> 2: case 0x0814 >> 2: case 0x081C >> 2:
    case 0x0824 >> 2: case 0x082C >> 2: case 0x0834 >> 2: case 0x083C >> 2:
    case 0x0844 >> 2: case 0x084C >> 2: case 0x0854 >> 2: case 0x085C >> 2:
    case 0x0864 >> 2: case 0x086C >> 2: case 0x0874 >> 2: case 0x087C >> 2:
        c = s->mac.macaddress[(offset >> 3) - 240].low;
        break;

    default:
        break;
    }
    return c;
}

static void ox820_gmac_mac_write(void *opaque, target_phys_addr_t offset,
                                 uint64_t value, unsigned size)
{
    ox820_gmac_state *s = (ox820_gmac_state *)opaque;
    offset &= 0xFFF;

    switch (offset >> 2) {
    case 0x0000 >> 2:
        s->mac.mac_config = value & 0x3FFFFFFC;
        break;
    case 0x0004 >> 2:
        s->mac.mac_frame_filter = value & 0x800007FF;
        break;

    case 0x0008 >> 2:
        s->mac.mac_table_high = value;
        break;

    case 0x000C >> 2:
        s->mac.mac_table_low = value;
        break;

    case 0x0010 >> 2:
        s->mac.gmii_address = value & 0x0000FFFE;
        ox820_gmac_mac_phy_address(s);
        break;

    case 0x0014 >> 2:
        s->mac.gmii_data = value & 0xFFFF;
        break;

    case 0x0018 >> 2:
        s->mac.flow_control = value & 0xFFFF00BF;
        break;

    case 0x001C >> 2:
        s->mac.vlan_tag = value & 0x1FFFF;
        break;

    case 0x0028 >> 2:
        s->mac.wakeup_frame_filter = value;
        break;

    case 0x002C >> 2:
        s->mac.pmt_control_status = (value & 0x000209) | (s->mac.pmt_control_status & ~0x209);
        break;

    case 0x003C >> 2:
        s->mac.interrupt_mask = (value & 0x20F) | (s->mac.interrupt_mask & ~0x20F);
        break;

    case 0x0040 >> 2: case 0x0048 >> 2: case 0x0050 >> 2: case 0x0058 >> 2:
    case 0x0060 >> 2: case 0x0068 >> 2: case 0x0070 >> 2: case 0x0078 >> 2:
    case 0x0080 >> 2: case 0x0088 >> 2: case 0x0090 >> 2: case 0x0098 >> 2:
    case 0x00A0 >> 2: case 0x00A8 >> 2: case 0x00B0 >> 2: case 0x00B8 >> 2:
        if((offset >> 3) == 0)
        {
            value &= 0x0000FFFF;
            value |= 0x80000000;
        }
        else
        {
            value &= 0xFF00FFFF;
        }
        s->mac.macaddress[offset >> 3].high = value;
        break;

    case 0x0044 >> 2: case 0x004C >> 2: case 0x0054 >> 2: case 0x005C >> 2:
    case 0x0064 >> 2: case 0x006C >> 2: case 0x0074 >> 2: case 0x007C >> 2:
    case 0x0084 >> 2: case 0x008C >> 2: case 0x0094 >> 2: case 0x009C >> 2:
    case 0x00A4 >> 2: case 0x00AC >> 2: case 0x00B4 >> 2: case 0x00BC >> 2:
        s->mac.macaddress[offset >> 3].low = value;
        break;

    case 0x0100 >> 2:
        s->mac.mmc_cntrl = value & 0x3F;
        break;

    case 0x0104 >> 2:
        s->mac.mmc_intr_rx = value & 0x00FFFFFF;
        break;

    case 0x0108 >> 2:
        s->mac.mmc_intr_tx = value & 0x01FFFFFF;
        break;

    case 0x010C >> 2:
        s->mac.mmc_intr_mask_rx = value & 0x00FFFFFF;
        break;

    case 0x0110 >> 2:
        s->mac.mmc_intr_mask_tx = value & 0x01FFFFFF;
        break;

    case 0x0114 >> 2:
        s->mac.txoctectcount_gb = value;
        break;

    case 0x0118 >> 2:
        s->mac.txframecount_gb = value;
        break;

    case 0x0148 >> 2:
        s->mac.txunderflowerror = value;
        break;

    case 0x0170 >> 2:
        s->mac.txpauseframes = value;
        break;

    case 0x0180 >> 2:
        s->mac.rxframecount_gb = value;
        break;

    case 0x0188 >> 2:
        s->mac.rxoctectcount_gb = value;
        break;

    case 0x01D0 >> 2:
        s->mac.rxpauseframes = value;
        break;

    case 0x0700 >> 2:
        s->mac.timestampcontrol = value & 0x0007FF1F;
        break;

    case 0x0704 >> 2:
        s->mac.subsecond_inc = value & 0xFF;
        break;

    case 0x0710 >> 2:
        s->mac.timestamp_update_high = value;
        break;

    case 0x0714 >> 2:
        s->mac.timestamp_update_low = value;
        break;

    case 0x0718 >> 2:
        s->mac.timestamp_addend = value;
        break;

    case 0x071C >> 2:
        s->mac.targettimehigh = value;
        break;

    case 0x0720 >> 2:
        s->mac.targettimelow = value & 0x7fffffff;
        break;

    case 0x072C >> 2:
        s->mac.pps_control = value & 0xf;
        break;

    case 0x0800 >> 2: case 0x0808 >> 2: case 0x0810 >> 2: case 0x0818 >> 2:
    case 0x0820 >> 2: case 0x0828 >> 2: case 0x0830 >> 2: case 0x0838 >> 2:
    case 0x0840 >> 2: case 0x0848 >> 2: case 0x0850 >> 2: case 0x0858 >> 2:
    case 0x0860 >> 2: case 0x0868 >> 2: case 0x0870 >> 2: case 0x0878 >> 2:
        s->mac.macaddress[(offset >> 3) - 240].high = value & 0xff00ffff;
        break;

    case 0x0804 >> 2: case 0x080C >> 2: case 0x0814 >> 2: case 0x081C >> 2:
    case 0x0824 >> 2: case 0x082C >> 2: case 0x0834 >> 2: case 0x083C >> 2:
    case 0x0844 >> 2: case 0x084C >> 2: case 0x0854 >> 2: case 0x085C >> 2:
    case 0x0864 >> 2: case 0x086C >> 2: case 0x0874 >> 2: case 0x087C >> 2:
        s->mac.macaddress[(offset >> 3) - 240].low = value;
        break;

    default:
        break;
    }
}

static uint64_t ox820_gmac_dma_read(void *opaque, target_phys_addr_t offset,
                                    unsigned size)
{
    ox820_gmac_state *s = (ox820_gmac_state *)opaque;
    uint32_t c = 0;

    offset &= 0x0FFF;
    switch (offset >> 2) {
    case 0x000 >> 2:
        c = s->dma.bus_mode;
        break;

    case 0x004 >> 2:
        c = s->dma.transmit_poll_demand;
        break;

    case 0x008 >> 2:
        c = s->dma.receive_poll_demand;
        break;

    case 0x00C >> 2:
        c = s->dma.receive_desc_list;
        break;

    case 0x010 >> 2:
        c = s->dma.transmit_desc_list;
        break;

    case 0x014 >> 2:
        c = s->dma.status;
        break;

    case 0x018 >> 2:
        c = s->dma.opmode;
        break;

    case 0x001C >> 2:
        c = s->dma.intenable;
        break;

    case 0x0020 >> 2:
        c = s->dma.miss_frame_and_buf_ovl_cnt;
        break;

    case 0x0024 >> 2:
        c = s->dma.rx_int_watchdog_timer;
        break;

    case 0x0048 >> 2:
        c = s->dma.cur_host_tx_desc;
        break;

    case 0x004C >> 2:
        c = s->dma.cur_host_rx_desc;
        break;

    case 0x0050 >> 2:
        c = s->dma.cur_host_tx_bufaddr;
        break;

    case 0x0054 >> 2:
        c = s->dma.cur_host_rx_bufaddr;
        break;

    case 0x0058 >> 2:
        c = 0x010F3F73;
        break;

    default:
        break;
    }
    return c;
}

static void ox820_gmac_dma_write(void *opaque, target_phys_addr_t offset,
                                 uint64_t value, unsigned size)
{
    ox820_gmac_state *s = (ox820_gmac_state *)opaque;

    offset &= 0xFFF;
    switch (offset >> 2) {
    case 0x000 >> 2:
        s->dma.bus_mode = value & 0x07FFFF7F;
        break;

    case 0x004 >> 2:
        break;

    case 0x008 >> 2:
        break;

    case 0x00C >> 2:
        s->dma.receive_desc_list = value;
        break;

    case 0x010 >> 2:
        s->dma.transmit_desc_list = value;
        break;

    case 0x014 >> 2:
        s->dma.status &= (~value);
        break;

    case 0x018 >> 2:
        s->dma.opmode = value & 0x07F1FFDE;
        break;

    case 0x001C >> 2:
        s->dma.intenable = value & 0x0001E7FF;
        break;

    case 0x0020 >> 2:
        s->dma.miss_frame_and_buf_ovl_cnt = value & 0x1FFFFFFF;
        break;

    case 0x0024 >> 2:
        s->dma.rx_int_watchdog_timer = value & 0xFF;
        break;

    default:
        break;
    }
}

static uint64_t ox820_gmac_netoe_read(void *opaque, target_phys_addr_t offset,
                                      unsigned size)
{
    ox820_gmac_state *s = (ox820_gmac_state *)opaque;
    uint32_t c = 0;

    offset &= 0xFFFF;
    switch (offset >> 2) {
    case 0x0000 >> 2:
        c = s->netoe.job_queue_baseaddr;
        break;

    case 0x0004 >> 2:
        c = s->netoe.job_queue_fill_level;
        break;

    case 0x0008 >> 2:
        c = s->netoe.job_queue_read_ptr;
        break;

    case 0x000C >> 2:
        c = s->netoe.job_queue_write_ptr;
        break;

    case 0x0018 >> 2:
        c = s->netoe.status;
        break;

    case 0x001C >> 2:
        c = s->netoe.jobs_completed;
        break;

    case 0x0020 >> 2:
        c = s->netoe.bytes_transmitted;
        break;

    case 0x0024 >> 2:
        c = s->netoe.packets_transmitted;
        break;

    case 0x0028 >> 2:
        c = s->netoe.tx_aborts;
        break;

    case 0x002C >> 2:
        c = s->netoe.tx_collisions;
        break;

    case 0x0030 >> 2:
        c = s->netoe.tx_carrier_errors;
        break;

    default:
        break;
    }
    return c;
}

static void ox820_gmac_netoe_write(void *opaque, target_phys_addr_t offset,
                                   uint64_t value, unsigned size)
{
    //ox820_gmac_state *s = (ox820_gmac_state *)opaque;

    switch (offset >> 2) {

    default:
        break;
    }
}

static void ox820_gmac_set_irq(void *opaque, int irq, int level)
{
    ox820_gmac_state *s = (ox820_gmac_state *)opaque;

    if(irq == 0)
    {
        s->rsten = level != 0;
    }
    if(irq == 1)
    {
        s->cken = level != 0;
    }

    ox820_gmac_irq_update(s);
}


static int ox820_gmac_eth_can_receive(VLANClientState *nc)
{
    ox820_gmac_state *s = DO_UPCAST(NICState, nc, nc)->opaque;
    return s->rsten == 0 && s->cken != 0;
}

static ssize_t ox820_gmac_eth_receive(VLANClientState *nc, const uint8_t *buf, size_t size)
{
//    unsigned char sa_bcast[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
//    ox820_gmac_state *s = DO_UPCAST(NICState, nc, nc)->opaque;

    if (size < 12) {
        return -1;
    }

#if 0
    /* Does the frame get through the address filters?  */
    if ((!use_ma0 || memcmp(buf, eth->macaddr[0], 6))
        && (!use_ma1 || memcmp(buf, eth->macaddr[1], 6))
        && (!r_bcast || memcmp(buf, sa_bcast, 6))
        && !eth_match_groupaddr(eth, buf))
        return size;
#endif
    /* FIXME: Find another way to pass on the fake csum.  */
//    etraxfs_dmac_input(eth->dma_in, (void *)buf, size + 4, 1);

    return size;
}

#if 0
static int ox820_gmac_eth_tx_push(void *opaque, unsigned char *buf, int len, bool eop)
{
    ox820_gmac_state *s = opaque;

    qemu_send_packet(&s->nic->nc, buf, len);
    return len;
}
#endif

static void ox820_gmac_eth_set_link(VLANClientState *nc)
{
    //ox820_gmac_state* s = DO_UPCAST(NICState, nc, nc)->opaque;
    //D(printf("%s %d\n", __func__, nc->link_down));
    //eth->phy.link = !nc->link_down;
}

static void ox820_gmac_eth_cleanup(VLANClientState *nc)
{
//    ox820_gmac_state* s = DO_UPCAST(NICState, nc, nc)->opaque;

#if 0
    /* Disconnect the client.  */
    eth->dma_out->client.push = NULL;
    eth->dma_out->client.opaque = NULL;
    eth->dma_in->client.opaque = NULL;
    eth->dma_in->client.pull = NULL;
#endif
}


static NetClientInfo ox820_gmac_eth_info = {
    .type = NET_CLIENT_TYPE_NIC,
    .size = sizeof(NICState),
    .can_receive = ox820_gmac_eth_can_receive,
    .receive = ox820_gmac_eth_receive,
    .cleanup = ox820_gmac_eth_cleanup,
    .link_status_changed = ox820_gmac_eth_set_link,
};

static void ox820_gmac_reset(DeviceState *d)
{
    ox820_gmac_state *s = DO_UPCAST(ox820_gmac_state, busdev.qdev, d);

    ox820_gmac_irq_update(s);
}

static const MemoryRegionOps ox820_gmac_mac_ops = {
    .read = ox820_gmac_mac_read,
    .write = ox820_gmac_mac_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps ox820_gmac_dma_ops = {
    .read = ox820_gmac_dma_read,
    .write = ox820_gmac_dma_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps ox820_gmac_netoe_ops = {
    .read = ox820_gmac_netoe_read,
    .write = ox820_gmac_netoe_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_ox820_gmac = {
    .name = "ox820-gmac",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(mac.mac_config, ox820_gmac_state),
        VMSTATE_UINT32(mac.mac_frame_filter, ox820_gmac_state),
        VMSTATE_UINT32(mac.mac_table_high, ox820_gmac_state),
        VMSTATE_UINT32(mac.mac_table_low, ox820_gmac_state),
        VMSTATE_UINT32(mac.gmii_address, ox820_gmac_state),
        VMSTATE_UINT32(mac.gmii_data, ox820_gmac_state),
        VMSTATE_UINT32(mac.flow_control, ox820_gmac_state),
        VMSTATE_UINT32(mac.vlan_tag, ox820_gmac_state),
        VMSTATE_UINT32(mac.debug, ox820_gmac_state),
        VMSTATE_UINT32(mac.wakeup_frame_filter, ox820_gmac_state),
        VMSTATE_UINT32(mac.pmt_control_status, ox820_gmac_state),
        VMSTATE_UINT32(mac.interrupt, ox820_gmac_state),
        VMSTATE_UINT32(mac.interrupt_mask, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[0].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[0].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[1].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[1].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[2].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[2].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[3].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[3].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[4].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[4].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[5].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[5].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[6].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[6].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[7].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[7].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[8].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[8].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[9].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[9].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[10].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[10].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[11].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[11].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[12].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[12].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[13].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[13].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[14].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[14].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[15].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[15].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[16].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[16].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[17].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[17].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[18].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[18].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[19].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[19].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[20].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[20].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[21].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[21].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[22].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[22].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[23].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[23].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[24].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[24].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[25].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[25].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[26].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[26].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[27].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[27].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[28].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[28].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[29].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[29].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[30].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[30].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[31].high, ox820_gmac_state),
        VMSTATE_UINT32(mac.macaddress[31].low, ox820_gmac_state),
        VMSTATE_UINT32(mac.mii_status, ox820_gmac_state),
        VMSTATE_UINT32(mac.mmc_cntrl, ox820_gmac_state),
        VMSTATE_UINT32(mac.mmc_intr_rx, ox820_gmac_state),
        VMSTATE_UINT32(mac.mmc_intr_tx, ox820_gmac_state),
        VMSTATE_UINT32(mac.mmc_intr_mask_rx, ox820_gmac_state),
        VMSTATE_UINT32(mac.mmc_intr_mask_tx, ox820_gmac_state),
        VMSTATE_UINT32(mac.txoctectcount_gb, ox820_gmac_state),
        VMSTATE_UINT32(mac.txframecount_gb, ox820_gmac_state),
        VMSTATE_UINT32(mac.txunderflowerror, ox820_gmac_state),
        VMSTATE_UINT32(mac.txpauseframes, ox820_gmac_state),
        VMSTATE_UINT32(mac.rxframecount_gb, ox820_gmac_state),
        VMSTATE_UINT32(mac.rxoctectcount_gb, ox820_gmac_state),
        VMSTATE_UINT32(mac.rxpauseframes, ox820_gmac_state),
        VMSTATE_UINT32(mac.timestampcontrol, ox820_gmac_state),
        VMSTATE_UINT32(mac.subsecond_inc, ox820_gmac_state),
        VMSTATE_UINT32(mac.timestamp_low, ox820_gmac_state),
        VMSTATE_UINT32(mac.timestamp_high, ox820_gmac_state),
        VMSTATE_UINT32(mac.timestamp_update_high, ox820_gmac_state),
        VMSTATE_UINT32(mac.timestamp_update_low, ox820_gmac_state),
        VMSTATE_UINT32(mac.timestamp_addend, ox820_gmac_state),
        VMSTATE_UINT32(mac.targettimehigh, ox820_gmac_state),
        VMSTATE_UINT32(mac.targettimelow, ox820_gmac_state),
        VMSTATE_UINT32(mac.timestamp_status, ox820_gmac_state),
        VMSTATE_UINT32(mac.pps_control, ox820_gmac_state),
        VMSTATE_UINT32(dma.bus_mode, ox820_gmac_state),
        VMSTATE_UINT32(dma.transmit_poll_demand, ox820_gmac_state),
        VMSTATE_UINT32(dma.receive_poll_demand, ox820_gmac_state),
        VMSTATE_UINT32(dma.receive_desc_list, ox820_gmac_state),
        VMSTATE_UINT32(dma.transmit_desc_list, ox820_gmac_state),
        VMSTATE_UINT32(dma.status, ox820_gmac_state),
        VMSTATE_UINT32(dma.opmode, ox820_gmac_state),
        VMSTATE_UINT32(dma.intenable, ox820_gmac_state),
        VMSTATE_UINT32(dma.miss_frame_and_buf_ovl_cnt, ox820_gmac_state),
        VMSTATE_UINT32(dma.rx_int_watchdog_timer, ox820_gmac_state),
        VMSTATE_UINT32(dma.cur_host_tx_desc, ox820_gmac_state),
        VMSTATE_UINT32(dma.cur_host_rx_desc, ox820_gmac_state),
        VMSTATE_UINT32(dma.cur_host_tx_bufaddr, ox820_gmac_state),
        VMSTATE_UINT32(dma.cur_host_rx_bufaddr, ox820_gmac_state),
        VMSTATE_UINT32(netoe.job_queue_baseaddr, ox820_gmac_state),
        VMSTATE_UINT32(netoe.job_queue_fill_level, ox820_gmac_state),
        VMSTATE_UINT32(netoe.job_queue_read_ptr, ox820_gmac_state),
        VMSTATE_UINT32(netoe.job_queue_write_ptr, ox820_gmac_state),
        VMSTATE_UINT32(netoe.status, ox820_gmac_state),
        VMSTATE_UINT32(netoe.jobs_completed, ox820_gmac_state),
        VMSTATE_UINT32(netoe.bytes_transmitted, ox820_gmac_state),
        VMSTATE_UINT32(netoe.packets_transmitted, ox820_gmac_state),
        VMSTATE_UINT32(netoe.tx_aborts, ox820_gmac_state),
        VMSTATE_UINT32(netoe.tx_collisions, ox820_gmac_state),
        VMSTATE_UINT32(netoe.tx_carrier_errors, ox820_gmac_state),
        VMSTATE_UINT32(cken, ox820_gmac_state),
        VMSTATE_UINT32(rsten, ox820_gmac_state),
        VMSTATE_UINT32(phyaddr, ox820_gmac_state),
        VMSTATE_END_OF_LIST()
    }
};

static int ox820_gmac_init(SysBusDevice *dev)
{
    ox820_gmac_state *s = FROM_SYSBUS(ox820_gmac_state, dev);

    memory_region_init(&s->iomem, "ox820-gmac", 0x20000);
    memory_region_init_io(&s->iomem_mac, &ox820_gmac_mac_ops, s, "mac", 0x1000);
    memory_region_init_io(&s->iomem_dma, &ox820_gmac_dma_ops, s, "dma", 0x1000);
    memory_region_init_io(&s->iomem_netoe, &ox820_gmac_netoe_ops, s, "netoe", 0x1000);
    memory_region_add_subregion(&s->iomem, 0x00000, &s->iomem_mac);
    memory_region_add_subregion(&s->iomem, 0x01000, &s->iomem_dma);
    memory_region_add_subregion(&s->iomem, 0x10000, &s->iomem_netoe);

    sysbus_init_mmio(dev, &s->iomem);
    qdev_init_gpio_in(&dev->qdev, ox820_gmac_set_irq, 2);
    sysbus_init_irq(dev, &s->mac_irq);
    sysbus_init_irq(dev, &s->pmt_irq);
    s->cken = 0;
    s->rsten = 1;

    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&ox820_gmac_eth_info, &s->conf,
                  object_get_typename(OBJECT(s)), dev->qdev.id, s);
    qemu_format_nic_info_str(&s->nic->nc, s->conf.macaddr.a);

    s->mac.macaddress[0].high = 0x80000000 | (s->conf.macaddr.a[5] << 8) | s->conf.macaddr.a[4];
    s->mac.macaddress[0].low = (s->conf.macaddr.a[3] << 24) | (s->conf.macaddr.a[2] << 16) | (s->conf.macaddr.a[1] << 8) | s->conf.macaddr.a[0];

    vmstate_register(&dev->qdev, -1, &vmstate_ox820_gmac, s);
    return 0;
}

static Property ox820_gmac_properties[] = {
    DEFINE_PROP_UINT32("phyaddr", ox820_gmac_state, phyaddr, 1),
    DEFINE_PROP_END_OF_LIST(),
};


static void ox820_gmac_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->no_user = 1;
    sdc->init = ox820_gmac_init;
    dc->reset = ox820_gmac_reset;
    dc->props = ox820_gmac_properties;
}

static TypeInfo ox820_gmac_info = {
    .name          = "ox820-gmac",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ox820_gmac_state),
    .class_init    = ox820_gmac_class_init,
};

static void ox820_gmac_register_devices(void)
{
    type_register_static(&ox820_gmac_info);
}

device_init(ox820_gmac_register_devices)
