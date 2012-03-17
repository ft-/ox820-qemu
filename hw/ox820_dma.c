/*
 * ox820-rps unit
 *
 * Written by Sven Bormann
 *
 * This code is licensed under the GPL.
 */

#include "sysbus.h"
#include "cpu-common.h"
#include "ox820_dma.h"

#define OFS_DMA_CTRL_STAT           0x00
#define MSK_DMA_CTRL_STAT_CLEAR_INT_REG_EN      (1 << 30)
#define MSK_DMA_CTRL_STAT_STARVE_LOW_PRIO       (1 << 29)
#define MSK_DMA_CTRL_STAT_FIXED_ADDR_D          (1 << 28)
#define MSK_DMA_CTRL_STAT_FIXED_ADDR_S          (1 << 27)
#define MSK_DMA_CTRL_STAT_INT_ENABLE            (1 << 26)
#define MSK_DMA_CTRL_STAT_PAUSE_DMA             (1 << 25)
#define MSK_DMA_CTRL_STAT_DEVICE_TYPE_D         (7 << 22)
#define VAL_DMA_CTRL_STAT_DEVICE_TYPE_D_8BIT    (0 << 22)
#define VAL_DMA_CTRL_STAT_DEVICE_TYPE_D_16BIT   (1 << 22)
#define VAL_DMA_CTRL_STAT_DEVICE_TYPE_D_32BIT   (2 << 22)
#define MSK_DMA_CTRL_STAT_DEVICE_TYPE_S         (7 << 19)
#define VAL_DMA_CTRL_STAT_DEVICE_TYPE_S_8BIT    (0 << 19)
#define VAL_DMA_CTRL_STAT_DEVICE_TYPE_S_16BIT   (1 << 19)
#define VAL_DMA_CTRL_STAT_DEVICE_TYPE_S_32BIT   (2 << 19)
#define MSK_DMA_CTRL_STAT_MODE_B                (1 << 18)
#define MSK_DMA_CTRL_STAT_MODE_A                (1 << 17)
#define MSK_DMA_CTRL_STAT_INC_ADDR_D            (1 << 16)
#define MSK_DMA_CTRL_STAT_INC_ADDR_S            (1 << 15)
#define MSK_DMA_CTRL_STAT_DIRECTION             (2 << 13)
#define MSK_DMA_CTRL_STAT_DIRECTION_SRC_SELECT  (1 << 13)
#define MSK_DMA_CTRL_STAT_DIRECTION_DST_SELECT  (1 << 14)
#define MSK_DMA_CTRL_STAT_CH_RESET              (1 << 12)
#define MSK_DMA_CTRL_STAT_NEXT_FREE             (1 << 11)
#define MSK_DMA_CTRL_STAT_INT                   (1 << 10)
#define MSK_DMA_CTRL_STAT_SFT_D_DREQ            (0xF << 6)
#define MSK_DMA_CTRL_STAT_SFT_S_DREQ            (0xF << 2)
#define MSK_DMA_CTRL_STAT_DMA_IN_PROGRESS       (1 << 1)
#define MSK_DMA_CTRL_STAT_FAIR_SHARE_ARB        (1 << 0)
#define MSK_DMA_CTRL_STAT_RW_MASK   (MSK_DMA_CTRL_STAT_CLEAR_INT_REG_EN | \
                                     MSK_DMA_CTRL_STAT_STARVE_LOW_PRIO | \
                                     MSK_DMA_CTRL_STAT_FIXED_ADDR_D | \
                                     MSK_DMA_CTRL_STAT_FIXED_ADDR_S | \
                                     MSK_DMA_CTRL_STAT_INT_ENABLE | \
                                     MSK_DMA_CTRL_STAT_PAUSE_DMA | \
                                     MSK_DMA_CTRL_STAT_DEVICE_TYPE_D | \
                                     MSK_DMA_CTRL_STAT_DEVICE_TYPE_S | \
                                     MSK_DMA_CTRL_STAT_MODE_B | \
                                     MSK_DMA_CTRL_STAT_MODE_A | \
                                     MSK_DMA_CTRL_STAT_INC_ADDR_D | \
                                     MSK_DMA_CTRL_STAT_INC_ADDR_S | \
                                     MSK_DMA_CTRL_STAT_DIRECTION | \
                                     MSK_DMA_CTRL_STAT_CH_RESET | \
                                     MSK_DMA_CTRL_STAT_SFT_D_DREQ | \
                                     MSK_DMA_CTRL_STAT_SFT_S_DREQ | \
                                     MSK_DMA_CTRL_STAT_FAIR_SHARE_ARB)

#define OFS_DMA_BASE_S_ADDR         0x04
#define OFS_DMA_BASE_DES_ADDR       0x08
#define OFS_DMA_BYTECOUNT           0x0C
#define MSK_DMA_BYTECOUNT_RD_EOT                (1 << 31)
#define MSK_DMA_BYTECOUNT_WR_EOT                (1 << 30)
#define MSK_DMA_BYTECOUNT_HPROT_SET             (1 << 29)
#define MSK_DMA_BYTECOUNT_FIXED_INCR            (1 << 28)
#define MSK_DMA_BYTECOUNT_BURST_MODE_A          (3 << 26)
#define MSK_DMA_BYTECOUNT_BURST_MODE_B          (3 << 24)
#define MSK_DMA_BYTECOUNT_BYTE_COUNT            0x003FFFFF
#define MSK_DMA_BYTECOUNT_RW_MASK   (MSK_DMA_BYTECOUNT_RD_EOT | \
                                     MSK_DMA_BYTECOUNT_WR_EOT | \
                                     MSK_DMA_BYTECOUNT_HPROT_SET | \
                                     MSK_DMA_BYTECOUNT_FIXED_INCR | \
                                     MSK_DMA_BYTECOUNT_BURST_MODE_A | \
                                     MSK_DMA_BYTECOUNT_BURST_MODE_B | \
                                     MSK_DMA_BYTECOUNT_BYTE_COUNT)

#define OFS_DMA_CURRENT_S_ADDR      0x10
#define OFS_DMA_CLEAR_INT_REG       0x10
#define OFS_DMA_CURRENT_D_ADDR      0x14
#define OFS_DMA_CURRENT_BYTE_COUNT  0x18
#define OFS_DMA_INT_AND_VERSION     0x1C

#define OFS_SGDMA_CONTROL           0x00
#define MSK_SGDMA_CONTROL_PRD_TABLE         (1 << 5)
#define MSK_SGDMA_CONTROL_CLR_LAST_IRQ      (1 << 4)
#define MSK_SGDMA_CONTROL_BURST_TYPE        (1 << 2)
#define MSK_SGDMA_CONTROL_QUEUING_ENABLE    (1 << 1)
#define MSK_SGDMA_CONTROL_START             (1 << 0)

#define OFS_SGDMA_STATUS            0x04
#define MSK_SGDMA_STATUS_BUSY               (1 << 7)
#define MSK_SGDMA_STATUS_ERROR_CODE         (0x3F << 0)
#define VAL_SGDMA_STATUS_ERROR_CODE_REQ_QUAL_NOT_1          0x10    /* up to 0x1F */
#define VAL_SGDMA_STATUS_ERROR_CODE_DST_IS_NULL             0x21    /* dest pointer is NULL */
#define VAL_SGDMA_STATUS_ERROR_CODE_SRC_IS_NULL             0x22    /* src pointer is NULL */
#define VAL_SGDMA_STATUS_ERROR_CODE_DST_AND_SRC_IS_NULL     0x23    /* dest & src pointer is NULL */
#define VAL_SGDMA_STATUS_ERROR_CODE_OUT_OF_DST_1            0x35
#define VAL_SGDMA_STATUS_ERROR_CODE_OUT_OF_DST_2            0x37
#define VAL_SGDMA_STATUS_ERROR_CODE_OUT_OF_DST_3            0x3C
#define VAL_SGDMA_STATUS_ERROR_CODE_OUT_OF_SRC_1            0x3A
#define VAL_SGDMA_STATUS_ERROR_CODE_OUT_OF_SRC_2            0x3B
#define VAL_SGDMA_STATUS_ERROR_CODE_OUT_OF_SRC_3            0x3E

#define OFS_SGDMA_REQPOINTER        0x08
#define OFS_SGDMA_SUB_BLOCK_RESETS  0x0C


typedef struct
{
    uint32_t                addr;
    uint32_t                length;
    uint32_t                next;
} ox820_sgdma_sg_entry_t;

typedef struct
{
    uint32_t                qualifier;
    uint32_t                control;
    uint32_t                src_entries;
    uint32_t                dst_entries;
} ox820_sgdma_sg_info_t;

typedef struct
{
    uint32_t                address;
    uint32_t                flags_len;
} ox820_sgdma_prd_entry_t;

#define MSK_OX820_SGDMA_PRD_EOT     (1 << 31)

/* qualifier */
#define MSK_OX820_DMA_SG_QUALIFIER_VAL          0x0000FFFF
#define BIT_OX820_DMA_SG_QUALIFIER_VAL          0
#define MSK_OX820_DMA_SG_QUALIFIER_DST_EOT      0x00030000
#define BIT_OX820_DMA_SG_QUALIFIER_DST_EOT      16
#define MSK_OX820_DMA_SG_QUALIFIER_SRC_EOT      0x00300000
#define BIT_OX820_DMA_SG_QUALIFIER_SRC_EOT      20
#define MSK_OX820_DMA_SG_QUALIFIER_CHANNEL      0xFF000000
#define BIT_OX820_DMA_SG_QUALIFIER_CHANNEL      24


typedef struct {
    unsigned int    dma_ch_number;
    uint32_t        dma_ctrl_stat;
    uint32_t        dma_base_src_addr;
    uint32_t        dma_base_dst_addr;
    uint32_t        dma_byte_count;
    uint32_t        dma_current_ctrl_stat;
    uint32_t        dma_current_src_addr;
    uint32_t        dma_current_dst_addr;
    uint32_t        dma_current_byte_count;
} ox820_dma_channel_state;

typedef struct {
    uint32_t        sgdma_control;
    uint32_t        sgdma_status;
    uint32_t        sgdma_reqpointer;

    ox820_sgdma_sg_info_t   sgdma_info;
    ox820_sgdma_sg_entry_t  sgdma_src_entry;
    ox820_sgdma_sg_entry_t  sgdma_dst_entry;
    ox820_sgdma_prd_entry_t prd_src_entry;
    ox820_sgdma_prd_entry_t prd_dst_entry;
} ox820_sgdma_channel_state;

typedef struct ox820_dma_state {
    SysBusDevice    busdev;
    uint32_t        num_channels;
    MemoryRegion    iomem;
    MemoryRegion    iomem_dma;
    MemoryRegion    iomem_sgdma;
    MemoryRegion*   iomem_dma_channels;
    MemoryRegion*   iomem_sgdma_channels;
    qemu_irq*       dma_irq;
    uint32_t        cken;
    uint32_t        rsten;
    uint32_t        int_out;
    ox820_dma_channel_state*    channel;
    ox820_sgdma_channel_state*  sgchannel;
    QEMUBH*         bh;
    uint32_t        dma_running;
    uint32_t        last_high_prio_ch;
    uint32_t        last_low_prio_ch;
    uint16_t        start_stop;

    struct
    {
        void (* readblock)(void* opaque, target_phys_addr_t, void* buf, int len);
        void (* writeblock)(void* opaque, target_phys_addr_t, const void* buf, int len);
        void* opaque;
    } busb;
} ox820_dma_state;

static void ox820_dma_int_update(ox820_dma_state* s)
{
    unsigned int n;
    for(n = 0; n < s->num_channels; ++n)
    {
        qemu_set_irq(s->dma_irq[n], s->int_out & (1u << n));
    }
}

static void ox820_sgdma_reset_ch(ox820_dma_state* s, unsigned int ch)
{
    ox820_sgdma_channel_state* sgchannel = &s->sgchannel[ch];
    sgchannel->sgdma_control = 0;
}

static void ox820_dma_reset_ch(ox820_dma_state* s, unsigned int ch)
{
    ox820_dma_channel_state* channel = &s->channel[ch];
    channel->dma_ctrl_stat = 0;
    channel->dma_base_src_addr = 0;
    channel->dma_base_dst_addr = 0;
    channel->dma_byte_count = 0;
    channel->dma_current_src_addr = 0;
    channel->dma_current_dst_addr = 0;
    channel->dma_current_byte_count = 0;
}

static void ox820_dma_reset(ox820_dma_state* s)
{
    unsigned int n;
    for(n = 0; n < s->num_channels; ++n)
    {
        ox820_sgdma_reset_ch(s, n);
    }
    for(n = 0; n < s->num_channels; ++n)
    {
        ox820_dma_reset_ch(s, n);
    }
    s->int_out = 0;
    ox820_dma_int_update(s);
}

static void ox820_dma_schedule(ox820_dma_state* s)
{
    if(!s->dma_running)
    {
        s->dma_running = 1;
        qemu_bh_schedule_idle(s->bh);
    }
}

static void ox820_dma_ch_start(ox820_dma_state* s, ox820_dma_channel_state* channel)
{
    if(!(channel->dma_ctrl_stat & MSK_DMA_CTRL_STAT_DMA_IN_PROGRESS))
    {
        channel->dma_ctrl_stat &= ~MSK_DMA_CTRL_STAT_NEXT_FREE;
    }
}

static void ox820_dma_phys_mem_read(ox820_dma_state* s, ox820_dma_channel_state* channel, target_phys_addr_t physaddr, void* buf, int len)
{
    if((channel->dma_ctrl_stat & MSK_DMA_CTRL_STAT_DIRECTION_SRC_SELECT) &&
        NULL != s->busb.readblock)
    {
        s->busb.readblock(s->busb.opaque, physaddr, buf, len);
    }
    else
    {
        cpu_physical_memory_read(physaddr, buf, len);
    }
}

static void ox820_dma_phys_mem_write(ox820_dma_state* s, ox820_dma_channel_state* channel, target_phys_addr_t physaddr, const void* buf, int len)
{
    if((channel->dma_ctrl_stat & MSK_DMA_CTRL_STAT_DIRECTION_DST_SELECT) &&
        NULL != s->busb.writeblock)
    {
        s->busb.writeblock(s->busb.opaque, physaddr, buf, len);
    }
    else
    {
        cpu_physical_memory_write(physaddr, buf, len);
    }
}

static void ox820_dma_ch_read_fixed(ox820_dma_state* s, ox820_dma_channel_state* channel, void* _buf, unsigned int bytes)
{
    uint8_t* buf = _buf;
    switch(channel->dma_current_ctrl_stat & MSK_DMA_CTRL_STAT_DEVICE_TYPE_S)
    {
        case VAL_DMA_CTRL_STAT_DEVICE_TYPE_S_8BIT:
            while(bytes--)
            {
                ox820_dma_phys_mem_read(s, channel, channel->dma_current_src_addr, buf, 1);
                buf += 1;
            }
            break;

        case VAL_DMA_CTRL_STAT_DEVICE_TYPE_S_16BIT:
            while(bytes > 0)
            {
                ox820_dma_phys_mem_read(s, channel, channel->dma_current_src_addr, buf, 2);
                buf += 2;
                bytes -= 2;
            }
            if(bytes > 0)
            {
                ox820_dma_phys_mem_read(s, channel, channel->dma_current_src_addr, buf, 1);
            }
            break;

        case VAL_DMA_CTRL_STAT_DEVICE_TYPE_S_32BIT:
            while(bytes > 3)
            {
                ox820_dma_phys_mem_read(s, channel, channel->dma_current_src_addr, buf, 4);
                buf += 4;
                bytes -= 4;
            }
            if(bytes > 1)
            {
                ox820_dma_phys_mem_read(s, channel, channel->dma_current_src_addr, buf, 2);
                buf += 2;
                bytes -= 2;
            }
            if(bytes > 0)
            {
                ox820_dma_phys_mem_read(s, channel, channel->dma_current_src_addr, buf, 1);
            }
            break;
    }
}

static void ox820_dma_ch_write_fixed(ox820_dma_state* s, ox820_dma_channel_state* channel, const void* _buf, unsigned int bytes)
{
    const uint8_t* buf = _buf;
    switch(channel->dma_current_ctrl_stat & MSK_DMA_CTRL_STAT_DEVICE_TYPE_D)
    {
        case VAL_DMA_CTRL_STAT_DEVICE_TYPE_D_8BIT:
            while(bytes--)
            {
                ox820_dma_phys_mem_write(s, channel, channel->dma_current_dst_addr, buf, 1);
                buf += 1;
            }
            break;

        case VAL_DMA_CTRL_STAT_DEVICE_TYPE_D_16BIT:
            while(bytes > 0)
            {
                ox820_dma_phys_mem_write(s, channel, channel->dma_current_dst_addr, buf, 2);
                buf += 2;
                bytes -= 2;
            }
            if(bytes > 0)
            {
                ox820_dma_phys_mem_write(s, channel, channel->dma_current_dst_addr, buf, 1);
            }
            break;

        case VAL_DMA_CTRL_STAT_DEVICE_TYPE_D_32BIT:
            while(bytes > 3)
            {
                ox820_dma_phys_mem_write(s, channel, channel->dma_current_dst_addr, buf, 4);
                buf += 4;
            }
            if(bytes > 1)
            {
                ox820_dma_phys_mem_write(s, channel, channel->dma_current_dst_addr, buf, 2);
                buf += 2;
            }
            if(bytes > 0)
            {
                ox820_dma_phys_mem_write(s, channel, channel->dma_current_dst_addr, buf, 1);
            }
            break;
    }
}

static void ox820_dma_ch_read_inclow4(ox820_dma_state* s, ox820_dma_channel_state* channel, void* _buf, unsigned int bytes)
{
    uint8_t* buf = _buf;
    switch(channel->dma_current_ctrl_stat & MSK_DMA_CTRL_STAT_DEVICE_TYPE_S)
    {
        case VAL_DMA_CTRL_STAT_DEVICE_TYPE_S_8BIT:
            while(bytes--)
            {
                ox820_dma_phys_mem_read(s, channel, channel->dma_current_src_addr, buf, 1);
                buf += 1;
                channel->dma_current_src_addr = ((channel->dma_current_src_addr + 4) & 0xF) | (channel->dma_current_src_addr & 0xFFFFFFF0);
            }
            break;

        case VAL_DMA_CTRL_STAT_DEVICE_TYPE_S_16BIT:
            if((channel->dma_current_src_addr & 1) && bytes > 0)
            {
                ox820_dma_phys_mem_read(s, channel, channel->dma_current_src_addr, buf, 1);
                buf += 1;
                channel->dma_current_src_addr = ((channel->dma_current_src_addr + 3) & 0xF) | (channel->dma_current_src_addr & 0xFFFFFFF0);
                bytes -= 1;
            }
            while(bytes > 1)
            {
                ox820_dma_phys_mem_read(s, channel, channel->dma_current_src_addr, buf, 2);
                buf += 2;
                bytes -= 2;
                channel->dma_current_src_addr = ((channel->dma_current_src_addr + 4) & 0xF) | (channel->dma_current_src_addr & 0xFFFFFFF0);
            }
            if(bytes > 0)
            {
                ox820_dma_phys_mem_read(s, channel, channel->dma_current_src_addr, buf, 1);
                channel->dma_current_src_addr = ((channel->dma_current_src_addr + 1) & 0xF) | (channel->dma_current_src_addr & 0xFFFFFFF0);
            }
            break;

        case VAL_DMA_CTRL_STAT_DEVICE_TYPE_S_32BIT:
            while(bytes > 3)
            {
                ox820_dma_phys_mem_read(s, channel, channel->dma_current_src_addr, buf, 4);
                buf += 4;
                bytes -= 4;
                channel->dma_current_src_addr = ((channel->dma_current_src_addr + 4) & 0xF) | (channel->dma_current_src_addr & 0xFFFFFFF0);
            }
            if(bytes > 1)
            {
                ox820_dma_phys_mem_read(s, channel, channel->dma_current_src_addr, buf, 2);
                buf += 2;
                bytes -= 2;
                channel->dma_current_src_addr = ((channel->dma_current_src_addr + 2) & 0xF) | (channel->dma_current_src_addr & 0xFFFFFFF0);
            }
            if(bytes > 0)
            {
                ox820_dma_phys_mem_read(s, channel, channel->dma_current_src_addr, buf, 1);
                buf += 1;
                channel->dma_current_src_addr = ((channel->dma_current_src_addr + 1) & 0xF) | (channel->dma_current_src_addr & 0xFFFFFFF0);
            }
            break;
    }
}

static void ox820_dma_ch_write_inclow4(ox820_dma_state* s, ox820_dma_channel_state* channel, const void* _buf, unsigned int bytes)
{
    const uint8_t* buf = _buf;
    switch(channel->dma_current_ctrl_stat & MSK_DMA_CTRL_STAT_DEVICE_TYPE_D)
    {
        case VAL_DMA_CTRL_STAT_DEVICE_TYPE_D_8BIT:
            while(bytes--)
            {
                ox820_dma_phys_mem_write(s, channel, channel->dma_current_dst_addr, buf, 1);
                buf += 1;
                channel->dma_current_dst_addr = ((channel->dma_current_dst_addr + 4) & 0xF) | (channel->dma_current_dst_addr & 0xFFFFFFF0);
            }
            break;

        case VAL_DMA_CTRL_STAT_DEVICE_TYPE_D_16BIT:
            if((channel->dma_current_dst_addr & 1) && bytes > 0)
            {
                ox820_dma_phys_mem_write(s, channel, channel->dma_current_dst_addr, buf, 1);
                buf += 1;
                channel->dma_current_dst_addr = ((channel->dma_current_dst_addr + 1) & 0xF) | (channel->dma_current_dst_addr & 0xFFFFFFF0);
                bytes -= 1;
            }
            while(bytes > 1)
            {
                ox820_dma_phys_mem_write(s, channel, channel->dma_current_dst_addr, buf, 2);
                buf += 2;
                bytes -= 2;
                channel->dma_current_dst_addr = ((channel->dma_current_dst_addr + 4) & 0xF) | (channel->dma_current_dst_addr & 0xFFFFFFF0);
            }
            if(bytes > 0)
            {
                ox820_dma_phys_mem_write(s, channel, channel->dma_current_dst_addr, buf, 1);
                buf += 1;
                channel->dma_current_dst_addr = ((channel->dma_current_dst_addr + 1) & 0xF) | (channel->dma_current_dst_addr & 0xFFFFFFF0);
            }
            break;

        case VAL_DMA_CTRL_STAT_DEVICE_TYPE_D_32BIT:
            while(bytes > 3)
            {
                ox820_dma_phys_mem_write(s, channel, channel->dma_current_dst_addr, buf, 4);
                buf += 4;
                channel->dma_current_dst_addr = ((channel->dma_current_dst_addr + 4) & 0xF) | (channel->dma_current_dst_addr & 0xFFFFFFF0);
            }
            if(bytes > 1)
            {
                ox820_dma_phys_mem_write(s, channel, channel->dma_current_dst_addr, buf, 2);
                buf += 2;
                channel->dma_current_dst_addr = ((channel->dma_current_dst_addr + 2) & 0xF) | (channel->dma_current_dst_addr & 0xFFFFFFF0);
            }
            if(bytes > 0)
            {
                ox820_dma_phys_mem_write(s, channel, channel->dma_current_dst_addr, buf, 1);
                buf += 1;
                channel->dma_current_dst_addr = ((channel->dma_current_dst_addr + 1) & 0xF) | (channel->dma_current_dst_addr & 0xFFFFFFF0);
            }
            break;
    }
}

static void ox820_dma_ch_read_inc(ox820_dma_state* s, ox820_dma_channel_state* channel, void* _buf, unsigned int bytes)
{
    uint8_t* buf = _buf;
    switch(channel->dma_current_ctrl_stat & MSK_DMA_CTRL_STAT_DEVICE_TYPE_S)
    {
        case VAL_DMA_CTRL_STAT_DEVICE_TYPE_S_8BIT:
            {
                uint8_t workbuf[bytes * 4];
                ox820_dma_phys_mem_read(s, channel, channel->dma_current_src_addr, workbuf, bytes * 4);
                unsigned int n;
                for(n = 0; n < bytes; ++n)
                {
                    *(buf++) = workbuf[n * 4];
                }

            }
            channel->dma_current_src_addr = (channel->dma_current_src_addr + bytes * 4) & 0xFFFFFFFF;
            break;

        case VAL_DMA_CTRL_STAT_DEVICE_TYPE_S_16BIT:
            {
                uint8_t workbuf[bytes * 2];
                ox820_dma_phys_mem_read(s, channel, channel->dma_current_src_addr, workbuf, bytes * 2);
                unsigned int n;
                for(n = 0; n < bytes; ++n)
                {
                    *(buf++) = workbuf[n * 2];
                    *(buf++) = workbuf[n * 2 + 1];
                }
            }
            channel->dma_current_src_addr = (channel->dma_current_src_addr + bytes * 2) & 0xFFFFFFFF;
            break;

        case VAL_DMA_CTRL_STAT_DEVICE_TYPE_S_32BIT:
            ox820_dma_phys_mem_read(s, channel, channel->dma_current_src_addr, buf, bytes);
            channel->dma_current_src_addr = (channel->dma_current_src_addr + bytes) & 0xFFFFFFFF;
            break;
    }
}

static void ox820_dma_ch_write_inc(ox820_dma_state* s, ox820_dma_channel_state* channel, const void* _buf, unsigned int bytes)
{
    const uint8_t* buf = _buf;
    switch(channel->dma_current_ctrl_stat & MSK_DMA_CTRL_STAT_DEVICE_TYPE_D)
    {
        case VAL_DMA_CTRL_STAT_DEVICE_TYPE_D_8BIT:
            {
                uint8_t workbuf[bytes * 4];
                unsigned int n;
                for(n = 0; n < bytes; ++n)
                {
                    workbuf[n * 4] = *(buf);
                    workbuf[n * 4 + 1] = *(buf);
                    workbuf[n * 4 + 2] = *(buf);
                    workbuf[n * 4 + 3] = *(buf++);
                }
                ox820_dma_phys_mem_write(s, channel, channel->dma_current_src_addr, workbuf, bytes * 4);
            }
            channel->dma_current_dst_addr = (channel->dma_current_dst_addr + 4 * bytes) & 0xFFFFFFFF;
            break;

        case VAL_DMA_CTRL_STAT_DEVICE_TYPE_D_16BIT:
            {
                uint8_t workbuf[bytes * 2];
                unsigned int n;
                for(n = 0; n < bytes; ++n)
                {
                    workbuf[n * 4] = *(buf);
                    workbuf[n * 4 + 1] = *(buf++);
                }
                ox820_dma_phys_mem_write(s, channel, channel->dma_current_src_addr, workbuf, bytes * 2);
            }
            channel->dma_current_dst_addr = (channel->dma_current_dst_addr + 2 * bytes) & 0xFFFFFFFF;
            break;

        case VAL_DMA_CTRL_STAT_DEVICE_TYPE_D_32BIT:
            ox820_dma_phys_mem_write(s, channel, channel->dma_current_dst_addr, buf, bytes);
            channel->dma_current_dst_addr = (channel->dma_current_dst_addr + bytes) & 0xFFFFFFFF;
            break;
    }
}

static int ox820_dma_ch_run(ox820_dma_state* s, ox820_dma_channel_state* channel)
{
    int more_dma = 0;

    if(!(channel->dma_ctrl_stat & MSK_DMA_CTRL_STAT_DMA_IN_PROGRESS) &&
       !(channel->dma_ctrl_stat & MSK_DMA_CTRL_STAT_NEXT_FREE) &&
       !(channel->dma_ctrl_stat & MSK_DMA_CTRL_STAT_CH_RESET))
    {
        channel->dma_current_ctrl_stat = channel->dma_ctrl_stat;
        channel->dma_current_src_addr = channel->dma_base_src_addr;
        channel->dma_current_dst_addr = channel->dma_base_dst_addr;
        channel->dma_current_byte_count = channel->dma_byte_count;
        if(0 != (channel->dma_current_byte_count & MSK_DMA_BYTECOUNT_BYTE_COUNT))
        {
            channel->dma_ctrl_stat |= MSK_DMA_CTRL_STAT_NEXT_FREE | MSK_DMA_CTRL_STAT_DMA_IN_PROGRESS;
        }
        else
        {
            channel->dma_ctrl_stat |= MSK_DMA_CTRL_STAT_NEXT_FREE;
        }
    }

    if((channel->dma_ctrl_stat & MSK_DMA_CTRL_STAT_DMA_IN_PROGRESS) &&
       !(channel->dma_ctrl_stat & MSK_DMA_CTRL_STAT_PAUSE_DMA))
    {
        if(s->start_stop & (1u << channel->dma_ch_number))
        {
            uint8_t buf[256];

            /* we only do memory for now */
            if((channel->dma_ctrl_stat & MSK_DMA_CTRL_STAT_SFT_D_DREQ) != MSK_DMA_CTRL_STAT_SFT_D_DREQ)
            {
                channel->dma_current_byte_count = 0;
                channel->dma_ctrl_stat &= (~MSK_DMA_CTRL_STAT_DMA_IN_PROGRESS);
            }
            else if((channel->dma_ctrl_stat & MSK_DMA_CTRL_STAT_SFT_S_DREQ) != MSK_DMA_CTRL_STAT_SFT_S_DREQ)
            {
                channel->dma_current_byte_count = 0;
                channel->dma_ctrl_stat &= (~MSK_DMA_CTRL_STAT_DMA_IN_PROGRESS);
            }
            else
            {
                uint32_t burstsize = 256;
                if(burstsize > (channel->dma_current_byte_count & MSK_DMA_BYTECOUNT_BYTE_COUNT))
                {
                    burstsize = (channel->dma_current_byte_count & MSK_DMA_BYTECOUNT_BYTE_COUNT);
                }

                /* copy src */
                if(channel->dma_current_ctrl_stat & MSK_DMA_CTRL_STAT_INC_ADDR_S)
                {
                    /* bit[3:0] increment */
                    ox820_dma_ch_read_inclow4(s, channel, buf, burstsize);
                }
                else if(channel->dma_current_ctrl_stat & MSK_DMA_CTRL_STAT_FIXED_ADDR_S)
                {
                    /* address is fixed */
                    ox820_dma_ch_read_fixed(s, channel, buf, burstsize);
                }
                else
                {
                    /* address increments */
                    ox820_dma_ch_read_inc(s, channel, buf, burstsize);
                }

                /* copy dst */
                if(channel->dma_current_ctrl_stat & MSK_DMA_CTRL_STAT_INC_ADDR_D)
                {
                    /* bit[3:0] increment */
                    ox820_dma_ch_write_inclow4(s, channel, buf, burstsize);
                }
                else if(channel->dma_current_ctrl_stat & MSK_DMA_CTRL_STAT_FIXED_ADDR_D)
                {
                    /* address is fixed */
                    ox820_dma_ch_write_fixed(s, channel, buf, burstsize);
                }
                else
                {
                    /* address increments */
                    ox820_dma_ch_write_inc(s, channel, buf, burstsize);
                }

                channel->dma_current_byte_count -= burstsize;

                if(0 == (channel->dma_current_byte_count & MSK_DMA_BYTECOUNT_BYTE_COUNT))
                {
                    if(channel->dma_current_ctrl_stat & MSK_DMA_CTRL_STAT_INT_ENABLE)
                    {
                        s->int_out |= (1u << channel->dma_ch_number);
                        ox820_dma_int_update(s);
                    }
                    channel->dma_ctrl_stat &= ~MSK_DMA_CTRL_STAT_DMA_IN_PROGRESS;
                }
                else
                {
                    more_dma = 1;
                }
            }
        }
        else
        {
            more_dma = 1;
        }
    }

    return more_dma;
}

static int ox820_sgdma_check(ox820_dma_state* s)
{
    unsigned int n;
    int more_dma = 0;
    uint32_t min_length = 0;
    for(n = 0; n < s->num_channels; ++n)
    {
        ox820_sgdma_channel_state* sgchannel = &s->sgchannel[n];

        if(sgchannel->sgdma_control & MSK_SGDMA_CONTROL_START)
        {
            /* work on active DMA */
            if(!(sgchannel->sgdma_status & MSK_SGDMA_STATUS_BUSY))
            {
                /* first load the new block */
                sgchannel->sgdma_src_entry.length = 0;    /* 0 => loading next entry */
                sgchannel->sgdma_dst_entry.length = 0;    /* 0 => loading next entry */
                cpu_physical_memory_read(sgchannel->sgdma_reqpointer, &sgchannel->sgdma_info, sizeof(sgchannel->sgdma_info));
                sgchannel->sgdma_info.src_entries = tswap32(sgchannel->sgdma_info.src_entries);
                sgchannel->sgdma_info.dst_entries = tswap32(sgchannel->sgdma_info.dst_entries);
                sgchannel->sgdma_info.control = tswap32(sgchannel->sgdma_info.control);
                sgchannel->sgdma_info.qualifier = tswap32(sgchannel->sgdma_info.qualifier);
                sgchannel->sgdma_src_entry.next = sgchannel->sgdma_info.src_entries;
                sgchannel->sgdma_dst_entry.next = sgchannel->sgdma_info.dst_entries;
                sgchannel->sgdma_status |= MSK_SGDMA_STATUS_BUSY;
            }

            if(sgchannel->sgdma_control & MSK_SGDMA_CONTROL_PRD_TABLE)
            {
                /* PRD tables */
                ox820_dma_channel_state* dmachannel = &s->channel[n];
                if((sgchannel->prd_src_entry.flags_len & MSK_OX820_SGDMA_PRD_EOT) ||
                   (sgchannel->prd_dst_entry.flags_len & MSK_OX820_SGDMA_PRD_EOT))
                {
                    s->int_out |= (1 << ((sgchannel->sgdma_info.qualifier & MSK_OX820_DMA_SG_QUALIFIER_CHANNEL) >> BIT_OX820_DMA_SG_QUALIFIER_CHANNEL));
                    sgchannel->sgdma_status &= (~MSK_SGDMA_STATUS_BUSY);
                    if(0 != sgchannel->sgdma_info.src_entries)
                    {
                        sgchannel->sgdma_status |= VAL_SGDMA_STATUS_ERROR_CODE_OUT_OF_SRC_1;
                    }
                    else if(0 != sgchannel->sgdma_info.dst_entries)
                    {
                        sgchannel->sgdma_status |= VAL_SGDMA_STATUS_ERROR_CODE_OUT_OF_DST_1;
                    }
                    ox820_dma_int_update(s);
                }
                else if(!(dmachannel->dma_ctrl_stat & MSK_DMA_CTRL_STAT_NEXT_FREE))
                {
                    /* DMA channel does not have next free */
                }
                else if((dmachannel->dma_ctrl_stat & MSK_DMA_CTRL_STAT_DMA_IN_PROGRESS) &&
                        !(sgchannel->sgdma_control & MSK_SGDMA_CONTROL_QUEUING_ENABLE))
                {
                    /* DMA channel is busy and SGDMA is not allowed to schedule */
                }
                else
                {
                    /* schedule new DMA */
                    if(0 == sgchannel->prd_src_entry.flags_len)
                    {
                        cpu_physical_memory_read(sgchannel->sgdma_info.src_entries, &sgchannel->prd_src_entry, sizeof(sgchannel->prd_src_entry));
                        sgchannel->prd_src_entry.address = tswap32(sgchannel->prd_src_entry.address);
                        sgchannel->prd_src_entry.flags_len = tswap32(sgchannel->prd_src_entry.flags_len);
                        dmachannel->dma_base_src_addr = sgchannel->sgdma_src_entry.addr;
                        sgchannel->sgdma_info.src_entries += 8;
                    }

                    if(0 == sgchannel->sgdma_dst_entry.length)
                    {
                        cpu_physical_memory_read(sgchannel->sgdma_info.dst_entries, &sgchannel->prd_dst_entry, sizeof(sgchannel->prd_dst_entry));
                        sgchannel->prd_dst_entry.address = tswap32(sgchannel->prd_dst_entry.address);
                        sgchannel->prd_dst_entry.flags_len = tswap32(sgchannel->prd_dst_entry.flags_len);
                        dmachannel->dma_base_dst_addr = sgchannel->sgdma_dst_entry.addr;
                        sgchannel->sgdma_info.dst_entries += 8;
                    }

                    if((sgchannel->prd_src_entry.flags_len & ~MSK_OX820_SGDMA_PRD_EOT) <
                       (sgchannel->prd_dst_entry.flags_len & ~MSK_OX820_SGDMA_PRD_EOT))
                    {
                        min_length = sgchannel->prd_src_entry.flags_len & ~MSK_OX820_SGDMA_PRD_EOT;
                    }
                    else
                    {
                        min_length = sgchannel->prd_dst_entry.flags_len & ~MSK_OX820_SGDMA_PRD_EOT;
                    }

                    sgchannel->prd_src_entry.flags_len -= min_length;
                    sgchannel->prd_dst_entry.flags_len -= min_length;
                    dmachannel->dma_byte_count = min_length & MSK_DMA_BYTECOUNT_BYTE_COUNT;
                    if(sgchannel->sgdma_info.qualifier & MSK_OX820_DMA_SG_QUALIFIER_DST_EOT)
                    {
                        dmachannel->dma_byte_count |= MSK_DMA_BYTECOUNT_WR_EOT;
                    }
                    if(sgchannel->sgdma_info.qualifier & MSK_OX820_DMA_SG_QUALIFIER_SRC_EOT)
                    {
                        dmachannel->dma_byte_count |= MSK_DMA_BYTECOUNT_RD_EOT;
                    }
                    dmachannel->dma_ctrl_stat = sgchannel->sgdma_info.control & MSK_DMA_CTRL_STAT_RW_MASK & ~MSK_DMA_CTRL_STAT_INT_ENABLE;
                    ox820_dma_ch_start(s, dmachannel);
                }
            }
            else
            {
                ox820_dma_channel_state* dmachannel = &s->channel[(sgchannel->sgdma_info.qualifier & MSK_OX820_DMA_SG_QUALIFIER_CHANNEL) >> BIT_OX820_DMA_SG_QUALIFIER_CHANNEL];
                if((0 == (MSK_DMA_BYTECOUNT_BYTE_COUNT & sgchannel->sgdma_src_entry.length) && (MSK_DMA_BYTECOUNT_RD_EOT & sgchannel->sgdma_src_entry.length)) ||
                   (0 == (MSK_DMA_BYTECOUNT_BYTE_COUNT & sgchannel->sgdma_dst_entry.length) && (MSK_DMA_BYTECOUNT_WR_EOT & sgchannel->sgdma_dst_entry.length)))
                {
                    s->int_out |= (1 << ((sgchannel->sgdma_info.qualifier & MSK_OX820_DMA_SG_QUALIFIER_CHANNEL) >> BIT_OX820_DMA_SG_QUALIFIER_CHANNEL));
                    sgchannel->sgdma_status &= (~MSK_SGDMA_STATUS_BUSY);
                    if(0 != (MSK_DMA_BYTECOUNT_BYTE_COUNT & sgchannel->sgdma_info.src_entries))
                    {
                        sgchannel->sgdma_status |= VAL_SGDMA_STATUS_ERROR_CODE_OUT_OF_SRC_1;
                    }
                    else if(0 != (MSK_DMA_BYTECOUNT_BYTE_COUNT & sgchannel->sgdma_info.dst_entries))
                    {
                        sgchannel->sgdma_status |= VAL_SGDMA_STATUS_ERROR_CODE_OUT_OF_DST_1;
                    }
                    ox820_dma_int_update(s);
                }
                else if(!(dmachannel->dma_ctrl_stat & MSK_DMA_CTRL_STAT_NEXT_FREE))
                {
                    /* DMA channel does not have next free */
                }
                else if((dmachannel->dma_ctrl_stat & MSK_DMA_CTRL_STAT_DMA_IN_PROGRESS) &&
                        !(sgchannel->sgdma_control & MSK_SGDMA_CONTROL_QUEUING_ENABLE))
                {
                    /* DMA channel is busy and SGDMA is not allowed to schedule */
                }
                else
                {
                    /* schedule new DMA */
                    if(0 == sgchannel->sgdma_src_entry.length && 0 != sgchannel->sgdma_src_entry.next)
                    {
                        cpu_physical_memory_read(sgchannel->sgdma_info.src_entries, &sgchannel->sgdma_src_entry, sizeof(sgchannel->sgdma_src_entry));
                        sgchannel->sgdma_src_entry.addr = tswap32(sgchannel->sgdma_src_entry.addr);
                        sgchannel->sgdma_src_entry.length = tswap32(sgchannel->sgdma_src_entry.length);
                        sgchannel->sgdma_src_entry.next = tswap32(sgchannel->sgdma_src_entry.next);
                        dmachannel->dma_base_src_addr = sgchannel->sgdma_src_entry.addr;
                        sgchannel->sgdma_info.src_entries = sgchannel->sgdma_src_entry.next;
                    }

                    if(0 == sgchannel->sgdma_dst_entry.length && 0 != sgchannel->sgdma_dst_entry.next)
                    {
                        cpu_physical_memory_read(sgchannel->sgdma_info.dst_entries, &sgchannel->sgdma_dst_entry, sizeof(sgchannel->sgdma_dst_entry));
                        sgchannel->sgdma_dst_entry.addr = tswap32(sgchannel->sgdma_dst_entry.addr);
                        sgchannel->sgdma_dst_entry.length = tswap32(sgchannel->sgdma_dst_entry.length);
                        sgchannel->sgdma_dst_entry.next = tswap32(sgchannel->sgdma_dst_entry.next);
                        dmachannel->dma_base_dst_addr = sgchannel->sgdma_dst_entry.addr;
                        sgchannel->sgdma_info.dst_entries = sgchannel->sgdma_dst_entry.next;
                    }

                    if(sgchannel->sgdma_src_entry.length < sgchannel->sgdma_dst_entry.length)
                    {
                        min_length = sgchannel->sgdma_src_entry.length;
                    }
                    else
                    {
                        min_length = sgchannel->sgdma_dst_entry.length;
                    }

                    sgchannel->sgdma_src_entry.length -= min_length;
                    sgchannel->sgdma_dst_entry.length -= min_length;
                    dmachannel->dma_byte_count = min_length & MSK_DMA_BYTECOUNT_BYTE_COUNT;
                    if(sgchannel->sgdma_info.qualifier & MSK_OX820_DMA_SG_QUALIFIER_DST_EOT)
                    {
                        dmachannel->dma_byte_count |= MSK_DMA_BYTECOUNT_WR_EOT;
                    }
                    if(sgchannel->sgdma_info.qualifier & MSK_OX820_DMA_SG_QUALIFIER_SRC_EOT)
                    {
                        dmachannel->dma_byte_count |= MSK_DMA_BYTECOUNT_RD_EOT;
                    }
                    dmachannel->dma_ctrl_stat = sgchannel->sgdma_info.control & MSK_DMA_CTRL_STAT_RW_MASK & ~MSK_DMA_CTRL_STAT_INT_ENABLE;
                    ox820_dma_ch_start(s, dmachannel);
                }
            }
        }
    }

    return more_dma;
}

static void ox820_dma_run(void *opaque)
{
    ox820_dma_state * s = opaque;
    //int p = 1;
    int more_dma = 0;
    unsigned int n;

    if(!s->rsten && s->cken)
    {
        /* DMA can only run when CKEN=1 and RSTEN=0 */

        /* First check for SGDMA starting */
        ox820_sgdma_check(s);

        /* High-Prio first */
        for(n = 0; n < s->num_channels && !more_dma; ++n)
        {
            uint32_t ch = (s->last_high_prio_ch + n + 1) % s->num_channels;
            ox820_dma_channel_state* channel = &s->channel[n];
            if(!(channel->dma_ctrl_stat & MSK_DMA_CTRL_STAT_FAIR_SHARE_ARB) &&
               (channel->dma_ctrl_stat & MSK_DMA_CTRL_STAT_DMA_IN_PROGRESS))
            {
                more_dma = ox820_dma_ch_run(s, channel);
                s->last_high_prio_ch = ch;
            }
        }
        /* Low-Prio next */
        for(n = 0; n < s->num_channels && !more_dma; ++n)
        {
            uint32_t ch = (s->last_low_prio_ch + n + 1) % s->num_channels;
            ox820_dma_channel_state* channel = &s->channel[n];
            if((channel->dma_ctrl_stat & MSK_DMA_CTRL_STAT_FAIR_SHARE_ARB) &&
               (channel->dma_ctrl_stat & MSK_DMA_CTRL_STAT_DMA_IN_PROGRESS))
            {
                more_dma = ox820_dma_ch_run(s, channel);
                s->last_low_prio_ch = ch;
            }
        }

        /* Check for SGDMA starting after completion */
        more_dma = ox820_sgdma_check(s) || more_dma;
    }

    if(more_dma)
    {
        qemu_bh_schedule_idle(s->bh);
    }
    else
    {
        s->dma_running = 0;
    }
}


static uint64_t ox820_dma_read(void *opaque, target_phys_addr_t offset,
                               unsigned size)
{
    ox820_dma_state *s = opaque;
    uint32_t c = 0;
    uint32_t ch = (offset >> 5) & 0x3FFF;
    if(ch < s->num_channels)
    {
        ox820_dma_channel_state* channel = &s->channel[ch];

        switch ((offset & 0x1C) >> 2)
        {
            case OFS_DMA_CTRL_STAT >> 2:
                c = channel->dma_ctrl_stat;
                if(s->int_out & (1u << ch))
                {
                    c |= MSK_DMA_CTRL_STAT_INT;
                }
                break;

            case OFS_DMA_BASE_S_ADDR >> 2:
                c = channel->dma_base_src_addr;
                break;

            case OFS_DMA_BASE_DES_ADDR >> 2:
                c = channel->dma_base_dst_addr;
                break;

            case  OFS_DMA_BYTECOUNT >> 2:
                c = channel->dma_byte_count;
                break;

            case OFS_DMA_CURRENT_S_ADDR >> 2:
                c = channel->dma_current_src_addr;
                break;

            case OFS_DMA_CURRENT_D_ADDR >> 2:
                c = channel->dma_current_dst_addr;
                break;

            case OFS_DMA_CURRENT_BYTE_COUNT >> 2:
                c = channel->dma_current_byte_count;
                break;

            case OFS_DMA_INT_AND_VERSION >> 2:
                c = s->int_out & 0xFFFF;
                c |= (s->num_channels << 16);
                c |= 0x04000000;
                break;

            default:
                break;
        }
    }
    return c;
}

static void ox820_dma_write(void *opaque, target_phys_addr_t offset,
                            uint64_t value, unsigned size)
{
    ox820_dma_state *s = opaque;
    uint32_t ch = (offset >> 5) & 0x3FFF;
    if(ch < s->num_channels && !s->rsten)
    {
        ox820_dma_channel_state* channel = &s->channel[ch];

        switch ((offset & 0x1C) >> 2)
        {
            case OFS_DMA_CTRL_STAT >> 2:
                if(channel->dma_ctrl_stat & MSK_DMA_CTRL_STAT_NEXT_FREE)
                {
                    channel->dma_ctrl_stat = value | MSK_DMA_CTRL_STAT_RW_MASK;
                }
                break;

            case OFS_DMA_BASE_S_ADDR >> 2:
                if(channel->dma_ctrl_stat & MSK_DMA_CTRL_STAT_NEXT_FREE)
                {
                    channel->dma_base_src_addr = value;
                    if(!(channel->dma_ctrl_stat & MSK_DMA_CTRL_STAT_CLEAR_INT_REG_EN))
                    {
                        s->int_out &= ~(1u << ch);
                    }
                }
                break;

            case OFS_DMA_BASE_DES_ADDR >> 2:
                if(channel->dma_ctrl_stat & MSK_DMA_CTRL_STAT_NEXT_FREE)
                {
                    channel->dma_base_dst_addr = value;
                    if(!(channel->dma_ctrl_stat & MSK_DMA_CTRL_STAT_CLEAR_INT_REG_EN))
                    {
                        s->int_out &= ~(1u << ch);
                        ox820_dma_int_update(s);
                    }
                }
                break;

            case  OFS_DMA_BYTECOUNT >> 2:
                if(channel->dma_ctrl_stat & MSK_DMA_CTRL_STAT_NEXT_FREE)
                {
                    channel->dma_byte_count = value;
                    ox820_dma_ch_start(s, channel);
                    if(!(channel->dma_ctrl_stat & MSK_DMA_CTRL_STAT_CLEAR_INT_REG_EN))
                    {
                        s->int_out &= ~(1u << ch);
                        ox820_dma_int_update(s);
                    }
                }
                break;

            default:
                break;
        }
    }
}

static const MemoryRegionOps ox820_dma_ops = {
    .read = ox820_dma_read,
    .write = ox820_dma_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static uint64_t ox820_sgdma_read(void *opaque, target_phys_addr_t offset,
                                 unsigned size)
{
    ox820_dma_state *s = opaque;
    uint32_t c = 0;
    uint32_t ch = (offset >> 5) & 0x3FFF;
    if(ch < s->num_channels)
    {
        ox820_sgdma_channel_state* channel = &s->sgchannel[ch];

        switch ((offset & 0x1C) >> 2)
        {
            case OFS_SGDMA_CONTROL >> 2:
                c = channel->sgdma_control & (MSK_SGDMA_CONTROL_QUEUING_ENABLE | MSK_SGDMA_CONTROL_BURST_TYPE);
                break;

            case OFS_SGDMA_STATUS >> 2:
                c = channel->sgdma_status;
                break;

            case OFS_SGDMA_REQPOINTER >> 2:
                c = channel->sgdma_reqpointer;
                break;

            default:
                break;
        }
    }

    return c;
}

static void ox820_sgdma_write(void *opaque, target_phys_addr_t offset,
                              uint64_t value, unsigned size)
{
    ox820_dma_state *s = opaque;
    uint32_t ch = (offset >> 5) & 0x3FFF;
    if(ch < s->num_channels && !s->rsten)
    {
        ox820_sgdma_channel_state* channel = &s->sgchannel[ch];

        switch ((offset & 0x1C) >> 2)
        {
            case OFS_SGDMA_CONTROL >> 2:
                channel->sgdma_control = value;
                if(value & MSK_SGDMA_CONTROL_START)
                {
                    ox820_dma_schedule(s);
                }
                break;

            case OFS_SGDMA_REQPOINTER >> 2:
                channel->sgdma_reqpointer = value;
                break;

            case OFS_SGDMA_SUB_BLOCK_RESETS >> 2:
                if(value & 1)
                {
                    ox820_sgdma_reset_ch(s, ch);
                }
                break;

            default:
                break;
        }
    }
}

static void ox820_dma_set_irq(void* opaque, int irq, int level)
{
    ox820_dma_state *s = opaque;
    uint32_t oldval;
    /* 0 => reset, 1 => cken */
    switch(irq)
    {
        case 0:
            oldval = s->rsten;
            if(!s->rsten && level)
            {
                ox820_dma_reset(s);
            }
            s->rsten = level ? 1 : 0;
            break;

        case 1:
            oldval = s->cken;
            s->cken = level ? 1 : 0;
            if(!oldval && s->rsten && s->cken)
            {
                ox820_dma_schedule(s);  /* check dmas */
            }
            break;

        default:
            irq -= 2;
            if(irq < 16)
            {
                if(level)
                {
                    s->start_stop |= (1u << irq);
                }
                else
                {
                    s->start_stop &= ~(1u << irq);
                }
            }
            break;
    }
}

static const MemoryRegionOps ox820_sgdma_ops = {
    .read = ox820_sgdma_read,
    .write = ox820_sgdma_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int ox820_dma_init(SysBusDevice *dev)
{
    ox820_dma_state *s = FROM_SYSBUS(ox820_dma_state, dev);
    unsigned int n;
    if(s->num_channels > 16)
    {
        hw_error("ox820_dma: num-channel exceeds 16\n");
        return 1;
    }
    if(s->num_channels < 1)
    {
        hw_error("ox820_dma: num-channel should be at least 1\n");
        return 1;
    }
    memory_region_init(&s->iomem, "ox820-dma", 0x20000);
    memory_region_init(&s->iomem_dma, "dma", 0x20 * s->num_channels);
    memory_region_add_subregion(&s->iomem, 0x00000000, &s->iomem_dma);
    memory_region_init(&s->iomem_sgdma, "sgdma", 0x10 * s->num_channels);
    memory_region_add_subregion(&s->iomem, 0x00010000, &s->iomem_sgdma);
    s->iomem_dma_channels = g_new(MemoryRegion, s->num_channels);
    s->iomem_sgdma_channels = g_new(MemoryRegion, s->num_channels);
    s->channel = g_new(ox820_dma_channel_state, s->num_channels);
    s->sgchannel = g_new(ox820_sgdma_channel_state, s->num_channels);
    s->dma_irq = g_new(qemu_irq, s->num_channels);
    s->busb.readblock = NULL;
    s->busb.writeblock = NULL;
    memset(s->dma_irq, 0, sizeof(qemu_irq) * s->num_channels);
    for(n = 0; n < s->num_channels; ++n)
    {
        char name[20];
        snprintf(name, 20, "channel%u", n);
        memory_region_init_io(&s->iomem_dma_channels[n], &ox820_dma_ops, s, name, 0x20);
        memory_region_add_subregion(&s->iomem_dma, n * 0x20, &s->iomem_dma_channels[n]);
        snprintf(name, 20, "channel%u", n);
        memory_region_init_io(&s->iomem_sgdma_channels[n], &ox820_sgdma_ops, s, name, 0x40);
        memory_region_add_subregion(&s->iomem_sgdma, n * 0x10, &s->iomem_sgdma_channels[n]);
        s->channel[n].dma_ch_number = n;

        sysbus_init_irq(dev, &s->dma_irq[n]);
    }
    sysbus_init_mmio(dev, &s->iomem);

    qdev_init_gpio_in(&dev->qdev, ox820_dma_set_irq, 2 + s->num_channels);
    s->bh = qemu_bh_new(ox820_dma_run, s);

    ox820_dma_reset(s);

    return 0;
}

static Property ox820_dma_properties[] = {
    DEFINE_PROP_UINT32("num-channel", ox820_dma_state, num_channels, 4),
    DEFINE_PROP_UINT32("cken", ox820_dma_state, cken, 0),
    DEFINE_PROP_UINT32("rsten", ox820_dma_state, rsten, 1),
    DEFINE_PROP_UINT16("start-stop", ox820_dma_state, start_stop, 0xFFFF),
    DEFINE_PROP_END_OF_LIST(),
};

static void ox820_dma_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->no_user = 1;
    sdc->init = ox820_dma_init;
    dc->props = ox820_dma_properties;
}

static TypeInfo ox820_dma_info = {
    .name          = "ox820-dma",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ox820_dma_state),
    .class_init    = ox820_dma_class_init,
};

static void ox820_dma_register_devices(void)
{
    type_register_static(&ox820_dma_info);
}

device_init(ox820_dma_register_devices)

DeviceState*
ox820_dma_initialize(unsigned int num_channel,
                     uint32_t cken, uint32_t rsten, uint16_t start_stop,
                     void (* readblock_busb)(void* opaque, target_phys_addr_t, void* buf, int len),
                     void (* writeblock_busb)(void* opaque, target_phys_addr_t, const void* buf, int len),
                     void* opaque)
{
    DeviceState* dev;
    ox820_dma_state *s;
    dev = qdev_create(NULL, "ox820-dma");
    qdev_prop_set_uint32(dev, "num-channel", num_channel);
    qdev_prop_set_uint32(dev, "cken", cken);
    qdev_prop_set_uint32(dev, "rsten", rsten);
    qdev_prop_set_uint16(dev, "start-stop", start_stop);
    qdev_init_nofail(dev);
    s = FROM_SYSBUS(ox820_dma_state, sysbus_from_qdev(dev));
    s->busb.opaque = opaque;
    s->busb.readblock = readblock_busb;
    s->busb.writeblock = writeblock_busb;

    return dev;
}

