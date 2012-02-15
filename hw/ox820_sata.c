/*
 * ox820-sata unit
 *
 * Written by Sven Bormann
 *
 * This code is licensed under the GPL.
 */

#include "sysbus.h"
#include "ox820_dma.h"
#include <zlib.h>

#define OFS_SATA_PORT_ORB1              0x0000
#define OFS_SATA_PORT_ORB2              0x0004
#define OFS_SATA_PORT_ORB3              0x0008
#define OFS_SATA_PORT_ORB4              0x000C
#define OFS_SATA_PORT_ORB5              0x0010
#define OFS_SATA_PORT_FIS_CTRL          0x0018
#define OFS_SATA_PORT_FIS_DATA          0x001C
#define OFS_SATA_PORT_RBC1              0x0020
#define OFS_SATA_PORT_RBC2              0x0024
#define OFS_SATA_PORT_RBC3              0x0028
#define OFS_SATA_PORT_RBC4              0x002C
#define OFS_SATA_PORT_INT_CTRL_STATUS   0x0030
#define OFS_SATA_PORT_INT_ENABLE_SET    0x0034
#define OFS_SATA_PORT_INT_ENABLE_CLEAR  0x0038
#define OFS_SATA_PORT_VERSION           0x003C
#define OFS_SATA_PORT_SACTIVE           0x004C
#define OFS_SATA_PORT_ACTIVE_TAG        0x0050
#define OFS_SATA_PORT_SATA_CTRL         0x005C
#define OFS_SATA_PORT_COMMAND           0x0060
#define OFS_SATA_PORT_PORT_CTRL         0x0068
#define OFS_SATA_PORT_DRIVE_CTRL        0x006C
#define OFS_SATA_PORT_LINK_ASYNC_DATA   0x0070
#define OFS_SATA_PORT_LINK_ASYNC_R_ADDR 0x0074
#define OFS_SATA_PORT_LINK_ASYNC_W_ADDR 0x0078
#define OFS_SATA_PORT_LINK_ASYNC_CTRL   0x007C
#define OFS_SATA_PORT_WIN_LOW_3100      0x0080
#define OFS_SATA_PORT_WIN_LOW_4732      0x0084
#define OFS_SATA_PORT_WIN_HIGH_3100     0x0088
#define OFS_SATA_PORT_WIN_HIGH_4732     0x008C
#define OFS_SATA_PORT_WIN_CTRL_ZERO     0x0090
#define OFS_SATA_PORT_WIN_CTRL_ONE      0x0094
#define OFS_SATA_PORT_WIN_CTRL_TWO      0x0098
#define OFS_SATA_PORT_PORT_CTRL2        0x00A0
#define OFS_SATA_PORT_PIO_SIZE          0x00A8
#define OFS_SATA_PORT_BACKUP1           0x00B0
#define OFS_SATA_PORT_BACKUP2           0x00B4
#define OFS_SATA_PORT_BACKUP3           0x00B8
#define OFS_SATA_PORT_BACKUP4           0x00BC
#define OFS_SATA_PORT_MSE_INFO1         0x00D0
#define OFS_SATA_PORT_MSE_INFO2         0x00D4
#define OFS_SATA_PORT_MSE_INFO3         0x00D8


/* sata port interrupts */
#define MSK_SATA_PORT_INT_WIN_CLIP              (1 << 13)
#define MSK_SATA_PORT_INT_WIN_DISALLOW          (1 << 12)
#define MSK_SATA_PORT_INT_BIST_FIS_RCVD         (1 << 11)
#define MSK_SATA_PORT_INT_REG_ACCESS_ERROR      (1 << 7)
#define MSK_SATA_PORT_INT_LINK_IRQ              (1 << 3)
#define MSK_SATA_PORT_INT_SATA_CMD_ERROR        (1 << 2)
#define MSK_SATA_PORT_INT_LINK_SERROR           (1 << 1)
#define MSK_SATA_PORT_INT_END_OF_CMD            (1 << 0)


/* core regs */
#define OFS_SATA_CORE_DATA_MUX_DEBUG_1          0x0000
#define OFS_SATA_CORE_DATA_COUNT_PORT_0         0x0010
#define OFS_SATA_CORE_INTERRUPT_STATUS          0x0030
#define OFS_SATA_CORE_INTERRUPT_ENABLE_SET      0x0034
#define OFS_SATA_CORE_INTERRUPT_ENABLE_CLEAR    0x0038
#define OFS_SATA_CORE_CONTROL                   0x005C
#define OFS_SATA_CORE_AHB_JBOD_DEBUG            0x0060
#define OFS_SATA_CORE_DISK_SIZE_LOWER           0x0070
#define OFS_SATA_CORE_DISK_SIZE_UPPER           0x0074
#define OFS_SATA_CORE_PORT_ERROR_MASK           0x0078
#define OFS_SATA_CORE_IDLE_STATUS               0x007C
#define OFS_SATA_CORE_DM_DEBUG_PTR0             0x0080
#define OFS_SATA_CORE_DATA_PLANE_CTRL           0x00AC
#define OFS_SATA_CORE_CONTEXT_MASK              0x00B4
#define OFS_SATA_CORE_DATA_PLANE_STAT           0x00B8


/* core ints */
#define MSK_SATA_CORE_INT_RAID                  0x00008000
#define MSK_SATA_CORE_INT_ATA_SGDMA_IDLE        0x00000F00
#define MSK_SATA_CORE_INT_PORT1                 0x00000002
#define MSK_SATA_CORE_INT_PORT0                 0x00000001

typedef struct {
    uint32_t        orb1;
    uint32_t        orb2;
    uint32_t        orb3;
    uint32_t        orb4;
    uint32_t        orb5;
    uint32_t        fis_ctrl;
    uint32_t        rbc1;
    uint32_t        rbc2;
    uint32_t        rbc3;
    uint32_t        rbc4;
    uint32_t        int_status;
    uint32_t        int_enable;
    uint32_t        sactive;
    uint32_t        active_tag;
    uint32_t        command;
    uint32_t        port_ctrl;
    uint32_t        drive_ctrl;
    uint32_t        link_async_data;
    uint32_t        link_async_r_addr;
    uint32_t        link_async_w_addr;
    uint32_t        link_async_ctrl;
    uint32_t        win_low_3100;
    uint32_t        win_low_4732;
    uint32_t        win_high_3100;
    uint32_t        win_high_4732;
    uint32_t        win_ctrl_zero;
    uint32_t        win_ctrl_one;
    uint32_t        win_ctrl_two;
    uint32_t        port_ctrl2;
    uint32_t        pio_size;
    uint32_t        backup1;
    uint32_t        backup2;
    uint32_t        backup3;
    uint32_t        backup4;
    uint32_t        mse_info1;
    uint32_t        mse_info2;
    uint32_t        mse_info3;
} ox820_sata_port;

typedef struct {
    uint32_t        orb1;
    uint32_t        orb2;
    uint32_t        orb3;
    uint32_t        orb4;
    uint32_t        orb5;
    uint32_t        rbc1;
    uint32_t        rbc2;
    uint32_t        rbc3;
    uint32_t        rbc4;
    uint32_t        int_status;
    uint32_t        int_enable;
    uint32_t        command;
    uint32_t        win_low_3100;
    uint32_t        win_low_4732;
    uint32_t        win_high_3100;
    uint32_t        win_high_4732;
    uint32_t        win_ctrl_zero;
    uint32_t        win_ctrl_one;
    uint32_t        win_ctrl_two;
    uint32_t        backup1;
    uint32_t        backup2;
    uint32_t        backup3;
    uint32_t        backup4;
} ox820_sata_raid;

typedef struct {
    uint32_t        data_mux_debug1;
    uint32_t        data_count_port0;
    uint32_t        int_status;
    uint32_t        int_enable;
    uint32_t        core_control;
    uint32_t        ahb_jbod_debug;
    uint32_t        disk_size_lower;
    uint32_t        disk_size_upper;
    uint32_t        port_error_mask;
    uint32_t        idle_status;
    uint32_t        dm_debug_ptr0;
    uint32_t        data_plane_ctrl;
    uint32_t        context_mask;
    uint32_t        data_plane_stat;
} ox820_sata_core;

typedef struct {
    SysBusDevice    busdev;
    MemoryRegion    iomem;
    MemoryRegion    iomem_port0;
    MemoryRegion    iomem_port1;
    MemoryRegion    iomem_ciph;
    MemoryRegion    iomem_core;
    MemoryRegion    iomem_raid;
    uint32_t        cken;
    uint32_t        rsten;
    uint32_t        num_port;
    qemu_irq        irq;
    qemu_irq        dma_irq[2];
    uint32_t        ucode_type;
    union
    {
        uint32_t        dword[0x100];
        uint8_t         byte[0x400];
    } ucode;
    ox820_sata_port port[2];
    ox820_sata_core core;
    ox820_sata_raid raid_port;
} ox820_sata_state;

enum ox820_sata_ucode_t
{
    OX820_SATA_UCODE_UNKNOWN,
    OX820_SATA_UCODE_JBOD,
    OX820_SATA_UCODE_RAID
};

static enum ox820_sata_ucode_t __attribute__((used)) ox820_sata_identify_ucode(ox820_sata_state* s)
{
    uint32_t crc = crc32(~0, s->ucode.byte, 0x2b4);
    switch(crc)
    {
        case 0xa83cb8bd:
            return OX820_SATA_UCODE_JBOD;
        case 0x33788817:
            return OX820_SATA_UCODE_RAID;
        default:
            return OX820_SATA_UCODE_UNKNOWN;
    }
}

static uint64_t ox820_sata_port_read(ox820_sata_state* s , ox820_sata_port* port,
                                     target_phys_addr_t offset, unsigned size)
{
    uint32_t c = 0;

    offset &= 0xFFFF;
    switch(offset >> 2)
    {
        case OFS_SATA_PORT_ORB1 >> 2:
            c = port->orb1;
            break;

        case OFS_SATA_PORT_ORB2 >> 2:
            c = port->orb2;
            break;

        case OFS_SATA_PORT_ORB3 >> 2:
            c = port->orb3;
            break;

        case OFS_SATA_PORT_ORB4 >> 2:
            c = port->orb4;
            break;

        case OFS_SATA_PORT_ORB5 >> 2:
            c = port->orb5;
            break;

        case OFS_SATA_PORT_FIS_CTRL >> 2:
            c = port->fis_ctrl;
            break;

        case OFS_SATA_PORT_FIS_DATA >> 2:
            break;

        case OFS_SATA_PORT_RBC1 >> 2:
            c = port->rbc1;
            break;

        case OFS_SATA_PORT_RBC2 >> 2:
            c = port->rbc2;
            break;

        case OFS_SATA_PORT_RBC3 >> 2:
            c = port->rbc3;
            break;

        case OFS_SATA_PORT_RBC4 >> 2:
            c = port->rbc4;
            break;

        case OFS_SATA_PORT_INT_CTRL_STATUS >> 2:
            c = (port->int_status & port->int_enable) | (port->int_status << 16);
            break;

        case OFS_SATA_PORT_INT_ENABLE_SET >> 2:
            break;

        case OFS_SATA_PORT_INT_ENABLE_CLEAR >> 2:
            break;

        case OFS_SATA_PORT_VERSION >> 2:
            c = 0x1f4;
            break;

        case OFS_SATA_PORT_SACTIVE >> 2:
            c = port->sactive;
            break;

        case OFS_SATA_PORT_ACTIVE_TAG >> 2:
            c = port->active_tag;
            break;

        case OFS_SATA_PORT_SATA_CTRL >> 2:
            break;

        case OFS_SATA_PORT_COMMAND >> 2:
            c = port->command;
            break;

        case OFS_SATA_PORT_PORT_CTRL >> 2:
            c = port->port_ctrl;
            break;

        case OFS_SATA_PORT_DRIVE_CTRL >> 2:
            c = port->drive_ctrl;
            break;

        case OFS_SATA_PORT_LINK_ASYNC_DATA >> 2:
            c = port->link_async_data;
            break;

        case OFS_SATA_PORT_LINK_ASYNC_R_ADDR >> 2:
            c = port->link_async_r_addr;
            break;

        case OFS_SATA_PORT_LINK_ASYNC_W_ADDR >> 2:
            c = port->link_async_w_addr;
            break;

        case OFS_SATA_PORT_LINK_ASYNC_CTRL >> 2:
            c = 1;
            break;

        case OFS_SATA_PORT_WIN_LOW_3100 >> 2:
            c = port->win_low_3100;
            break;

        case OFS_SATA_PORT_WIN_LOW_4732 >> 2:
            c = port->win_low_4732;
            break;

        case OFS_SATA_PORT_WIN_HIGH_3100 >> 2:
            c = port->win_high_3100;
            break;

        case OFS_SATA_PORT_WIN_HIGH_4732 >> 2:
            c = port->win_high_4732;
            break;

        case OFS_SATA_PORT_WIN_CTRL_ZERO >> 2:
            c = port->win_ctrl_zero;
            break;

        case OFS_SATA_PORT_WIN_CTRL_ONE >> 2:
            c = port->win_ctrl_one;
            break;

        case OFS_SATA_PORT_WIN_CTRL_TWO >> 2:
            c = port->win_ctrl_two;
            break;

        case OFS_SATA_PORT_PORT_CTRL2 >> 2:
            c = port->port_ctrl2;
            break;

        case OFS_SATA_PORT_PIO_SIZE >> 2:
            c = port->pio_size;
            break;

        case OFS_SATA_PORT_BACKUP1 >> 2:
            c = port->backup1;
            break;

        case OFS_SATA_PORT_BACKUP2 >> 2:
            c = port->backup2;
            break;

        case OFS_SATA_PORT_BACKUP3 >> 2:
            c = port->backup3;
            break;

        case OFS_SATA_PORT_BACKUP4 >> 2:
            c = port->backup4;
            break;

        case OFS_SATA_PORT_MSE_INFO1 >> 2:
            c = port->mse_info1;
            break;

        case OFS_SATA_PORT_MSE_INFO2 >> 2:
            c = port->mse_info2;
            break;

        case OFS_SATA_PORT_MSE_INFO3 >> 2:
            c = port->mse_info3;
            break;
    }
    return c;
}

static void ox820_sata_port_write(ox820_sata_state* s, ox820_sata_port* port,
                                  target_phys_addr_t offset, uint64_t value, unsigned size)
{
    offset &= 0xFFFF;
    switch(offset >> 2)
    {
        case OFS_SATA_PORT_ORB1 >> 2:
            port->orb1 = value;
            break;

        case OFS_SATA_PORT_ORB2 >> 2:
            port->orb2 = value;
            break;

        case OFS_SATA_PORT_ORB3 >> 2:
            port->orb3 = value;
            break;

        case OFS_SATA_PORT_ORB4 >> 2:
            port->orb4 = value;
            break;

        case OFS_SATA_PORT_ORB5 >> 2:
            port->orb5 = value;
            break;

        case OFS_SATA_PORT_FIS_CTRL >> 2:
            port->fis_ctrl = value;
            break;

        case OFS_SATA_PORT_FIS_DATA >> 2:
            break;

        case OFS_SATA_PORT_RBC1 >> 2:
            port->rbc1 = value;
            break;

        case OFS_SATA_PORT_RBC2 >> 2:
            port->rbc2 = value;
            break;

        case OFS_SATA_PORT_RBC3 >> 2:
            port->rbc3 = value;
            break;

        case OFS_SATA_PORT_RBC4 >> 2:
            port->rbc4 = value;
            break;

        case OFS_SATA_PORT_INT_CTRL_STATUS >> 2:
            port->int_status &= ~value;
            break;

        case OFS_SATA_PORT_INT_ENABLE_SET >> 2:
            port->int_enable |= (value & 0x388F);
            break;

        case OFS_SATA_PORT_INT_ENABLE_CLEAR >> 2:
            port->int_enable &= ~value;
            break;

        case OFS_SATA_PORT_SACTIVE >> 2:
            port->sactive = value;
            break;

        case OFS_SATA_PORT_ACTIVE_TAG >> 2:
            port->active_tag = value;
            break;

        case OFS_SATA_PORT_SATA_CTRL >> 2:
            break;

        case OFS_SATA_PORT_COMMAND >> 2:
            port->command = value;
            break;

        case OFS_SATA_PORT_PORT_CTRL >> 2:
            port->port_ctrl = value;
            break;

        case OFS_SATA_PORT_DRIVE_CTRL >> 2:
            port->drive_ctrl = value;
            break;

        case OFS_SATA_PORT_LINK_ASYNC_DATA >> 2:
            port->link_async_data = value;
            break;
        case OFS_SATA_PORT_LINK_ASYNC_R_ADDR >> 2:
            port->link_async_r_addr = value;
            break;

        case OFS_SATA_PORT_LINK_ASYNC_W_ADDR >> 2:
            port->link_async_w_addr = value;
            break;

        case OFS_SATA_PORT_LINK_ASYNC_CTRL >> 2:
            break;

        case OFS_SATA_PORT_WIN_LOW_3100 >> 2:
            port->win_low_3100 = value;
            break;

        case OFS_SATA_PORT_WIN_LOW_4732 >> 2:
            port->win_low_4732 = value;
            break;

        case OFS_SATA_PORT_WIN_HIGH_3100 >> 2:
            port->win_high_3100 = value;
            break;

        case OFS_SATA_PORT_WIN_HIGH_4732 >> 2:
            port->win_high_4732 = value;
            break;

        case OFS_SATA_PORT_WIN_CTRL_ZERO >> 2:
            port->win_ctrl_zero = value;
            break;

        case OFS_SATA_PORT_WIN_CTRL_ONE >> 2:
            port->win_ctrl_one = value;
            break;

        case OFS_SATA_PORT_WIN_CTRL_TWO >> 2:
            port->win_ctrl_two = value;
            break;

        case OFS_SATA_PORT_PORT_CTRL2 >> 2:
            port->port_ctrl2 = value;
            break;

        case OFS_SATA_PORT_PIO_SIZE >> 2:
            port->pio_size = value;
            break;

        case OFS_SATA_PORT_BACKUP1 >> 2:
            port->backup1 = value;
            break;

        case OFS_SATA_PORT_BACKUP2 >> 2:
            port->backup2 = value;
            break;

        case OFS_SATA_PORT_BACKUP3 >> 2:
            port->backup3 = value;
            break;

        case OFS_SATA_PORT_BACKUP4 >> 2:
            port->backup4 = value;
            break;

        case OFS_SATA_PORT_MSE_INFO1 >> 2:
            port->mse_info1 = value;
            break;

        case OFS_SATA_PORT_MSE_INFO2 >> 2:
            port->mse_info2 = value;
            break;

        case OFS_SATA_PORT_MSE_INFO3 >> 2:
            port->mse_info3 = value;
            break;
    }
}

static uint64_t ox820_sata_port0_read(void *opaque, target_phys_addr_t offset,
                                      unsigned size)
{
    ox820_sata_state *s = opaque;
    return ox820_sata_port_read(s, &s->port[0], offset, size);
}

static void ox820_sata_port0_write(void *opaque, target_phys_addr_t offset,
                                   uint64_t value, unsigned size)
{
    ox820_sata_state *s = opaque;
    ox820_sata_port_write(s, &s->port[0], offset, value, size);
}

static uint64_t ox820_sata_port1_read(void *opaque, target_phys_addr_t offset,
                                      unsigned size)
{
    ox820_sata_state *s = opaque;
    return ox820_sata_port_read(s, &s->port[1], offset, size);
}

static void ox820_sata_port1_write(void *opaque, target_phys_addr_t offset,
                                   uint64_t value, unsigned size)
{
    ox820_sata_state *s = opaque;
    ox820_sata_port_write(s, &s->port[1], offset, value, size);
}

static uint64_t ox820_sata_ciph_read(void *opaque, target_phys_addr_t offset,
                                     unsigned size)
{
//    ox820_sata_state *s = opaque;
    uint32_t c = 0;

    return c;
}

static void ox820_sata_ciph_write(void *opaque, target_phys_addr_t offset,
                                  uint64_t value, unsigned size)
{
//    ox820_sata_state *s = opaque;

}

static uint64_t ox820_sata_core_read(void *opaque, target_phys_addr_t offset,
                                     unsigned size)
{
    ox820_sata_state *s = opaque;
    uint32_t c = 0;
    offset &= 0xFFFF;
    if(offset >= 0x1000 && offset <= 0x13FF)
    {
        c = s->ucode.dword[offset >> 2];
    }
    else
    {
        switch(offset >> 2)
        {
            case OFS_SATA_CORE_DATA_MUX_DEBUG_1 >> 2:
                c = s->core.data_mux_debug1;
                break;

            case OFS_SATA_CORE_DATA_COUNT_PORT_0 >> 2:
                c = s->core.data_count_port0;
                break;

            case OFS_SATA_CORE_INTERRUPT_STATUS >> 2:
                c = (s->core.int_status & s->core.int_enable) | (s->core.int_status << 16);
                break;

            case OFS_SATA_CORE_INTERRUPT_ENABLE_SET >> 2:
                break;

            case OFS_SATA_CORE_INTERRUPT_ENABLE_CLEAR >> 2:
                break;

            case OFS_SATA_CORE_CONTROL >> 2:
                c = s->core.core_control;
                break;

            case OFS_SATA_CORE_AHB_JBOD_DEBUG >> 2:
                c = s->core.ahb_jbod_debug;
                break;

            case OFS_SATA_CORE_DISK_SIZE_LOWER >> 2:
                c = s->core.disk_size_lower;
                break;

            case OFS_SATA_CORE_DISK_SIZE_UPPER >> 2:
                c = s->core.disk_size_upper;
                break;

            case OFS_SATA_CORE_PORT_ERROR_MASK >> 2:
                c = s->core.port_error_mask;
                break;

            case OFS_SATA_CORE_IDLE_STATUS >> 2:
                c = s->core.idle_status;
                break;

            case OFS_SATA_CORE_DM_DEBUG_PTR0 >> 2:
                c = s->core.dm_debug_ptr0;
                break;

            case OFS_SATA_CORE_DATA_PLANE_CTRL >> 2:
                c = s->core.data_plane_ctrl;
                break;

            case OFS_SATA_CORE_CONTEXT_MASK >> 2:
                c = s->core.context_mask;
                break;

            case OFS_SATA_CORE_DATA_PLANE_STAT >> 2:
                c = s->core.data_plane_stat;
                break;
        }
    }
    return c;
}

static void ox820_sata_core_write(void *opaque, target_phys_addr_t offset,
                                  uint64_t value, unsigned size)
{
    ox820_sata_state *s = opaque;
    offset &= 0xFFFF;
    if(offset >= 0x1000 && offset <= 0x13FF)
    {
        s->ucode.dword[offset >> 2] = (uint32_t) value;
    }
    else
    {
        switch(offset >> 2)
        {
            case OFS_SATA_CORE_DATA_MUX_DEBUG_1 >> 2:
                s->core.data_mux_debug1 = value;
                break;

            case OFS_SATA_CORE_DATA_COUNT_PORT_0 >> 2:
                s->core.data_count_port0 = value;
                break;

            case OFS_SATA_CORE_INTERRUPT_STATUS >> 2:
                s->core.int_status &= (~value | 0x3);
                break;

            case OFS_SATA_CORE_INTERRUPT_ENABLE_SET >> 2:
                s->core.int_enable |= (value & 0xFFFF);
                break;

            case OFS_SATA_CORE_INTERRUPT_ENABLE_CLEAR >> 2:
                s->core.int_enable &= ~value;
                break;

            case OFS_SATA_CORE_CONTROL >> 2:
                s->core.core_control = value;
                break;

            case OFS_SATA_CORE_AHB_JBOD_DEBUG >> 2:
                s->core.ahb_jbod_debug = value;
                break;

            case OFS_SATA_CORE_DISK_SIZE_LOWER >> 2:
                s->core.disk_size_lower = value;
                break;

            case OFS_SATA_CORE_DISK_SIZE_UPPER >> 2:
                s->core.disk_size_upper = value;
                break;

            case OFS_SATA_CORE_PORT_ERROR_MASK >> 2:
                s->core.port_error_mask = value;
                break;

            case OFS_SATA_CORE_IDLE_STATUS >> 2:
                s->core.idle_status = value;
                break;

            case OFS_SATA_CORE_DM_DEBUG_PTR0 >> 2:
                s->core.dm_debug_ptr0 = value;
                break;

            case OFS_SATA_CORE_DATA_PLANE_CTRL >> 2:
                s->core.data_plane_ctrl = value;
                break;

            case OFS_SATA_CORE_CONTEXT_MASK >> 2:
                s->core.context_mask = value;
                break;

            case OFS_SATA_CORE_DATA_PLANE_STAT >> 2:
                s->core.data_plane_stat = value;
                break;
        }
    }
}

static uint64_t ox820_sata_raid_read(void *opaque, target_phys_addr_t offset,
                                     unsigned size)
{
    ox820_sata_state *s = opaque;
    uint32_t c = 0;
    switch(c >> 2)
    {
        case OFS_SATA_PORT_ORB1 >> 2:
            c = s->raid_port.orb1;
            break;

        case OFS_SATA_PORT_ORB2 >> 2:
            c = s->raid_port.orb2;
            break;

        case OFS_SATA_PORT_ORB3 >> 2:
            c = s->raid_port.orb3;
            break;

        case OFS_SATA_PORT_ORB4 >> 2:
            c = s->raid_port.orb4;
            break;

        case OFS_SATA_PORT_ORB5 >> 2:
            c = s->raid_port.orb5;
            break;

        case OFS_SATA_PORT_RBC1 >> 2:
            c = s->raid_port.rbc1;
            break;

        case OFS_SATA_PORT_RBC2 >> 2:
            c = s->raid_port.rbc2;
            break;

        case OFS_SATA_PORT_RBC3 >> 2:
            c = s->raid_port.rbc3;
            break;

        case OFS_SATA_PORT_RBC4 >> 2:
            c = s->raid_port.rbc4;
            break;

        case OFS_SATA_PORT_INT_CTRL_STATUS >> 2:
            c = (s->raid_port.int_status & s->raid_port.int_enable) | (s->raid_port.int_status << 16);
            break;

        case OFS_SATA_PORT_INT_ENABLE_SET >> 2:
            break;

        case OFS_SATA_PORT_INT_ENABLE_CLEAR >> 2:
            break;

        case OFS_SATA_PORT_VERSION >> 2:
            c = 0x1f3;
            break;

        case OFS_SATA_PORT_SATA_CTRL >> 2:
            break;

        case OFS_SATA_PORT_COMMAND >> 2:
            c = s->raid_port.command;
            break;

        case OFS_SATA_PORT_WIN_LOW_3100 >> 2:
            c = s->raid_port.win_low_3100;
            break;

        case OFS_SATA_PORT_WIN_LOW_4732 >> 2:
            c = s->raid_port.win_low_4732;
            break;

        case OFS_SATA_PORT_WIN_HIGH_3100 >> 2:
            c = s->raid_port.win_high_3100;
            break;

        case OFS_SATA_PORT_WIN_HIGH_4732 >> 2:
            c = s->raid_port.win_high_4732;
            break;

        case OFS_SATA_PORT_WIN_CTRL_ZERO >> 2:
            c = s->raid_port.win_ctrl_zero;
            break;

        case OFS_SATA_PORT_WIN_CTRL_ONE >> 2:
            c = s->raid_port.win_ctrl_one;
            break;

        case OFS_SATA_PORT_WIN_CTRL_TWO >> 2:
            c = s->raid_port.win_ctrl_two;
            break;

        case OFS_SATA_PORT_BACKUP1 >> 2:
            c = s->raid_port.backup1;
            break;

        case OFS_SATA_PORT_BACKUP2 >> 2:
            c = s->raid_port.backup2;
            break;

        case OFS_SATA_PORT_BACKUP3 >> 2:
            c = s->raid_port.backup3;
            break;

        case OFS_SATA_PORT_BACKUP4 >> 2:
            c = s->raid_port.backup4;
            break;
    }
    return c;
}

static void ox820_sata_raid_write(void *opaque, target_phys_addr_t offset,
                                  uint64_t value, unsigned size)
{
    ox820_sata_state *s = opaque;

    switch(offset >> 2)
    {
        case OFS_SATA_PORT_ORB1 >> 2:
            s->raid_port.orb1 = value;
            break;

        case OFS_SATA_PORT_ORB2 >> 2:
            s->raid_port.orb2 = value;
            break;

        case OFS_SATA_PORT_ORB3 >> 2:
            s->raid_port.orb3 = value;
            break;

        case OFS_SATA_PORT_ORB4 >> 2:
            s->raid_port.orb4 = value;
            break;

        case OFS_SATA_PORT_ORB5 >> 2:
            s->raid_port.orb5 = value;
            break;

        case OFS_SATA_PORT_RBC1 >> 2:
            s->raid_port.rbc1 = value;
            break;

        case OFS_SATA_PORT_RBC2 >> 2:
            s->raid_port.rbc2 = value;
            break;

        case OFS_SATA_PORT_RBC3 >> 2:
            s->raid_port.rbc3 = value;
            break;

        case OFS_SATA_PORT_RBC4 >> 2:
            s->raid_port.rbc4 = value;
            break;

        case OFS_SATA_PORT_INT_CTRL_STATUS >> 2:
            s->raid_port.int_status &= ~value;
            break;

        case OFS_SATA_PORT_INT_ENABLE_SET >> 2:
            s->raid_port.int_enable |= (value & 0x388F);
            break;

        case OFS_SATA_PORT_INT_ENABLE_CLEAR >> 2:
            s->raid_port.int_enable &= ~value;
            break;

        case OFS_SATA_PORT_SATA_CTRL >> 2:
            break;

        case OFS_SATA_PORT_COMMAND >> 2:
            s->raid_port.command = value;
            break;

        case OFS_SATA_PORT_WIN_LOW_3100 >> 2:
            s->raid_port.win_low_3100 = value;
            break;

        case OFS_SATA_PORT_WIN_LOW_4732 >> 2:
            s->raid_port.win_low_4732 = value;
            break;

        case OFS_SATA_PORT_WIN_HIGH_3100 >> 2:
            s->raid_port.win_high_3100 = value;
            break;

        case OFS_SATA_PORT_WIN_HIGH_4732 >> 2:
            s->raid_port.win_high_4732 = value;
            break;

        case OFS_SATA_PORT_WIN_CTRL_ZERO >> 2:
            s->raid_port.win_ctrl_zero = value;
            break;

        case OFS_SATA_PORT_WIN_CTRL_ONE >> 2:
            s->raid_port.win_ctrl_one = value;
            break;

        case OFS_SATA_PORT_WIN_CTRL_TWO >> 2:
            s->raid_port.win_ctrl_two = value;
            break;

        case OFS_SATA_PORT_BACKUP1 >> 2:
            s->raid_port.backup1 = value;
            break;

        case OFS_SATA_PORT_BACKUP2 >> 2:
            s->raid_port.backup2 = value;
            break;

        case OFS_SATA_PORT_BACKUP3 >> 2:
            s->raid_port.backup3 = value;
            break;

        case OFS_SATA_PORT_BACKUP4 >> 2:
            s->raid_port.backup4 = value;
            break;

    }
}

static void ox820_sata_reset(ox820_sata_state* s)
{

}

static void ox820_sata_reset_hnd(DeviceState *d)
{
    ox820_sata_state *s = DO_UPCAST(ox820_sata_state, busdev.qdev, d);

    ox820_sata_reset(s);
}

static void ox820_sata_set_irq(void* opaque, int irq, int level)
{
    ox820_sata_state *s = opaque;
    uint32_t oldval;
    /* 0 => reset, 1 => cken */
    switch(irq)
    {
        case 0:
            qemu_set_irq(s->dma_irq[0], level);
            oldval = s->rsten;
            if(!s->rsten && level)
            {
                ox820_sata_reset(s);
            }
            s->rsten = level ? 1 : 0;
            break;

        case 1:
            qemu_set_irq(s->dma_irq[1], level);
            oldval = s->cken;
            s->cken = level ? 1 : 0;
            if(!oldval && s->rsten && s->cken)
            {
                /* check ports */
            }
            break;
    }
}


static const MemoryRegionOps ox820_sata_port0_ops = {
    .read = ox820_sata_port0_read,
    .write = ox820_sata_port0_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps ox820_sata_port1_ops = {
    .read = ox820_sata_port1_read,
    .write = ox820_sata_port1_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};
static const MemoryRegionOps ox820_sata_ciph_ops = {
    .read = ox820_sata_ciph_read,
    .write = ox820_sata_ciph_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps ox820_sata_core_ops = {
    .read = ox820_sata_core_read,
    .write = ox820_sata_core_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps ox820_sata_raid_ops = {
    .read = ox820_sata_raid_read,
    .write = ox820_sata_raid_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_ox820_sata = {
    .name = "ox820-sata",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(cken, ox820_sata_state),
        VMSTATE_UINT32(rsten, ox820_sata_state),
        VMSTATE_UINT32(ucode_type, ox820_sata_state),
        VMSTATE_END_OF_LIST()
    }
};

static void ox820_readblock(void* opaque, target_phys_addr_t physaddr, void* buf, int len)
{

}

static void ox820_writeblock(void* opaque, target_phys_addr_t physaddr, const void* buf, int len)
{

}

static int ox820_sata_init(SysBusDevice *dev)
{
    ox820_sata_state *s = FROM_SYSBUS(ox820_sata_state, dev);
    DeviceState* dma_dev;
    SysBusDevice* busdev;

    memory_region_init(&s->iomem, "ox820-sata", 0x100000);
    memory_region_init_io(&s->iomem_port0, &ox820_sata_port0_ops, s, "port0", 0x10000);
    memory_region_init_io(&s->iomem_port1, &ox820_sata_port1_ops, s, "port1", 0x10000);
    memory_region_init_io(&s->iomem_ciph, &ox820_sata_ciph_ops, s, "cipher", 0x10000);
    memory_region_init_io(&s->iomem_core, &ox820_sata_core_ops, s, "core", 0x10000);
    memory_region_init_io(&s->iomem_raid, &ox820_sata_raid_ops, s, "raid", 0x10000);

    dma_dev = ox820_dma_initialize(2, 0, 1,
                                   ox820_readblock,
                                   ox820_writeblock,
                                   s);
    /* TODO: check where both IRQs have to be connected to */
    busdev = sysbus_from_qdev(dma_dev);
    s->dma_irq[0] = qdev_get_gpio_in(dma_dev, 0);
    s->dma_irq[1] = qdev_get_gpio_in(dma_dev, 1);
    memory_region_add_subregion(&s->iomem, 0x00000, &s->iomem_port0);
    memory_region_add_subregion(&s->iomem, 0x10000, &s->iomem_port1);
    memory_region_add_subregion(&s->iomem, 0xA0000, sysbus_mmio_get_region(busdev, 0));
    memory_region_add_subregion(&s->iomem, 0xC0000, &s->iomem_ciph);
    memory_region_add_subregion(&s->iomem, 0xE0000, &s->iomem_core);
    memory_region_add_subregion(&s->iomem, 0xF0000, &s->iomem_raid);
    sysbus_init_mmio(dev, &s->iomem);
    qdev_init_gpio_in(&dev->qdev, ox820_sata_set_irq, 2);
    sysbus_init_irq(dev, &s->irq);

    vmstate_register(&dev->qdev, -1, &vmstate_ox820_sata, s);
    return 0;
}

static Property ox820_sata_properties[] = {
    DEFINE_PROP_UINT32("num-port", ox820_sata_state, num_port, 1),
    DEFINE_PROP_UINT32("cken", ox820_sata_state, cken, 0),
    DEFINE_PROP_UINT32("rsten", ox820_sata_state, rsten, 1),
    DEFINE_PROP_END_OF_LIST(),
};

static void ox820_sata_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->no_user = 1;
    sdc->init = ox820_sata_init;
    dc->reset = ox820_sata_reset_hnd;
    dc->props = ox820_sata_properties;
}

static TypeInfo ox820_sata_info = {
    .name          = "ox820-sata",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ox820_sata_state),
    .class_init    = ox820_sata_class_init,
};

static void ox820_sata_register_devices(void)
{
    type_register_static(&ox820_sata_info);
}

device_init(ox820_sata_register_devices)
