/*
 * ox820-rps unit
 *
 * Written by Sven Bormann
 *
 * This code is licensed under the GPL.
 */

#include "sysbus.h"

typedef struct {
    SysBusDevice    busdev;
    MemoryRegion    iomem;
    uint32_t        mfa_secsel_ctrl;
    uint32_t        cken_stat;
    uint32_t        rsten_stat;
    uint32_t        usbmph_ctrl;
    uint32_t        usbmph_stat;
    uint32_t        sema_stat;
    uint32_t        sema_maska_ctrl;
    uint32_t        sema_maskb_ctrl;
    uint32_t        sema_maskc_ctrl;
    uint32_t        audio_ctrl;
    uint32_t        gmaca_ctrl;
    uint32_t        arm_stat;
    uint32_t        usbaphy_ctrl;
    uint32_t        usbaphy_stat;
    uint32_t        mfa_tersel_ctrl;
    uint32_t        usb_ctrl;
    uint32_t        mfa_quatsel_ctrl;
    uint32_t        mfa_debugsel_ctrl;
    uint32_t        mfa_altsel_ctrl;
    uint32_t        mfa_pullup_ctrl;
    uint32_t        scratchword0;
    uint32_t        scratchword1;
    uint32_t        scratchword2;
    uint32_t        scratchword3;
    uint32_t        monmux_ctrl;
    uint32_t        gmacb_ctrl;
    uint32_t        usbbphy_ctrl;
    uint32_t        usbbphy_stat;
    uint32_t        ref300_div_ctrl;
    uint32_t        gmaca_delay;
    uint32_t        gmacb_delay;
    uint32_t        gmac_dll;
    uint32_t        gmac_samp;
    uint32_t        amba_ctrl;
    uint32_t        hcsl_ctrl;
    uint32_t        usbaphy_tune;
    uint32_t        usbbphy_tune;
    uint32_t        pciea_ctrl;
    uint32_t        pcieb_ctrl;
    uint32_t        pciephy_ctrl;
    uint32_t        ddramio_ctrl;
    uint32_t        pciea_pom0_mem_addr_offset;
    uint32_t        pciea_pom1_mem_addr_offset;
    uint32_t        pciea_in0_mem_addr_start;
    uint32_t        pciea_in1_mem_addr_start;
    uint32_t        pciea_in_io_addr_start;
    uint32_t        pciea_in_cfg0_addr_start;
    uint32_t        pciea_in_cfg1_addr_start;
    uint32_t        pciea_in_msg_addr_start;
    uint32_t        pciea_in0_mem_addr_limit;
    uint32_t        pciea_in1_mem_addr_limit;
    uint32_t        pciea_in_io_addr_limit;
    uint32_t        pciea_in_cfg0_addr_limit;
    uint32_t        pciea_in_cfg1_addr_limit;
    uint32_t        pciea_in_msg_addr_limit;
    uint32_t        pciea_ahb_slave_ctrl;
    uint32_t        pcieb_pom0_mem_addr_offset;
    uint32_t        pcieb_pom1_mem_addr_offset;
    uint32_t        pcieb_in0_mem_addr_start;
    uint32_t        pcieb_in1_mem_addr_start;
    uint32_t        pcieb_in_io_addr_start;
    uint32_t        pcieb_in_cfg0_addr_start;
    uint32_t        pcieb_in_cfg1_addr_start;
    uint32_t        pcieb_in_msg_addr_start;
    uint32_t        pcieb_in0_mem_addr_limit;
    uint32_t        pcieb_in1_mem_addr_limit;
    uint32_t        pcieb_in_io_addr_limit;
    uint32_t        pcieb_in_cfg0_addr_limit;
    uint32_t        pcieb_in_cfg1_addr_limit;
    uint32_t        pcieb_in_msg_addr_limit;
    uint32_t        pcieb_ahb_slave_ctrl;
    uint32_t        pciea_ahb_slave_cfg_ctrl;
    uint32_t        pcieb_ahb_slave_cfg_ctrl;
    uint32_t        pciea_ahb_mstr_ctrl;
    uint32_t        pcieb_ahb_mstr_ctrl;
    uint32_t        pciea_ahb_slv_misc_info;
    uint32_t        pcieb_ahb_slv_misc_info;
    uint32_t        plla_ctrl0;
    uint32_t        plla_ctrl1;
    uint32_t        plla_ctrl2;
    uint32_t        plla_ctrl3;

    qemu_irq        irq_sema_a;
    qemu_irq        irq_sema_b;
    qemu_irq        irq_sema_c;
} ox820_sysctrl_state;

static void ox820_sysctrl_sema_update(ox820_sysctrl_state* s)
{
    qemu_set_irq(s->irq_sema_a, (s->sema_stat & s->sema_maska_ctrl) != 0);
    qemu_set_irq(s->irq_sema_b, (s->sema_stat & s->sema_maskb_ctrl) != 0);
    qemu_set_irq(s->irq_sema_c, (s->sema_stat & s->sema_maskc_ctrl) != 0);
}

static uint64_t ox820_sysctrl_read(void *opaque, target_phys_addr_t offset,
                           unsigned size)
{
    ox820_sysctrl_state *s = (ox820_sysctrl_state *)opaque;
    uint32_t c = 0;

    switch (offset >> 2) {
    case 0x0014 >> 2:
        c = s->mfa_secsel_ctrl;
        break;

    case 0x0024 >> 2:
        c = s->cken_stat;
        break;

    case 0x0028 >> 2:
        c = s->rsten_stat;
        break;

    case 0x0040 >> 2:
        c = s->usbmph_ctrl;
        break;

    case 0x0044 >> 2:
        c = s->usbmph_stat;
        break;

    case 0x004C >> 2:
        c = s->sema_stat;
        break;

    case 0x0058 >> 2:
        c = s->sema_maska_ctrl;
        break;

    case 0x005C >> 2:
        c = s->sema_maskb_ctrl;
        break;

    case 0x0060 >> 2:
        c = s->sema_maskc_ctrl;
        break;

    case 0x0070 >> 2:
        c = s->audio_ctrl;
        break;

    case 0x0078 >> 2:
        c = s->gmaca_ctrl;
        break;

    case 0x0080 >> 2:
        c = s->arm_stat;
        break;

    case 0x0084 >> 2:
        c = s->usbaphy_ctrl;
        break;

    case 0x0088 >> 2:
        c = s->usbaphy_stat;
        break;

    case 0x008C >> 2:
        c = s->mfa_tersel_ctrl;
        break;

    case 0x0090 >> 2:
        c = s->usb_ctrl;
        break;

    case 0x0094 >> 2:
        c = s->mfa_quatsel_ctrl;
        break;

    case 0x009C >> 2:
        c = s->mfa_debugsel_ctrl;
        break;

    case 0x00A4 >> 2:
        c = s->mfa_altsel_ctrl;
        break;

    case 0x00AC >> 2:
        c = s->mfa_pullup_ctrl;
        break;

    case 0x00C4 >> 2:
        c = s->scratchword0;
        break;

    case 0x00C8 >> 2:
        c = s->scratchword1;
        break;

    case 0x00CC >> 2:
        c = s->scratchword2;
        break;

    case 0x00D0 >> 2:
        c = s->scratchword3;
        break;

    case 0x00D4 >> 2:
        c = s->monmux_ctrl;
        break;

    case 0x00EC >> 2:
        c = s->gmacb_ctrl;
        break;

    case 0x00F0 >> 2:
        c = s->usbbphy_ctrl;
        break;

    case 0x00F4 >> 2:
        c = s->usbbphy_stat;
        break;

    case 0x00F8 >> 2:
        c = s->ref300_div_ctrl;
        break;

    case 0x0100 >> 2:
        c = s->gmaca_delay;
        break;

    case 0x0104 >> 2:
        c = s->gmacb_delay;
        break;

    case 0x0108 >> 2:
        c = s->gmac_dll;
        break;

    case 0x010C >> 2:
        c = s->gmac_samp;
        break;

    case 0x0110 >> 2:
        c = s->amba_ctrl;
        break;

    case 0x0114 >> 2:
        c = s->hcsl_ctrl;
        break;

    case 0x0118 >> 2:
        c = s->usbaphy_tune;
        break;

    case 0x011C >> 2:
        c = s->usbbphy_tune;
        break;

    case 0x0120 >> 2:
        c = s->pciea_ctrl;
        break;

    case 0x0124 >> 2:
        c = s->pcieb_ctrl;
        break;

    case 0x0128 >> 2:
        c = s->pciephy_ctrl;
        break;

    case 0x0134 >> 2:
        c = s->ddramio_ctrl;
        break;

    case 0x0138 >> 2:
        c = s->pciea_pom0_mem_addr_offset;
        break;

    case 0x013C >> 2:
        c = s->pciea_pom1_mem_addr_offset;
        break;

    case 0x0140 >> 2:
        c = s->pciea_in0_mem_addr_start;
        break;

    case 0x0144 >> 2:
        c = s->pciea_in1_mem_addr_start;
        break;

    case 0x0148 >> 2:
        c = s->pciea_in_io_addr_start;
        break;

    case 0x014C >> 2:
        c = s->pciea_in_cfg0_addr_start;
        break;

    case 0x0150 >> 2:
        c = s->pciea_in_cfg1_addr_start;
        break;

    case 0x0154 >> 2:
        c = s->pciea_in_msg_addr_start;
        break;

    case 0x0158 >> 2:
        c = s->pciea_in0_mem_addr_limit;
        break;

    case 0x015C >> 2:
        c = s->pciea_in1_mem_addr_limit;
        break;

    case 0x0160 >> 2:
        c = s->pciea_in_io_addr_limit;
        break;

    case 0x0164 >> 2:
        c = s->pciea_in_cfg0_addr_limit;
        break;

    case 0x0168 >> 2:
        c = s->pciea_in_cfg1_addr_limit;
        break;

    case 0x016C >> 2:
        c = s->pciea_in_msg_addr_limit;
        break;

    case 0x0170 >> 2:
        c = s->pciea_ahb_slave_ctrl;
        break;

    case 0x0174 >> 2:
        c = s->pcieb_pom0_mem_addr_offset;
        break;

    case 0x0178 >> 2:
        c = s->pcieb_pom1_mem_addr_offset;
        break;

    case 0x017C >> 2:
        c = s->pcieb_in0_mem_addr_start;
        break;

    case 0x0180 >> 2:
        c = s->pcieb_in1_mem_addr_start;
        break;

    case 0x0184 >> 2:
        c = s->pcieb_in_io_addr_start;
        break;

    case 0x0188 >> 2:
        c = s->pcieb_in_cfg0_addr_start;
        break;

    case 0x018C >> 2:
        c = s->pcieb_in_cfg1_addr_start;
        break;

    case 0x0190 >> 2:
        c = s->pcieb_in_msg_addr_start;
        break;

    case 0x0194 >> 2:
        c = s->pcieb_in0_mem_addr_limit;
        break;

    case 0x0198 >> 2:
        c = s->pcieb_in1_mem_addr_limit;
        break;

    case 0x019C >> 2:
        c = s->pcieb_in_io_addr_limit;
        break;

    case 0x01A0 >> 2:
        c = s->pcieb_in_cfg0_addr_limit;
        break;

    case 0x01A4 >> 2:
        c = s->pcieb_in_cfg1_addr_limit;
        break;

    case 0x01A8 >> 2:
        c = s->pcieb_in_msg_addr_limit;
        break;

    case 0x01AC >> 2:
        c = s->pcieb_ahb_slave_ctrl;
        break;

    case 0x01B0 >> 2:
        c = s->pciea_ahb_slave_cfg_ctrl;
        break;

    case 0x01B4 >> 2:
        c = s->pcieb_ahb_slave_cfg_ctrl;
        break;

    case 0x01B8 >> 2:
        c = s->pciea_ahb_mstr_ctrl;
        break;

    case 0x01BC >> 2:
        c = s->pcieb_ahb_mstr_ctrl;
        break;

    case 0x01C0 >> 2:
        c = s->pciea_ahb_slv_misc_info;
        break;

    case 0x01C4 >> 2:
        c = s->pcieb_ahb_slv_misc_info;
        break;

    case 0x01F0 >> 2:
        c = s->plla_ctrl0;
        break;

    case 0x01F4 >> 2:
        c = s->plla_ctrl1;
        break;

    case 0x01F8 >> 2:
        c = s->plla_ctrl2;
        break;

    case 0x01FC >> 2:
        c = s->plla_ctrl3;
        break;

    default:
        return 0;
    }
    return c;
}

static void ox820_sysctrl_write(void *opaque, target_phys_addr_t offset,
                                uint64_t value, unsigned size)
{
    ox820_sysctrl_state *s = (ox820_sysctrl_state *)opaque;

    switch(offset >> 2) {
    case 0x0014 >> 2:
        s->mfa_secsel_ctrl = value;
        break;

    case 0x002C >> 2:
        s->cken_stat |= value;
        break;

    case 0x0030 >> 2:
        s->cken_stat &= (~value);
        break;

    case 0x0034 >> 2:
        s->rsten_stat |= value; /* TODO: how to pass on reset from here */
        break;

    case 0x0038 >> 2:
        s->rsten_stat &= (~value);
        break;

    case 0x0044 >> 2:
        s->usbmph_stat = value;
        break;

    case 0x0050 >> 2:
        s->sema_stat |= value;
        break;

    case 0x0054 >> 2:
        s->sema_stat &= (~value);
        break;

    case 0x0058 >> 2:
        s->sema_maska_ctrl = value;
        ox820_sysctrl_sema_update(s);
        break;

    case 0x005C >> 2:
        s->sema_maskb_ctrl = value;
        ox820_sysctrl_sema_update(s);
        break;

    case 0x0060 >> 2:
        s->sema_maskc_ctrl = value;
        ox820_sysctrl_sema_update(s);
        break;

    case 0x0070 >> 2:
        s->audio_ctrl = value;
        break;

    case 0x0078 >> 2:
        s->gmaca_ctrl = value;
        break;

    case 0x0080 >> 2:
        s->arm_stat = value;
        break;

    case 0x0084 >> 2:
        s->usbaphy_ctrl = value;
        break;

    case 0x0088 >> 2:
        s->usbaphy_stat = value;
        break;

    case 0x008C >> 2:
        s->mfa_tersel_ctrl = value;
        break;

    case 0x0090 >> 2:
        s->usb_ctrl = value;
        break;

    case 0x0094 >> 2:
        s->mfa_quatsel_ctrl = value;
        break;

    case 0x009C >> 2:
        s->mfa_debugsel_ctrl = value;
        break;

    case 0x00A4 >> 2:
        s->mfa_altsel_ctrl = value;
        break;

    case 0x00AC >> 2:
        s->mfa_pullup_ctrl = value;
        break;

    case 0x00C4 >> 2:
        s->scratchword0 = value;
        break;

    case 0x00C8 >> 2:
        s->scratchword1 = value;
        break;

    case 0x00CC >> 2:
        s->scratchword2 = value;
        break;

    case 0x00D0 >> 2:
        s->scratchword3 = value;
        break;

    case 0x00D4 >> 2:
        s->monmux_ctrl = value;
        break;

    case 0x00EC >> 2:
        s->gmacb_ctrl = value;
        break;

    case 0x00F0 >> 2:
        s->usbbphy_ctrl = value;
        break;

    case 0x00F4 >> 2:
        s->usbbphy_stat = value;
        break;

    case 0x00F8 >> 2:
        s->ref300_div_ctrl = value;
        break;

    case 0x0100 >> 2:
        s->gmaca_delay = value;
        break;

    case 0x0104 >> 2:
        s->gmacb_delay = value;
        break;

    case 0x0108 >> 2:
        s->gmac_dll = value;
        break;

    case 0x010C >> 2:
        s->gmac_samp = value;
        break;

    case 0x0110 >> 2:
        s->amba_ctrl = value;
        break;

    case 0x0114 >> 2:
        s->hcsl_ctrl = value;
        break;

    case 0x0118 >> 2:
        s->usbaphy_tune = value;
        break;

    case 0x011C >> 2:
        s->usbbphy_tune = value;
        break;

    case 0x0120 >> 2:
        s->pciea_ctrl = value;
        break;

    case 0x0124 >> 2:
        s->pcieb_ctrl = value;
        break;

    case 0x0128 >> 2:
        s->pciephy_ctrl = value;
        break;

    case 0x0134 >> 2:
        s->ddramio_ctrl = value;
        break;

    case 0x0138 >> 2:
        s->pciea_pom0_mem_addr_offset = value;
        break;

    case 0x013C >> 2:
        s->pciea_pom1_mem_addr_offset = value;
        break;

    case 0x0140 >> 2:
        s->pciea_in0_mem_addr_start = value;
        break;

    case 0x0144 >> 2:
        s->pciea_in1_mem_addr_start = value;
        break;

    case 0x0148 >> 2:
        s->pciea_in_io_addr_start = value;
        break;

    case 0x014C >> 2:
        s->pciea_in_cfg0_addr_start = value;
        break;

    case 0x0150 >> 2:
        s->pciea_in_cfg1_addr_start = value;
        break;

    case 0x0154 >> 2:
        s->pciea_in_msg_addr_start = value;
        break;

    case 0x0158 >> 2:
        s->pciea_in0_mem_addr_limit = value;
        break;

    case 0x015C >> 2:
        s->pciea_in1_mem_addr_limit = value;
        break;

    case 0x0160 >> 2:
        s->pciea_in_io_addr_limit = value;
        break;

    case 0x0164 >> 2:
        s->pciea_in_cfg0_addr_limit = value;
        break;

    case 0x0168 >> 2:
        s->pciea_in_cfg1_addr_limit = value;
        break;

    case 0x016C >> 2:
        s->pciea_in_msg_addr_limit = value;
        break;

    case 0x0170 >> 2:
        s->pciea_ahb_slave_ctrl = value;
        break;

    case 0x0174 >> 2:
        s->pcieb_pom0_mem_addr_offset = value;
        break;

    case 0x0178 >> 2:
        s->pcieb_pom1_mem_addr_offset = value;
        break;

    case 0x017C >> 2:
        s->pcieb_in0_mem_addr_start = value;
        break;

    case 0x0180 >> 2:
        s->pcieb_in1_mem_addr_start = value;
        break;

    case 0x0184 >> 2:
        s->pcieb_in_io_addr_start = value;
        break;

    case 0x0188 >> 2:
        s->pcieb_in_cfg0_addr_start = value;
        break;

    case 0x018C >> 2:
        s->pcieb_in_cfg1_addr_start = value;
        break;

    case 0x0190 >> 2:
        s->pcieb_in_msg_addr_start = value;
        break;

    case 0x0194 >> 2:
        s->pcieb_in0_mem_addr_limit = value;
        break;

    case 0x0198 >> 2:
        s->pcieb_in1_mem_addr_limit = value;
        break;

    case 0x019C >> 2:
        s->pcieb_in_io_addr_limit = value;
        break;

    case 0x01A0 >> 2:
        s->pcieb_in_cfg0_addr_limit = value;
        break;

    case 0x01A4 >> 2:
        s->pcieb_in_cfg1_addr_limit = value;
        break;

    case 0x01A8 >> 2:
        s->pcieb_in_msg_addr_limit = value;
        break;

    case 0x01AC >> 2:
        s->pcieb_ahb_slave_ctrl = value;
        break;

    case 0x01B0 >> 2:
        s->pciea_ahb_slave_cfg_ctrl = value;
        break;

    case 0x01B4 >> 2:
        s->pcieb_ahb_slave_cfg_ctrl = value;
        break;

    case 0x01B8 >> 2:
        s->pciea_ahb_mstr_ctrl = value;
        break;

    case 0x01BC >> 2:
        s->pcieb_ahb_mstr_ctrl = value;
        break;

    case 0x01C0 >> 2:
        s->pciea_ahb_slv_misc_info = value;
        break;

    case 0x01C4 >> 2:
        s->pcieb_ahb_slv_misc_info = value;
        break;

    case 0x01F0 >> 2:
        s->plla_ctrl0 = value;
        break;

    case 0x01F4 >> 2:
        s->plla_ctrl1 = value;
        break;

    case 0x01F8 >> 2:
        s->plla_ctrl2 = value;
        break;

    case 0x01FC >> 2:
        s->plla_ctrl3 = value;
        break;

    default:
        break;
    }
}

static void ox820_sysctrl_reset(DeviceState *d)
{
    ox820_sysctrl_state *s = DO_UPCAST(ox820_sysctrl_state, busdev.qdev, d);

    s->mfa_secsel_ctrl = 0;
    s->cken_stat = 0x10000;
    s->rsten_stat = 0x8FFEFFF2;
    s->usbmph_ctrl = 0;
    s->usbmph_stat = 0;
    s->sema_stat = 0;
    s->sema_maska_ctrl = 0;
    s->sema_maskb_ctrl = 0;
    s->sema_maskc_ctrl = 0;
    s->audio_ctrl = 0;
    s->gmaca_ctrl = 0;
    s->arm_stat = 0;
    s->usbaphy_ctrl = 0;
    s->usbaphy_stat = 0;
    s->mfa_tersel_ctrl = 0;
    s->usb_ctrl = 0;
    s->mfa_quatsel_ctrl = 0;
    s->mfa_debugsel_ctrl = 0;
    s->mfa_altsel_ctrl = 0;
    s->mfa_pullup_ctrl = 0;
    s->scratchword0 = 0;
    s->scratchword1 = 0;
    s->scratchword2 = 0;
    s->scratchword3 = 0;
    s->monmux_ctrl = 0;
    s->gmacb_ctrl = 0;
    s->usbbphy_ctrl = 0;
    s->usbbphy_stat = 0;
    s->ref300_div_ctrl = 0;
    s->gmaca_delay = 0;
    s->gmacb_delay = 0;
    s->gmac_dll = 0;
    s->gmac_samp = 0;
    s->amba_ctrl = 0;
    s->hcsl_ctrl = 0;
    s->usbaphy_tune = 0;
    s->usbbphy_tune = 0;
    s->pciea_ctrl = 0;
    s->pcieb_ctrl = 0;
    s->pciephy_ctrl = 0;
    s->ddramio_ctrl = 0;
    s->pciea_pom0_mem_addr_offset = 0;
    s->pciea_pom1_mem_addr_offset = 0;
    s->pciea_in0_mem_addr_start = 0;
    s->pciea_in1_mem_addr_start = 0;
    s->pciea_in_io_addr_start = 0;
    s->pciea_in_cfg0_addr_start = 0;
    s->pciea_in_cfg1_addr_start = 0;
    s->pciea_in_msg_addr_start = 0;
    s->pciea_in0_mem_addr_limit = 0;
    s->pciea_in1_mem_addr_limit = 0;
    s->pciea_in_io_addr_limit = 0;
    s->pciea_in_cfg0_addr_limit = 0;
    s->pciea_in_cfg1_addr_limit = 0;
    s->pciea_in_msg_addr_limit = 0;
    s->pciea_ahb_slave_ctrl = 0;
    s->pcieb_pom0_mem_addr_offset = 0;
    s->pcieb_pom1_mem_addr_offset = 0;
    s->pcieb_in0_mem_addr_start = 0;
    s->pcieb_in1_mem_addr_start = 0;
    s->pcieb_in_io_addr_start = 0;
    s->pcieb_in_cfg0_addr_start = 0;
    s->pcieb_in_cfg1_addr_start = 0;
    s->pcieb_in_msg_addr_start = 0;
    s->pcieb_in0_mem_addr_limit = 0;
    s->pcieb_in1_mem_addr_limit = 0;
    s->pcieb_in_io_addr_limit = 0;
    s->pcieb_in_cfg0_addr_limit = 0;
    s->pcieb_in_cfg1_addr_limit = 0;
    s->pcieb_in_msg_addr_limit = 0;
    s->pcieb_ahb_slave_ctrl = 0;
    s->pciea_ahb_slave_cfg_ctrl = 0;
    s->pcieb_ahb_slave_cfg_ctrl = 0;
    s->pciea_ahb_mstr_ctrl = 0;
    s->pcieb_ahb_mstr_ctrl = 0;
    s->pciea_ahb_slv_misc_info = 0;
    s->pcieb_ahb_slv_misc_info = 0;
    s->plla_ctrl0 = 0;
    s->plla_ctrl1 = 0;
    s->plla_ctrl2 = 0;
    s->plla_ctrl3 = 0;

    ox820_sysctrl_sema_update(s);
}

static const MemoryRegionOps ox820_sysctrl_ops = {
    .read = ox820_sysctrl_read,
    .write = ox820_sysctrl_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_ox820_sysctrl = {
    .name = "ox820-sysctrl",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(mfa_secsel_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(cken_stat, ox820_sysctrl_state),
        VMSTATE_UINT32(rsten_stat, ox820_sysctrl_state),
        VMSTATE_UINT32(usbmph_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(usbmph_stat, ox820_sysctrl_state),
        VMSTATE_UINT32(sema_stat, ox820_sysctrl_state),
        VMSTATE_UINT32(sema_maska_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(sema_maskb_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(sema_maskc_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(audio_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(gmaca_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(arm_stat, ox820_sysctrl_state),
        VMSTATE_UINT32(usbaphy_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(usbaphy_stat, ox820_sysctrl_state),
        VMSTATE_UINT32(mfa_tersel_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(usb_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(mfa_quatsel_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(mfa_debugsel_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(mfa_altsel_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(mfa_pullup_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(scratchword0, ox820_sysctrl_state),
        VMSTATE_UINT32(scratchword1, ox820_sysctrl_state),
        VMSTATE_UINT32(scratchword2, ox820_sysctrl_state),
        VMSTATE_UINT32(scratchword3, ox820_sysctrl_state),
        VMSTATE_UINT32(monmux_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(gmacb_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(usbbphy_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(usbbphy_stat, ox820_sysctrl_state),
        VMSTATE_UINT32(ref300_div_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(gmaca_delay, ox820_sysctrl_state),
        VMSTATE_UINT32(gmacb_delay, ox820_sysctrl_state),
        VMSTATE_UINT32(gmac_dll, ox820_sysctrl_state),
        VMSTATE_UINT32(gmac_samp, ox820_sysctrl_state),
        VMSTATE_UINT32(amba_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(hcsl_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(usbaphy_tune, ox820_sysctrl_state),
        VMSTATE_UINT32(usbbphy_tune, ox820_sysctrl_state),
        VMSTATE_UINT32(pciea_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(pcieb_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(pciephy_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(ddramio_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(pciea_pom0_mem_addr_offset, ox820_sysctrl_state),
        VMSTATE_UINT32(pciea_pom1_mem_addr_offset, ox820_sysctrl_state),
        VMSTATE_UINT32(pciea_in0_mem_addr_start, ox820_sysctrl_state),
        VMSTATE_UINT32(pciea_in1_mem_addr_start, ox820_sysctrl_state),
        VMSTATE_UINT32(pciea_in_io_addr_start, ox820_sysctrl_state),
        VMSTATE_UINT32(pciea_in_cfg0_addr_start, ox820_sysctrl_state),
        VMSTATE_UINT32(pciea_in_cfg1_addr_start, ox820_sysctrl_state),
        VMSTATE_UINT32(pciea_in_msg_addr_start, ox820_sysctrl_state),
        VMSTATE_UINT32(pciea_in0_mem_addr_limit, ox820_sysctrl_state),
        VMSTATE_UINT32(pciea_in1_mem_addr_limit, ox820_sysctrl_state),
        VMSTATE_UINT32(pciea_in_io_addr_limit, ox820_sysctrl_state),
        VMSTATE_UINT32(pciea_in_cfg0_addr_limit, ox820_sysctrl_state),
        VMSTATE_UINT32(pciea_in_cfg1_addr_limit, ox820_sysctrl_state),
        VMSTATE_UINT32(pciea_in_msg_addr_limit, ox820_sysctrl_state),
        VMSTATE_UINT32(pciea_ahb_slave_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(pcieb_pom0_mem_addr_offset, ox820_sysctrl_state),
        VMSTATE_UINT32(pcieb_pom1_mem_addr_offset, ox820_sysctrl_state),
        VMSTATE_UINT32(pcieb_in0_mem_addr_start, ox820_sysctrl_state),
        VMSTATE_UINT32(pcieb_in1_mem_addr_start, ox820_sysctrl_state),
        VMSTATE_UINT32(pcieb_in_io_addr_start, ox820_sysctrl_state),
        VMSTATE_UINT32(pcieb_in_cfg0_addr_start, ox820_sysctrl_state),
        VMSTATE_UINT32(pcieb_in_cfg1_addr_start, ox820_sysctrl_state),
        VMSTATE_UINT32(pcieb_in_msg_addr_start, ox820_sysctrl_state),
        VMSTATE_UINT32(pcieb_in0_mem_addr_limit, ox820_sysctrl_state),
        VMSTATE_UINT32(pcieb_in1_mem_addr_limit, ox820_sysctrl_state),
        VMSTATE_UINT32(pcieb_in_io_addr_limit, ox820_sysctrl_state),
        VMSTATE_UINT32(pcieb_in_cfg0_addr_limit, ox820_sysctrl_state),
        VMSTATE_UINT32(pcieb_in_cfg1_addr_limit, ox820_sysctrl_state),
        VMSTATE_UINT32(pcieb_in_msg_addr_limit, ox820_sysctrl_state),
        VMSTATE_UINT32(pcieb_ahb_slave_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(pciea_ahb_slave_cfg_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(pcieb_ahb_slave_cfg_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(pciea_ahb_mstr_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(pcieb_ahb_mstr_ctrl, ox820_sysctrl_state),
        VMSTATE_UINT32(pciea_ahb_slv_misc_info, ox820_sysctrl_state),
        VMSTATE_UINT32(pcieb_ahb_slv_misc_info, ox820_sysctrl_state),
        VMSTATE_UINT32(plla_ctrl0, ox820_sysctrl_state),
        VMSTATE_UINT32(plla_ctrl1, ox820_sysctrl_state),
        VMSTATE_UINT32(plla_ctrl2, ox820_sysctrl_state),
        VMSTATE_UINT32(plla_ctrl3, ox820_sysctrl_state),
        VMSTATE_END_OF_LIST()
    }
};

static int ox820_sysctrl_init(SysBusDevice *dev)
{
    ox820_sysctrl_state *s = FROM_SYSBUS(ox820_sysctrl_state, dev);

    memory_region_init_io(&s->iomem, &ox820_sysctrl_ops, s, "ox820-sysctrl", 0x10000);
    sysbus_init_mmio(dev, &s->iomem);
    sysbus_init_irq(dev, &s->irq_sema_a);
    sysbus_init_irq(dev, &s->irq_sema_b);
    sysbus_init_irq(dev, &s->irq_sema_c);

    s->mfa_secsel_ctrl = 0;
    s->cken_stat = 0x10000;
    s->rsten_stat = 0x8FFEFFF2;
    s->usbmph_ctrl = 0;
    s->usbmph_stat = 0;
    s->sema_stat = 0;
    s->sema_maska_ctrl = 0;
    s->sema_maskb_ctrl = 0;
    s->sema_maskc_ctrl = 0;
    s->audio_ctrl = 0;
    s->gmaca_ctrl = 0;
    s->arm_stat = 0;
    s->usbaphy_ctrl = 0;
    s->usbaphy_stat = 0;
    s->mfa_tersel_ctrl = 0;
    s->usb_ctrl = 0;
    s->mfa_quatsel_ctrl = 0;
    s->mfa_debugsel_ctrl = 0;
    s->mfa_altsel_ctrl = 0;
    s->mfa_pullup_ctrl = 0;
    s->scratchword0 = 0;
    s->scratchword1 = 0;
    s->scratchword2 = 0;
    s->scratchword3 = 0;
    s->monmux_ctrl = 0;
    s->gmacb_ctrl = 0;
    s->usbbphy_ctrl = 0;
    s->usbbphy_stat = 0;
    s->ref300_div_ctrl = 0;
    s->gmaca_delay = 0;
    s->gmacb_delay = 0;
    s->gmac_dll = 0;
    s->gmac_samp = 0;
    s->amba_ctrl = 0;
    s->hcsl_ctrl = 0;
    s->usbaphy_tune = 0;
    s->usbbphy_tune = 0;
    s->pciea_ctrl = 0;
    s->pcieb_ctrl = 0;
    s->pciephy_ctrl = 0;
    s->ddramio_ctrl = 0;
    s->pciea_pom0_mem_addr_offset = 0;
    s->pciea_pom1_mem_addr_offset = 0;
    s->pciea_in0_mem_addr_start = 0;
    s->pciea_in1_mem_addr_start = 0;
    s->pciea_in_io_addr_start = 0;
    s->pciea_in_cfg0_addr_start = 0;
    s->pciea_in_cfg1_addr_start = 0;
    s->pciea_in_msg_addr_start = 0;
    s->pciea_in0_mem_addr_limit = 0;
    s->pciea_in1_mem_addr_limit = 0;
    s->pciea_in_io_addr_limit = 0;
    s->pciea_in_cfg0_addr_limit = 0;
    s->pciea_in_cfg1_addr_limit = 0;
    s->pciea_in_msg_addr_limit = 0;
    s->pciea_ahb_slave_ctrl = 0;
    s->pcieb_pom0_mem_addr_offset = 0;
    s->pcieb_pom1_mem_addr_offset = 0;
    s->pcieb_in0_mem_addr_start = 0;
    s->pcieb_in1_mem_addr_start = 0;
    s->pcieb_in_io_addr_start = 0;
    s->pcieb_in_cfg0_addr_start = 0;
    s->pcieb_in_cfg1_addr_start = 0;
    s->pcieb_in_msg_addr_start = 0;
    s->pcieb_in0_mem_addr_limit = 0;
    s->pcieb_in1_mem_addr_limit = 0;
    s->pcieb_in_io_addr_limit = 0;
    s->pcieb_in_cfg0_addr_limit = 0;
    s->pcieb_in_cfg1_addr_limit = 0;
    s->pcieb_in_msg_addr_limit = 0;
    s->pcieb_ahb_slave_ctrl = 0;
    s->pciea_ahb_slave_cfg_ctrl = 0;
    s->pcieb_ahb_slave_cfg_ctrl = 0;
    s->pciea_ahb_mstr_ctrl = 0;
    s->pcieb_ahb_mstr_ctrl = 0;
    s->pciea_ahb_slv_misc_info = 0;
    s->pcieb_ahb_slv_misc_info = 0;
    s->plla_ctrl0 = 0;
    s->plla_ctrl1 = 0;
    s->plla_ctrl2 = 0;
    s->plla_ctrl3 = 0;

    vmstate_register(&dev->qdev, -1, &vmstate_ox820_sysctrl, s);
    return 0;
}

static void ox820_sysctrl_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->no_user = 1;
    sdc->init = ox820_sysctrl_init;
    dc->reset = ox820_sysctrl_reset;
}

static TypeInfo ox820_sysctrl_info = {
    .name          = "ox820-sysctrl",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ox820_sysctrl_state),
    .class_init    = ox820_sysctrl_class_init,
};

static void ox820_sysctrl_register_devices(void)
{
    type_register_static(&ox820_sysctrl_info);
}

device_init(ox820_sysctrl_register_devices)
