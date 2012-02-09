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
    uint32_t        mfb_secsel_ctrl;
    uint32_t        leon_ctrl;
    uint32_t        mfb_tersel_ctrl;
    uint32_t        mfb_quatsel_ctrl;
    uint32_t        secure_ctrl;
    uint32_t        mfb_debugsel_ctrl;
    uint32_t        mfb_altsel_ctrl;
    uint32_t        mfb_pullup_ctrl;
    uint32_t        leon_debug;
    uint32_t        pllb_div_ctrl;
    uint32_t        otp_ctrl;
    uint32_t        otp_rdata;
    uint32_t        pllb_ctrl0;
    uint32_t        pllb_ctrl1;
    uint32_t        pllb_ctrl2;
    uint32_t        pllb_ctrl3;
} ox820_secctrl_state;


static uint64_t ox820_secctrl_read(void *opaque, target_phys_addr_t offset,
                           unsigned size)
{
    ox820_secctrl_state *s = (ox820_secctrl_state *)opaque;
    uint32_t c = 0;

    offset -= s->iomem.addr;
    switch (offset >> 2) {
    case 0x0014 >> 2:
        c = s->mfb_secsel_ctrl;
        break;

    case 0x0068 >> 2:
        c = s->leon_ctrl;
        break;

    case 0x008C >> 2:
        c = s->mfb_tersel_ctrl;
        break;

    case 0x0094 >> 2:
        c = s->mfb_quatsel_ctrl;
        break;

    case 0x0098 >> 2:
        c = s->secure_ctrl;
        break;

    case 0x009C >> 2:
        c = s->mfb_debugsel_ctrl;
        break;

    case 0x00A4 >> 2:
        c = s->mfb_altsel_ctrl;
        break;

    case 0x00AC >> 2:
        c = s->mfb_pullup_ctrl;
        break;

    case 0x00F0 >> 2:
        c = s->leon_debug;
        break;

    case 0x00F8 >> 2:
        c = s->pllb_div_ctrl;
        break;

    case 0x01E0 >> 2:
        c = s->otp_ctrl;
        break;

    case 0x01E4 >> 2:
        c = s->otp_rdata;
        break;

    case 0x01F0 >> 2:
        c = s->pllb_ctrl0;
        break;

    case 0x01F4 >> 2:
        c = s->pllb_ctrl1;
        break;

    case 0x01F8 >> 2:
        c = s->pllb_ctrl2;
        break;

    case 0x01FC >> 2:
        c = s->pllb_ctrl3;
        break;

    default:
        return 0;
    }
    return c;
}

static void ox820_secctrl_write(void *opaque, target_phys_addr_t offset,
                                uint64_t value, unsigned size)
{
    ox820_secctrl_state *s = (ox820_secctrl_state *)opaque;

    offset -= s->iomem.addr;
    switch(offset >> 2) {
    case 0x0014 >> 2:
        s->mfb_secsel_ctrl = value;
        break;

    case 0x0068 >> 2:
        s->leon_ctrl = value;
        break;

    case 0x008C >> 2:
        s->mfb_tersel_ctrl = value;
        break;

    case 0x0094 >> 2:
        s->mfb_quatsel_ctrl = value;
        break;

    case 0x0098 >> 2:
        s->secure_ctrl = value;
        break;

    case 0x009C >> 2:
        s->mfb_debugsel_ctrl = value;
        break;

    case 0x00A4 >> 2:
        s->mfb_altsel_ctrl = value;
        break;

    case 0x00AC >> 2:
        s->mfb_pullup_ctrl = value;
        break;

    case 0x00F0 >> 2:
        s->leon_debug = value;
        break;

    case 0x00F8 >> 2:
        s->pllb_div_ctrl = value;
        break;

    case 0x01E0 >> 2:
        s->otp_ctrl = value;
        break;

    case 0x01E4 >> 2:
        s->otp_rdata = value;
        break;

    case 0x01F0 >> 2:
        s->pllb_ctrl0 = value;
        break;

    case 0x01F4 >> 2:
        s->pllb_ctrl1 = value;
        break;

    case 0x01F8 >> 2:
        s->pllb_ctrl2 = value;
        break;

    case 0x01FC >> 2:
        s->pllb_ctrl3 = value;
        break;

    default:
        break;
    }
}

static void ox820_secctrl_reset(DeviceState *d)
{
    ox820_secctrl_state *s = DO_UPCAST(ox820_secctrl_state, busdev.qdev, d);

    s->mfb_secsel_ctrl = 0;
    s->leon_ctrl = 0;
    s->mfb_tersel_ctrl = 0;
    s->mfb_quatsel_ctrl = 0;
    s->mfb_debugsel_ctrl = 0;
    s->secure_ctrl = 0;
    s->mfb_altsel_ctrl = 0;
    s->mfb_pullup_ctrl = 0;
    s->leon_debug = 0;
    s->pllb_div_ctrl = 0;
    s->otp_ctrl = 0;
    s->otp_rdata = 0;
    s->pllb_ctrl0 = 0;
    s->pllb_ctrl1 = 0;
    s->pllb_ctrl2 = 0;
    s->pllb_ctrl3 = 0;
}

static const MemoryRegionOps ox820_secctrl_ops = {
    .read = ox820_secctrl_read,
    .write = ox820_secctrl_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_ox820_secctrl = {
    .name = "ox820-secctrl",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(mfb_secsel_ctrl, ox820_secctrl_state),
        VMSTATE_UINT32(leon_ctrl, ox820_secctrl_state),
        VMSTATE_UINT32(mfb_tersel_ctrl, ox820_secctrl_state),
        VMSTATE_UINT32(mfb_quatsel_ctrl, ox820_secctrl_state),
        VMSTATE_UINT32(mfb_debugsel_ctrl, ox820_secctrl_state),
        VMSTATE_UINT32(secure_ctrl, ox820_secctrl_state),
        VMSTATE_UINT32(mfb_altsel_ctrl, ox820_secctrl_state),
        VMSTATE_UINT32(mfb_pullup_ctrl, ox820_secctrl_state),
        VMSTATE_UINT32(leon_debug, ox820_secctrl_state),
        VMSTATE_UINT32(pllb_div_ctrl, ox820_secctrl_state),
        VMSTATE_UINT32(otp_ctrl, ox820_secctrl_state),
        VMSTATE_UINT32(otp_rdata, ox820_secctrl_state),
        VMSTATE_UINT32(pllb_ctrl0, ox820_secctrl_state),
        VMSTATE_UINT32(pllb_ctrl1, ox820_secctrl_state),
        VMSTATE_UINT32(pllb_ctrl2, ox820_secctrl_state),
        VMSTATE_UINT32(pllb_ctrl3, ox820_secctrl_state),
        VMSTATE_END_OF_LIST()
    }
};

static int ox820_secctrl_init(SysBusDevice *dev)
{
    ox820_secctrl_state *s = FROM_SYSBUS(ox820_secctrl_state, dev);

    memory_region_init_io(&s->iomem, &ox820_secctrl_ops, s, "ox820-secctrl", 0x10000);
    sysbus_init_mmio(dev, &s->iomem);

    s->mfb_secsel_ctrl = 0;
    s->leon_ctrl = 0;
    s->mfb_tersel_ctrl = 0;
    s->mfb_quatsel_ctrl = 0;
    s->mfb_debugsel_ctrl = 0;
    s->secure_ctrl = 0;
    s->mfb_altsel_ctrl = 0;
    s->mfb_pullup_ctrl = 0;
    s->leon_debug = 0;
    s->pllb_div_ctrl = 0;
    s->otp_ctrl = 0;
    s->otp_rdata = 0;
    s->pllb_ctrl0 = 0;
    s->pllb_ctrl1 = 0;
    s->pllb_ctrl2 = 0;
    s->pllb_ctrl3 = 0;

    vmstate_register(&dev->qdev, -1, &vmstate_ox820_secctrl, s);
    return 0;
}

static void ox820_secctrl_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->no_user = 1;
    sdc->init = ox820_secctrl_init;
    dc->reset = ox820_secctrl_reset;
}

static TypeInfo ox820_secctrl_info = {
    .name          = "ox820-secctrl",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ox820_secctrl_state),
    .class_init    = ox820_secctrl_class_init,
};

static void ox820_secctrl_register_devices(void)
{
    type_register_static(&ox820_secctrl_info);
}

device_init(ox820_secctrl_register_devices)
