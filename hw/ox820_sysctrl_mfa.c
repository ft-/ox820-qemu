/*
 * ox820-sysctrl unit
 *
 * Written by Sven Bormann
 *
 * This code is licensed under the GPL.
 */

#include "sysbus.h"

typedef struct {
    SysBusDevice    busdev;
    MemoryRegion    iomem0;
    MemoryRegion    iomem1;
    MemoryRegion    iomem2;
    uint32_t        mfa_secsel_ctrl;
    uint32_t        mfa_tersel_ctrl;
    uint32_t        mfa_quatsel_ctrl;
    uint32_t        mfa_debugsel_ctrl;
    uint32_t        mfa_altsel_ctrl;
    uint32_t        mfa_pullup_ctrl;
} ox820_sysctrl_mfa_state;

static uint64_t ox820_sysctrl_mfa0_read(void *opaque, target_phys_addr_t offset,
                           unsigned size)
{
    ox820_sysctrl_mfa_state *s = (ox820_sysctrl_mfa_state *)opaque;
    uint32_t c = 0;

    offset -= s->iomem0.addr;
    switch (offset >> 2) {
    case 0x0000 >> 2:
        c = s->mfa_secsel_ctrl;
        break;

    default:
        return 0;
    }
    return c;
}

static void ox820_sysctrl_mfa0_write(void *opaque, target_phys_addr_t offset,
                                uint64_t value, unsigned size)
{
    ox820_sysctrl_mfa_state *s = (ox820_sysctrl_mfa_state *)opaque;

    offset -= s->iomem0.addr;
    switch(offset >> 2) {
    case 0x0000 >> 2:
        s->mfa_secsel_ctrl = value;
        break;

    default:
        break;
    }
}

static const MemoryRegionOps ox820_sysctrl_mfa0_ops = {
    .read = ox820_sysctrl_mfa0_read,
    .write = ox820_sysctrl_mfa0_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static uint64_t ox820_sysctrl_mfa1_read(void *opaque, target_phys_addr_t offset,
                           unsigned size)
{
    ox820_sysctrl_mfa_state *s = (ox820_sysctrl_mfa_state *)opaque;
    uint32_t c = 0;

    offset -= s->iomem1.addr;
    switch (offset >> 2) {
    case 0x0000 >> 2:
        c = s->mfa_tersel_ctrl;
        break;

    default:
        return 0;
    }
    return c;
}

static void ox820_sysctrl_mfa1_write(void *opaque, target_phys_addr_t offset,
                                uint64_t value, unsigned size)
{
    ox820_sysctrl_mfa_state *s = (ox820_sysctrl_mfa_state *)opaque;

    offset -= s->iomem1.addr;
    switch(offset >> 2) {
    case 0x0000 >> 2:
        s->mfa_tersel_ctrl = value;
        break;

    default:
        break;
    }
}

static const MemoryRegionOps ox820_sysctrl_mfa1_ops = {
    .read = ox820_sysctrl_mfa1_read,
    .write = ox820_sysctrl_mfa1_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static uint64_t ox820_sysctrl_mfa2_read(void *opaque, target_phys_addr_t offset,
                           unsigned size)
{
    ox820_sysctrl_mfa_state *s = (ox820_sysctrl_mfa_state *)opaque;
    uint32_t c = 0;

    offset -= s->iomem2.addr;
    switch (offset >> 2) {
    case 0x0000 >> 2:
        c = s->mfa_quatsel_ctrl;
        break;

    case 0x0004 >> 2:
        c = s->mfa_debugsel_ctrl;
        break;

    case 0x0008 >> 2:
        c = s->mfa_altsel_ctrl;
        break;

    case 0x000C >> 2:
        c = s->mfa_pullup_ctrl;
        break;

    default:
        return 0;
    }
    return c;
}

static void ox820_sysctrl_mfa2_write(void *opaque, target_phys_addr_t offset,
                                uint64_t value, unsigned size)
{
    ox820_sysctrl_mfa_state *s = (ox820_sysctrl_mfa_state *)opaque;

    offset -= s->iomem2.addr;
    switch(offset >> 2) {
    case 0x0000 >> 2:
        s->mfa_quatsel_ctrl = value;
        break;

    case 0x0004 >> 2:
        s->mfa_debugsel_ctrl = value;
        break;

    case 0x0008 >> 2:
        s->mfa_altsel_ctrl = value;
        break;

    case 0x000C >> 2:
        s->mfa_pullup_ctrl = value;
        break;

    default:
        break;
    }
}

static const MemoryRegionOps ox820_sysctrl_mfa2_ops = {
    .read = ox820_sysctrl_mfa2_read,
    .write = ox820_sysctrl_mfa2_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void ox820_sysctrl_mfa_reset(DeviceState *d)
{
    ox820_sysctrl_mfa_state *s = DO_UPCAST(ox820_sysctrl_mfa_state, busdev.qdev, d);

    s->mfa_secsel_ctrl = 0;
    s->mfa_tersel_ctrl = 0;
    s->mfa_quatsel_ctrl = 0;
    s->mfa_debugsel_ctrl = 0;
    s->mfa_altsel_ctrl = 0;
    s->mfa_pullup_ctrl = 0;
}

static const VMStateDescription vmstate_ox820_sysctrl_mfa = {
    .name = "ox820-sysctrl-mfa",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(mfa_secsel_ctrl, ox820_sysctrl_mfa_state),
        VMSTATE_UINT32(mfa_tersel_ctrl, ox820_sysctrl_mfa_state),
        VMSTATE_UINT32(mfa_quatsel_ctrl, ox820_sysctrl_mfa_state),
        VMSTATE_UINT32(mfa_debugsel_ctrl, ox820_sysctrl_mfa_state),
        VMSTATE_UINT32(mfa_altsel_ctrl, ox820_sysctrl_mfa_state),
        VMSTATE_UINT32(mfa_pullup_ctrl, ox820_sysctrl_mfa_state),
        VMSTATE_END_OF_LIST()
    }
};

static int ox820_sysctrl_mfa_init(SysBusDevice *dev)
{
    ox820_sysctrl_mfa_state *s = FROM_SYSBUS(ox820_sysctrl_mfa_state, dev);

    memory_region_init_io(&s->iomem0, &ox820_sysctrl_mfa0_ops, s, "ox820-sysctrl-mfa", 0x4);
    memory_region_init_io(&s->iomem1, &ox820_sysctrl_mfa1_ops, s, "ox820-sysctrl-mfa", 0x4);
    memory_region_init_io(&s->iomem2, &ox820_sysctrl_mfa2_ops, s, "ox820-sysctrl-mfa", 0x10);
    sysbus_init_mmio(dev, &s->iomem0);
    sysbus_init_mmio(dev, &s->iomem1);
    sysbus_init_mmio(dev, &s->iomem2);

    s->mfa_secsel_ctrl = 0;
    s->mfa_tersel_ctrl = 0;
    s->mfa_quatsel_ctrl = 0;
    s->mfa_debugsel_ctrl = 0;
    s->mfa_altsel_ctrl = 0;
    s->mfa_pullup_ctrl = 0;

    vmstate_register(&dev->qdev, -1, &vmstate_ox820_sysctrl_mfa, s);
    return 0;
}

static void ox820_sysctrl_mfa_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->no_user = 1;
    sdc->init = ox820_sysctrl_mfa_init;
    dc->reset = ox820_sysctrl_mfa_reset;
}

static TypeInfo ox820_sysctrl_mfa_info = {
    .name          = "ox820-sysctrl-mfa",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ox820_sysctrl_mfa_state),
    .class_init    = ox820_sysctrl_mfa_class_init,
};

static void ox820_sysctrl_mfa_register_devices(void)
{
    type_register_static(&ox820_sysctrl_mfa_info);
}

device_init(ox820_sysctrl_mfa_register_devices)
