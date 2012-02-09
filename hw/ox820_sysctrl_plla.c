/*
 * ox820-sysctrl-plla dummy unit
 *
 * Written by Sven Bormann
 *
 * This code is licensed under the GPL.
 */

#include "sysbus.h"

typedef struct {
    SysBusDevice    busdev;
    MemoryRegion    iomem;
    uint32_t        plla_ctrl0;
    uint32_t        plla_ctrl1;
    uint32_t        plla_ctrl2;
    uint32_t        plla_ctrl3;
} ox820_sysctrl_plla_state;

static uint64_t ox820_sysctrl_plla_read(void *opaque, target_phys_addr_t offset,
                           unsigned size)
{
    ox820_sysctrl_plla_state *s = (ox820_sysctrl_plla_state *)opaque;
    uint32_t c = 0;

    offset -= s->iomem.addr;
    switch (offset >> 2) {
    case 0x0000 >> 2:
        c = s->plla_ctrl0;
        break;

    case 0x0004 >> 2:
        c = s->plla_ctrl1;
        break;

    case 0x0008 >> 2:
        c = s->plla_ctrl2;
        break;

    case 0x000C >> 2:
        c = s->plla_ctrl3;
        break;

    default:
        return 0;
    }
    return c;
}

static void ox820_sysctrl_plla_write(void *opaque, target_phys_addr_t offset,
                                uint64_t value, unsigned size)
{
    ox820_sysctrl_plla_state *s = (ox820_sysctrl_plla_state *)opaque;

    offset -= s->iomem.addr;
    switch(offset >> 2) {
    case 0x0000 >> 2:
        s->plla_ctrl0 = value;
        break;

    case 0x0004 >> 2:
        s->plla_ctrl1 = value;
        break;

    case 0x0008 >> 2:
        s->plla_ctrl2 = value;
        break;

    case 0x000C >> 2:
        s->plla_ctrl3 = value;
        break;

    default:
        break;
    }
}

static void ox820_sysctrl_plla_reset(DeviceState *d)
{
    ox820_sysctrl_plla_state *s = DO_UPCAST(ox820_sysctrl_plla_state, busdev.qdev, d);

    s->plla_ctrl0 = 0;
    s->plla_ctrl1 = 0;
    s->plla_ctrl2 = 0;
    s->plla_ctrl3 = 0;
}

static const MemoryRegionOps ox820_sysctrl_plla_ops = {
    .read = ox820_sysctrl_plla_read,
    .write = ox820_sysctrl_plla_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_ox820_sysctrl_plla = {
    .name = "ox820-sysctrl-plla",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(plla_ctrl0, ox820_sysctrl_plla_state),
        VMSTATE_UINT32(plla_ctrl1, ox820_sysctrl_plla_state),
        VMSTATE_UINT32(plla_ctrl2, ox820_sysctrl_plla_state),
        VMSTATE_UINT32(plla_ctrl3, ox820_sysctrl_plla_state),
        VMSTATE_END_OF_LIST()
    }
};

static int ox820_sysctrl_plla_init(SysBusDevice *dev)
{
    ox820_sysctrl_plla_state *s = FROM_SYSBUS(ox820_sysctrl_plla_state, dev);

    memory_region_init_io(&s->iomem, &ox820_sysctrl_plla_ops, s, "ox820-sysctrl-plla", 0x10);
    sysbus_init_mmio(dev, &s->iomem);

    s->plla_ctrl0 = 0;
    s->plla_ctrl1 = 0;
    s->plla_ctrl2 = 0;
    s->plla_ctrl3 = 0;

    vmstate_register(&dev->qdev, -1, &vmstate_ox820_sysctrl_plla, s);
    return 0;
}

static void ox820_sysctrl_plla_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->no_user = 1;
    sdc->init = ox820_sysctrl_plla_init;
    dc->reset = ox820_sysctrl_plla_reset;
}

static TypeInfo ox820_sysctrl_plla_info = {
    .name          = "ox820-sysctrl-plla",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ox820_sysctrl_plla_state),
    .class_init    = ox820_sysctrl_plla_class_init,
};

static void ox820_sysctrl_plla_register_devices(void)
{
    type_register_static(&ox820_sysctrl_plla_info);
}

device_init(ox820_sysctrl_plla_register_devices)
