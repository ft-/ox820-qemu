/*
 * ox820-sysctrl rst ck unit
 *
 * Written by Sven Bormann
 *
 * This code is licensed under the GPL.
 */

#include "sysbus.h"

typedef struct {
    SysBusDevice    busdev;
    MemoryRegion    iomem;
    uint32_t        ref300_divider;
} ox820_sysctrl_ref300_state;

static uint64_t ox820_sysctrl_ref300_read(void *opaque, target_phys_addr_t offset,
                           unsigned size)
{
    ox820_sysctrl_ref300_state *s = (ox820_sysctrl_ref300_state *)opaque;
    uint32_t c = 0;

    offset -= s->iomem.addr;
    switch (offset >> 2) {
    case 0x0000 >> 2:
        c = s->ref300_divider;
        break;

    default:
        return 0;
    }
    return c;
}

static void ox820_sysctrl_ref300_write(void *opaque, target_phys_addr_t offset,
                                uint64_t value, unsigned size)
{
    ox820_sysctrl_ref300_state *s = (ox820_sysctrl_ref300_state *)opaque;

    offset -= s->iomem.addr;
    switch(offset >> 2) {
    case 0x0000 >> 2:
        s->ref300_divider = value;
        break;

    default:
        break;
    }
}

static void ox820_sysctrl_ref300_reset(DeviceState *d)
{
    ox820_sysctrl_ref300_state *s = DO_UPCAST(ox820_sysctrl_ref300_state, busdev.qdev, d);

    s->ref300_divider = 0;
}

static const MemoryRegionOps ox820_sysctrl_ref300_ops = {
    .read = ox820_sysctrl_ref300_read,
    .write = ox820_sysctrl_ref300_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_ox820_sysctrl_ref300 = {
    .name = "ox820-sysctrl-ref300",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(ref300_divider, ox820_sysctrl_ref300_state),
        VMSTATE_END_OF_LIST()
    }
};

static int ox820_sysctrl_ref300_init(SysBusDevice *dev)
{
    ox820_sysctrl_ref300_state *s = FROM_SYSBUS(ox820_sysctrl_ref300_state, dev);

    memory_region_init_io(&s->iomem, &ox820_sysctrl_ref300_ops, s, "ox820-sysctrl-ref300", 0x4);
    sysbus_init_mmio(dev, &s->iomem);

    s->ref300_divider = 0;

    vmstate_register(&dev->qdev, -1, &vmstate_ox820_sysctrl_ref300, s);
    return 0;
}

static void ox820_sysctrl_ref300_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->no_user = 1;
    sdc->init = ox820_sysctrl_ref300_init;
    dc->reset = ox820_sysctrl_ref300_reset;
}

static TypeInfo ox820_sysctrl_ref300_info = {
    .name          = "ox820-sysctrl-ref300",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ox820_sysctrl_ref300_state),
    .class_init    = ox820_sysctrl_ref300_class_init,
};

static void ox820_sysctrl_ref300_register_devices(void)
{
    type_register_static(&ox820_sysctrl_ref300_info);
}

device_init(ox820_sysctrl_ref300_register_devices)
