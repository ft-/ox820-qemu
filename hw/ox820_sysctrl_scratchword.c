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
    uint32_t        scratchword[4];
} ox820_sysctrl_scratchword_state;

static uint64_t ox820_sysctrl_scratchword_read(void *opaque, target_phys_addr_t offset,
                           unsigned size)
{
    ox820_sysctrl_scratchword_state *s = (ox820_sysctrl_scratchword_state *)opaque;
    uint32_t c = 0;

    offset -= s->iomem.addr;
    switch (offset >> 2) {
    case 0x0000 >> 2:
        c = s->scratchword[0];
        break;

    case 0x0004 >> 2:
        c = s->scratchword[1];
        break;

    case 0x0008 >> 2:
        c = s->scratchword[2];
        break;

    case 0x000C >> 2:
        c = s->scratchword[3];
        break;

    default:
        return 0;
    }
    return c;
}

static void ox820_sysctrl_scratchword_write(void *opaque, target_phys_addr_t offset,
                                uint64_t value, unsigned size)
{
    ox820_sysctrl_scratchword_state *s = (ox820_sysctrl_scratchword_state *)opaque;

    offset -= s->iomem.addr;
    switch(offset >> 2) {
    case 0x0000 >> 2:
        s->scratchword[0] = value;
        break;

    case 0x0004 >> 2:
        s->scratchword[1] = value;
        break;

    case 0x0008 >> 2:
        s->scratchword[2] = value;
        break;

    case 0x000C >> 2:
        s->scratchword[3] = value;
        break;

    default:
        break;
    }
}

static void ox820_sysctrl_scratchword_reset(DeviceState *d)
{
    ox820_sysctrl_scratchword_state *s = DO_UPCAST(ox820_sysctrl_scratchword_state, busdev.qdev, d);

    s->scratchword[0] = 0;
    s->scratchword[1] = 0;
}

static const MemoryRegionOps ox820_sysctrl_scratchword_ops = {
    .read = ox820_sysctrl_scratchword_read,
    .write = ox820_sysctrl_scratchword_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_ox820_sysctrl_scratchword = {
    .name = "ox820-sysctrl-scratchword",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(scratchword, ox820_sysctrl_scratchword_state, 4),
        VMSTATE_END_OF_LIST()
    }
};

static int ox820_sysctrl_scratchword_init(SysBusDevice *dev)
{
    ox820_sysctrl_scratchword_state *s = FROM_SYSBUS(ox820_sysctrl_scratchword_state, dev);

    memory_region_init_io(&s->iomem, &ox820_sysctrl_scratchword_ops, s, "ox820-sysctrl-scratchword", 0x10);
    sysbus_init_mmio(dev, &s->iomem);

    s->scratchword[0] = 0;
    s->scratchword[1] = 0;
    s->scratchword[2] = 0;
    s->scratchword[3] = 0;

    vmstate_register(&dev->qdev, -1, &vmstate_ox820_sysctrl_scratchword, s);
    return 0;
}

static void ox820_sysctrl_scratchword_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->no_user = 1;
    sdc->init = ox820_sysctrl_scratchword_init;
    dc->reset = ox820_sysctrl_scratchword_reset;
}

static TypeInfo ox820_sysctrl_scratchword_info = {
    .name          = "ox820-sysctrl-scratchword",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ox820_sysctrl_scratchword_state),
    .class_init    = ox820_sysctrl_scratchword_class_init,
};

static void ox820_sysctrl_scratchword_register_devices(void)
{
    type_register_static(&ox820_sysctrl_scratchword_info);
}

device_init(ox820_sysctrl_scratchword_register_devices)
