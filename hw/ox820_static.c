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
    uint32_t        static_cfg[4];
    uint32_t        ext_control_reg;
} ox820_static_state;


static uint64_t ox820_static_read(void *opaque, target_phys_addr_t offset,
                           unsigned size)
{
    ox820_static_state *s = (ox820_static_state *)opaque;
    uint32_t c = 0;

    offset -= s->iomem.addr;
    switch (offset >> 2) {
    case 0x0000 >> 2:
        c = 2;
        break;

    case 0x0004 >> 2:
        c = s->static_cfg[0];
        break;

    case 0x0008 >> 2:
        c = s->static_cfg[1];
        break;

    case 0x000C >> 2:
        c = s->static_cfg[2];
        break;

    case 0x0010 >> 2:
        c = s->static_cfg[3];
        break;

    case 0x0014 >> 2:
        c = s->ext_control_reg;
        break;

    default:
        return 0;
    }
    return c;
}

static void ox820_static_write(void *opaque, target_phys_addr_t offset,
                                uint64_t value, unsigned size)
{
    ox820_static_state *s = (ox820_static_state *)opaque;

    offset -= s->iomem.addr;
    switch(offset >> 2) {

    case 0x0004 >> 2:
        s->static_cfg[0] = value;
        break;

    case 0x0008 >> 2:
        s->static_cfg[1] = value;
        break;

    case 0x000C >> 2:
        s->static_cfg[2] = value;
        break;

    case 0x0010 >> 2:
        s->static_cfg[3] = value;
        break;

    case 0x0014 >> 2:
        s->ext_control_reg = value;
        break;

    default:
        break;
    }
}

static void ox820_static_reset(DeviceState *d)
{
    ox820_static_state *s = DO_UPCAST(ox820_static_state, busdev.qdev, d);

    s->static_cfg[0] = 0;
    s->static_cfg[1] = 0;
    s->static_cfg[2] = 0;
    s->static_cfg[3] = 0;
    s->ext_control_reg = 0;
}

static const MemoryRegionOps ox820_static_ops = {
    .read = ox820_static_read,
    .write = ox820_static_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};


static const VMStateDescription vmstate_ox820_static = {
    .name = "ox820-static",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(static_cfg, ox820_static_state, 4),
        VMSTATE_UINT32(ext_control_reg, ox820_static_state),
        VMSTATE_END_OF_LIST()
    }
};

static int ox820_static_init(SysBusDevice *dev)
{
    ox820_static_state *s = FROM_SYSBUS(ox820_static_state, dev);

    memory_region_init_io(&s->iomem, &ox820_static_ops, s, "ox820-static", 0x400000);
    sysbus_init_mmio(dev, &s->iomem);

    s->ext_control_reg = 0;
    s->static_cfg[0] = 0;
    s->static_cfg[1] = 0;
    s->static_cfg[2] = 0;
    s->static_cfg[3] = 0;

    vmstate_register(&dev->qdev, -1, &vmstate_ox820_static, s);
    return 0;
}

static void ox820_static_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->no_user = 1;
    sdc->init = ox820_static_init;
    dc->reset = ox820_static_reset;
}

static TypeInfo ox820_static_info = {
    .name          = "ox820-static",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ox820_static_state),
    .class_init    = ox820_static_class_init,
};

static void ox820_static_register_devices(void)
{
    type_register_static(&ox820_static_info);
}

device_init(ox820_static_register_devices)
