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
    uint32_t        address;
    uint32_t        data;
    uint32_t        jtag_control;
} ox820_sataphy_state;


static uint64_t ox820_sataphy_read(void *opaque, target_phys_addr_t offset,
                           unsigned size)
{
    ox820_sataphy_state *s = opaque;
    uint32_t c = 0;

    switch (offset >> 2) {
    case 0x0000 >> 2:
        c = s->address;
        break;

    case 0x0004 >> 2:
        break;

    case 0x0008 >> 2:
        c = s->jtag_control;
        break;

    default:
        return 0;
    }
    return c;
}

static void ox820_sataphy_write(void *opaque, target_phys_addr_t offset,
                                uint64_t value, unsigned size)
{
    ox820_sataphy_state *s = opaque;

    switch(offset >> 2) {

    case 0x0000 >> 2:
        s->address = value & 0xFFFF;
        break;

    case 0x0008 >> 2:
        s->jtag_control = value & 0x17;
        break;

    default:
        break;
    }
}

static void ox820_sataphy_reset(DeviceState *d)
{
    ox820_sataphy_state *s = DO_UPCAST(ox820_sataphy_state, busdev.qdev, d);

    s->address = 0;
    s->jtag_control = 0;
}

static const MemoryRegionOps ox820_sataphy_ops = {
    .read = ox820_sataphy_read,
    .write = ox820_sataphy_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};


static const VMStateDescription vmstate_ox820_sataphy = {
    .name = "ox820-sataphy",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(address, ox820_sataphy_state),
        VMSTATE_UINT32(jtag_control, ox820_sataphy_state),
        VMSTATE_END_OF_LIST()
    }
};

static int ox820_sataphy_init(SysBusDevice *dev)
{
    ox820_sataphy_state *s = FROM_SYSBUS(ox820_sataphy_state, dev);

    memory_region_init_io(&s->iomem, &ox820_sataphy_ops, s, "ox820-sataphy", 0x100000);
    sysbus_init_mmio(dev, &s->iomem);

    s->address = 0;
    s->jtag_control = 0;

    vmstate_register(&dev->qdev, -1, &vmstate_ox820_sataphy, s);
    return 0;
}

static void ox820_sataphy_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->no_user = 1;
    sdc->init = ox820_sataphy_init;
    dc->reset = ox820_sataphy_reset;
}

static TypeInfo ox820_sataphy_info = {
    .name          = "ox820-sataphy",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ox820_sataphy_state),
    .class_init    = ox820_sataphy_class_init,
};

static void ox820_sataphy_register_devices(void)
{
    type_register_static(&ox820_sataphy_info);
}

device_init(ox820_sataphy_register_devices)
