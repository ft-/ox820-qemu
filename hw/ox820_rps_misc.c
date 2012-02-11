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
    uint32_t        chip_configuration;
    uint32_t        chip_id;
} ox820_rps_misc_state;


static uint64_t ox820_rps_misc_read(void *opaque, target_phys_addr_t offset,
                           unsigned size)
{
    ox820_rps_misc_state *s = (ox820_rps_misc_state*)opaque;
    uint32_t c = 0;

    offset -= s->iomem.addr;
    switch (offset >> 2) {
    case (0x03C0 - 0x3C0) >> 2:
        c = s->chip_configuration;
        break;

    case (0x03FC - 0x3C0) >> 2:
        c = s->chip_id;
        break;

    default:
        return 0;
    }
    return c;
}

static void ox820_rps_misc_write(void *opaque, target_phys_addr_t offset,
                                uint64_t value, unsigned size)
{
}

static const MemoryRegionOps ox820_rps_misc_ops = {
    .read = ox820_rps_misc_read,
    .write = ox820_rps_misc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int ox820_rps_misc_init(SysBusDevice *dev)
{
    ox820_rps_misc_state *s = FROM_SYSBUS(ox820_rps_misc_state, dev);

    memory_region_init_io(&s->iomem, &ox820_rps_misc_ops, s, "ox820-rps-misc", 0x40);
    sysbus_init_mmio(dev, &s->iomem);
    return 0;
}

static Property ox820_rps_misc_properties[] = {
    DEFINE_PROP_UINT32("chip-configuration", ox820_rps_misc_state, chip_configuration, 0x00000000),
    DEFINE_PROP_UINT32("chip-id", ox820_rps_misc_state, chip_id, 0x38323000),
    DEFINE_PROP_END_OF_LIST(),
};

static void ox820_rps_misc_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->no_user = 1;
    sdc->init = ox820_rps_misc_init;
    dc->props = ox820_rps_misc_properties;
}

static TypeInfo ox820_rps_misc_info = {
    .name          = "ox820-rps-misc",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ox820_rps_misc_state),
    .class_init    = ox820_rps_misc_class_init,
};

static void ox820_rps_misc_register_devices(void)
{
    type_register_static(&ox820_rps_misc_info);
}

device_init(ox820_rps_misc_register_devices)
