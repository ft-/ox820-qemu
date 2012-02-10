/*
 * ox820-nand unit
 *
 * Written by Sven Bormann
 *
 * This code is licensed under the GPL.
 */

#include "sysbus.h"
#include "blockdev.h"
#include "flash.h"

typedef struct {
    SysBusDevice    busdev;
    MemoryRegion    iomem;
    DriveInfo      *nand_drive;
    DeviceState    *nand;
} ox820_nand_state;

static int ox820_nand_cle(target_phys_addr_t offset)
{
    return (offset & (1 << 19)) != 0;
}

static int ox820_nand_ale(target_phys_addr_t offset)
{
    return (offset & (1 << 18)) != 0;
}

static uint64_t ox820_nand_read(void *opaque, target_phys_addr_t offset,
                           unsigned size)
{
    ox820_nand_state *s = (ox820_nand_state *)opaque;
    uint32_t c = 0;
    int cle = ox820_nand_cle(offset);
    int ale = ox820_nand_ale(offset);

    nand_setpins(s->nand,
                 cle,
                 ale,
                 0,
                 1, 0);
    c = nand_getio(s->nand);

    return c;
}

static void ox820_nand_write(void *opaque, target_phys_addr_t offset,
                                uint64_t value, unsigned size)
{
    ox820_nand_state *s = (ox820_nand_state *)opaque;
    int cle = ox820_nand_cle(offset);
    int ale = ox820_nand_ale(offset);

    nand_setpins(s->nand,
                 cle,
                 ale,
                 0,
                 1, 0);
    nand_setio(s->nand, value);
}

static const MemoryRegionOps ox820_nand_ops = {
    .read = ox820_nand_read,
    .write = ox820_nand_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int ox820_nand_init(SysBusDevice *dev)
{
    ox820_nand_state *s = FROM_SYSBUS(ox820_nand_state, dev);

    memory_region_init_io(&s->iomem, &ox820_nand_ops, s, "ox820-nand", 0x100000);
    sysbus_init_mmio(dev, &s->iomem);

    s->nand_drive = drive_get(IF_MTD, 0, 0);
    /* 128MB Flash */
    s->nand = nand_init(s->nand_drive ? s->nand_drive->bdrv : NULL,
                    NAND_MFR_HYNIX, 0xF1);

    return 0;
}

static void ox820_nand_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->no_user = 1;
    sdc->init = ox820_nand_init;
}

static TypeInfo ox820_nand_info = {
    .name          = "ox820-nand",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ox820_nand_state),
    .class_init    = ox820_nand_class_init,
};

static void ox820_nand_register_devices(void)
{
    type_register_static(&ox820_nand_info);
}

device_init(ox820_nand_register_devices)
