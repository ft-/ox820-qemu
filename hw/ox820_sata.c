/*
 * ox820-sata unit
 *
 * Written by Sven Bormann
 *
 * This code is licensed under the GPL.
 */

#include "sysbus.h"
#include "ox820_dma.h"

typedef struct {
    SysBusDevice    busdev;
    MemoryRegion    iomem;
    MemoryRegion    iomem_port0;
    MemoryRegion    iomem_port1;
    MemoryRegion    iomem_ciph;
    MemoryRegion    iomem_core;
    MemoryRegion    iomem_raid;
    uint32_t        cken;
    uint32_t        rsten;
    uint32_t        num_port;
    qemu_irq        irq;
    qemu_irq        dma_irq[2];
} ox820_sata_state;

static uint64_t ox820_sata_port0_read(void *opaque, target_phys_addr_t offset,
                                      unsigned size)
{
//    ox820_sata_state *s = opaque;
    uint32_t c = 0;

    return c;
}

static void ox820_sata_port0_write(void *opaque, target_phys_addr_t offset,
                                   uint64_t value, unsigned size)
{
//    ox820_sata_state *s = opaque;

}

static uint64_t ox820_sata_port1_read(void *opaque, target_phys_addr_t offset,
                                      unsigned size)
{
//    ox820_sata_state *s = opaque;
    uint32_t c = 0;

    return c;
}

static void ox820_sata_port1_write(void *opaque, target_phys_addr_t offset,
                                   uint64_t value, unsigned size)
{
//    ox820_sata_state *s = opaque;

}

static uint64_t ox820_sata_ciph_read(void *opaque, target_phys_addr_t offset,
                                     unsigned size)
{
//    ox820_sata_state *s = opaque;
    uint32_t c = 0;

    return c;
}

static void ox820_sata_ciph_write(void *opaque, target_phys_addr_t offset,
                                  uint64_t value, unsigned size)
{
//    ox820_sata_state *s = opaque;

}

static uint64_t ox820_sata_core_read(void *opaque, target_phys_addr_t offset,
                                     unsigned size)
{
//    ox820_sata_state *s = opaque;
    uint32_t c = 0;

    return c;
}

static void ox820_sata_core_write(void *opaque, target_phys_addr_t offset,
                                  uint64_t value, unsigned size)
{
//    ox820_sata_state *s = opaque;

}

static uint64_t ox820_sata_raid_read(void *opaque, target_phys_addr_t offset,
                                     unsigned size)
{
//    ox820_sata_state *s = opaque;
    uint32_t c = 0;

    return c;
}

static void ox820_sata_raid_write(void *opaque, target_phys_addr_t offset,
                                  uint64_t value, unsigned size)
{
//    ox820_sata_state *s = opaque;

}

static void ox820_sata_reset(ox820_sata_state* s)
{

}

static void ox820_sata_reset_hnd(DeviceState *d)
{
    ox820_sata_state *s = DO_UPCAST(ox820_sata_state, busdev.qdev, d);

    ox820_sata_reset(s);
}

static void ox820_sata_set_irq(void* opaque, int irq, int level)
{
    ox820_sata_state *s = opaque;
    uint32_t oldval;
    /* 0 => reset, 1 => cken */
    switch(irq)
    {
        case 0:
            qemu_set_irq(s->dma_irq[0], level);
            oldval = s->rsten;
            if(!s->rsten && level)
            {
                ox820_sata_reset(s);
            }
            s->rsten = level ? 1 : 0;
            break;

        case 1:
            qemu_set_irq(s->dma_irq[1], level);
            oldval = s->cken;
            s->cken = level ? 1 : 0;
            if(!oldval && s->rsten && s->cken)
            {
                /* check ports */
            }
            break;
    }
}


static const MemoryRegionOps ox820_sata_port0_ops = {
    .read = ox820_sata_port0_read,
    .write = ox820_sata_port0_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps ox820_sata_port1_ops = {
    .read = ox820_sata_port1_read,
    .write = ox820_sata_port1_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};
static const MemoryRegionOps ox820_sata_ciph_ops = {
    .read = ox820_sata_ciph_read,
    .write = ox820_sata_ciph_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps ox820_sata_core_ops = {
    .read = ox820_sata_core_read,
    .write = ox820_sata_core_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps ox820_sata_raid_ops = {
    .read = ox820_sata_raid_read,
    .write = ox820_sata_raid_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_ox820_sata = {
    .name = "ox820-sata",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(cken, ox820_sata_state),
        VMSTATE_UINT32(rsten, ox820_sata_state),
        VMSTATE_END_OF_LIST()
    }
};

static void ox820_readblock(void* opaque, target_phys_addr_t physaddr, void* buf, int len)
{

}

static void ox820_writeblock(void* opaque, target_phys_addr_t physaddr, const void* buf, int len)
{

}

static int ox820_sata_init(SysBusDevice *dev)
{
    ox820_sata_state *s = FROM_SYSBUS(ox820_sata_state, dev);
    DeviceState* dma_dev;
    SysBusDevice* busdev;

    memory_region_init(&s->iomem, "ox820-sata", 0x100000);
    memory_region_init_io(&s->iomem_port0, &ox820_sata_port0_ops, s, "port0", 0x10000);
    memory_region_init_io(&s->iomem_port1, &ox820_sata_port1_ops, s, "port1", 0x10000);
    memory_region_init_io(&s->iomem_ciph, &ox820_sata_ciph_ops, s, "cipher", 0x10000);
    memory_region_init_io(&s->iomem_core, &ox820_sata_core_ops, s, "core", 0x10000);
    memory_region_init_io(&s->iomem_raid, &ox820_sata_raid_ops, s, "raid", 0x10000);

    dma_dev = ox820_dma_initialize(2, 0, 1,
                                   ox820_readblock,
                                   ox820_writeblock,
                                   s);
    /* TODO: check where both IRQs have to be connected to */
    busdev = sysbus_from_qdev(dma_dev);
    s->dma_irq[0] = qdev_get_gpio_in(dma_dev, 0);
    s->dma_irq[1] = qdev_get_gpio_in(dma_dev, 1);
    memory_region_add_subregion(&s->iomem, 0x00000, &s->iomem_port0);
    memory_region_add_subregion(&s->iomem, 0x10000, &s->iomem_port1);
    memory_region_add_subregion(&s->iomem, 0xA0000, sysbus_mmio_get_region(busdev, 0));
    memory_region_add_subregion(&s->iomem, 0xC0000, &s->iomem_ciph);
    memory_region_add_subregion(&s->iomem, 0xE0000, &s->iomem_core);
    memory_region_add_subregion(&s->iomem, 0xF0000, &s->iomem_raid);
    sysbus_init_mmio(dev, &s->iomem);
    qdev_init_gpio_in(&dev->qdev, ox820_sata_set_irq, 2);
    sysbus_init_irq(dev, &s->irq);

    vmstate_register(&dev->qdev, -1, &vmstate_ox820_sata, s);
    return 0;
}

static Property ox820_sata_properties[] = {
    DEFINE_PROP_UINT32("num-port", ox820_sata_state, num_port, 1),
    DEFINE_PROP_UINT32("cken", ox820_sata_state, cken, 0),
    DEFINE_PROP_UINT32("rsten", ox820_sata_state, rsten, 1),
    DEFINE_PROP_END_OF_LIST(),
};

static void ox820_sata_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->no_user = 1;
    sdc->init = ox820_sata_init;
    dc->reset = ox820_sata_reset_hnd;
    dc->props = ox820_sata_properties;
}

static TypeInfo ox820_sata_info = {
    .name          = "ox820-sata",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ox820_sata_state),
    .class_init    = ox820_sata_class_init,
};

static void ox820_sata_register_devices(void)
{
    type_register_static(&ox820_sata_info);
}

device_init(ox820_sata_register_devices)
