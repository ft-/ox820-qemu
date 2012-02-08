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
    uint32_t        int_raw_source;
    uint32_t        int_enabled;

    uint32_t        fiq_raw_source;
    uint32_t        fiq_enabled;
    uint32_t        fiq_select;

    qemu_irq        irq;
    qemu_irq        fiq;
} ox820_rps_irq_state;

static void ox820_rps_irq_update(ox820_rps_irq_state* s)
{
    uint32_t imask_source = s->int_enabled & s->int_raw_source;
    uint32_t fiq_mask_source = s->fiq_enabled & s->fiq_raw_source;
    qemu_set_irq(s->irq, imask_source != 0);
    qemu_set_irq(s->fiq, fiq_mask_source != 0);
}

static uint64_t ox820_rps_irq_read(void *opaque, target_phys_addr_t offset,
                           unsigned size)
{
    ox820_rps_irq_state *s = (ox820_rps_irq_state *)opaque;
    uint32_t c = 0;

    switch (offset >> 2) {
        case 0x0000 >> 2:
            c = s->int_enabled & s->int_raw_source;
            break;

        case 0x0004 >> 2:
            c = s->int_raw_source;
            break;

        case 0x0008 >> 2:
            c = s->int_enabled;
            break;

        case 0x000C >> 2:
            break;

        case 0x0010 >> 2:
            break;

        case 0x0100 >> 2:
            c = s->fiq_enabled & s->fiq_raw_source;
            break;

        case 0x0104 >> 2:
            c = s->fiq_raw_source;
            break;

        case 0x0108 >> 2:
            c = s->fiq_enabled;
            break;

        case 0x010C >> 2:
            c = 0;
            break;

        case 0x01FC >> 2:
            c = s->fiq_select;
            break;

        default:
            return 0;
    }
    return c;
}

static void ox820_rps_irq_write(void *opaque, target_phys_addr_t offset,
                                uint64_t value, unsigned size)
{
    ox820_rps_irq_state *s = (ox820_rps_irq_state *)opaque;

    switch(offset >> 2) {
        case 0x0000 >> 2:
            break;

        case 0x0004 >> 2:
            break;

        case 0x0008 >> 2:
            s->int_enabled |= value;
            ox820_rps_irq_update(s);
            break;

        case 0x000C >> 2:
            s->int_enabled &= (uint32_t)(~value);
            s->int_raw_source &= (s->int_enabled & (~2));
            ox820_rps_irq_update(s);
            break;

        case 0x0010 >> 2:
            if(value & 1)
            {
                s->int_raw_source |= 2;
                if(s->fiq_select == 1)
                {
                    s->fiq_raw_source = 1;
                }
                ox820_rps_irq_update(s);
            }
            break;

        case 0x0100 >> 2:
            break;

        case 0x0104 >> 2:
            break;

        case 0x0108 >> 2:
            s->fiq_enabled |= (value & 1);
            ox820_rps_irq_update(s);
            break;

        case 0x010C >> 2:
            s->fiq_enabled &= (~(value & 1));
            if(1 == s->fiq_select)
            {
                s->fiq_raw_source = 0;
            }
            break;

        case 0x01FC >> 2:
            s->fiq_select = value & 0x1F;
            ox820_rps_irq_update(s);
            break;

        default:
            break;
    }
}

static void ox820_rps_irq_set_irq(void *opaque, int irq, int level)
{
    ox820_rps_irq_state *s = (ox820_rps_irq_state *)opaque;

    if (level)
        s->int_raw_source |= 1u << irq;
    else
        s->int_raw_source &= ~(1u << irq);
    ox820_rps_irq_update(s);
}


static void ox820_rps_irq_reset(DeviceState *d)
{
    ox820_rps_irq_state *s = DO_UPCAST(ox820_rps_irq_state, busdev.qdev, d);

    s->fiq_select = 0;
    s->fiq_enabled = 0;
    s->int_enabled = 0;
    s->int_raw_source &= (~2);
    ox820_rps_irq_update(s);
}

static const MemoryRegionOps ox820_rps_irq_ops = {
    .read = ox820_rps_irq_read,
    .write = ox820_rps_irq_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_ox820_rps_irq = {
    .name = "ox820-rps-irq",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(int_raw_source, ox820_rps_irq_state),
        VMSTATE_UINT32(int_enabled, ox820_rps_irq_state),
        VMSTATE_UINT32(fiq_raw_source, ox820_rps_irq_state),
        VMSTATE_UINT32(fiq_enabled, ox820_rps_irq_state),
        VMSTATE_UINT32(fiq_select, ox820_rps_irq_state),
        VMSTATE_END_OF_LIST()
    }
};

static int ox820_rps_irq_init(SysBusDevice *dev)
{
    ox820_rps_irq_state *s = FROM_SYSBUS(ox820_rps_irq_state, dev);

    memory_region_init_io(&s->iomem, &ox820_rps_irq_ops, s, "ox820-rps-irq", 0x200);
    sysbus_init_mmio(dev, &s->iomem);
    qdev_init_gpio_in(&dev->qdev, ox820_rps_irq_set_irq, 32);
    sysbus_init_irq(dev, &s->irq);
    sysbus_init_irq(dev, &s->fiq);

    s->int_raw_source = 0;
    s->fiq_raw_source = 0;
    s->int_enabled = 0;
    s->fiq_enabled = 0;
    s->fiq_select = 0;
    vmstate_register(&dev->qdev, -1, &vmstate_ox820_rps_irq, s);
    return 0;
}

static void ox820_rps_irq_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->no_user = 1;
    sdc->init = ox820_rps_irq_init;
    dc->reset = ox820_rps_irq_reset;
}

static TypeInfo ox820_rps_irq_info = {
    .name          = "ox820-rps-irq",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ox820_rps_irq_state),
    .class_init    = ox820_rps_irq_class_init,
};

static void ox820_rps_irq_register_devices(void)
{
    type_register_static(&ox820_rps_irq_info);
}

device_init(ox820_rps_irq_register_devices)
