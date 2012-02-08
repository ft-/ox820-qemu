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
    uint32_t        timer_load;
    uint32_t        timer_current_count;
    uint32_t        timer_control;
    uint32_t        timer_irq;
    qemu_irq        irq;
} ox820_rps_timer_state;


static void ox820_rps_timer_update(ox820_rps_timer_state* s)
{

}

static uint64_t ox820_rps_timer_read(void *opaque, target_phys_addr_t offset,
                           unsigned size)
{
    ox820_rps_timer_state *s = (ox820_rps_timer_state *)opaque;
    uint32_t c = 0;

    switch (offset >> 2) {
    case 0x0000 >> 2:
        c = s->timer_load;
        break;

    case 0x0004 >> 2:
        c = s->timer_current_count;
        break;

    case 0x0008 >> 2:
        c = s->timer_control;
        break;

    case 0x000C >> 2:
        break;

    default:
        return 0;
    }
    return c;
}

static void ox820_rps_timer_write(void *opaque, target_phys_addr_t offset,
                                uint64_t value, unsigned size)
{
    ox820_rps_timer_state *s = (ox820_rps_timer_state *)opaque;

    switch(offset >> 2) {
    case 0x0000 >> 2:
        s->timer_load = value & 0xFFFFFF;
        break;

    case 0x0004 >> 2:
        break;

    case 0x0008 >> 2:
        if((value & 0x80) && !(s->timer_control & 0x80))
        {
            s->timer_current_count = s->timer_load;
        }
        s->timer_control = value & 0xCC;
        ox820_rps_timer_update(s);
        break;

    case 0x000C >> 2:
        s->timer_irq = 0;
        ox820_rps_timer_update(s);
        break;

    default:
        break;
    }
}

static void ox820_rps_timer_reset(DeviceState *d)
{
    ox820_rps_timer_state *s = DO_UPCAST(ox820_rps_timer_state, busdev.qdev, d);

    s->timer_control = 0;
    s->timer_load = 0;
    s->timer_current_count = 0;
    s->timer_irq = 0;
    ox820_rps_timer_update(s);
}

static const MemoryRegionOps ox820_rps_timer_ops = {
    .read = ox820_rps_timer_read,
    .write = ox820_rps_timer_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_ox820_rps_timer = {
    .name = "ox820-rps-timer",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(timer_load, ox820_rps_timer_state),
        VMSTATE_UINT32(timer_current_count, ox820_rps_timer_state),
        VMSTATE_UINT32(timer_control, ox820_rps_timer_state),
        VMSTATE_UINT32(timer_irq, ox820_rps_timer_state),
        VMSTATE_END_OF_LIST()
    }
};

static int ox820_rps_timer_init(SysBusDevice *dev)
{
    ox820_rps_timer_state *s = FROM_SYSBUS(ox820_rps_timer_state, dev);

    memory_region_init_io(&s->iomem, &ox820_rps_timer_ops, s, "ox820-rps-timer", 0x10);
    sysbus_init_mmio(dev, &s->iomem);
    sysbus_init_irq(dev, &s->irq);

    s->timer_load = 0;
    s->timer_current_count = 0;
    s->timer_control = 0;
    s->timer_irq = 0;
    vmstate_register(&dev->qdev, -1, &vmstate_ox820_rps_timer, s);
    return 0;
}

static void ox820_rps_timer_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->no_user = 1;
    sdc->init = ox820_rps_timer_init;
    dc->reset = ox820_rps_timer_reset;
}

static TypeInfo ox820_rps_timer_info = {
    .name          = "ox820-rps-timer",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ox820_rps_timer_state),
    .class_init    = ox820_rps_timer_class_init,
};

static void ox820_rps_timer_register_devices(void)
{
    type_register_static(&ox820_rps_timer_info);
}

device_init(ox820_rps_timer_register_devices)
