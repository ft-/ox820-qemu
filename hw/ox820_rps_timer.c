/*
 * ox820-rps unit
 *
 * Written by Sven Bormann
 *
 * This code is licensed under the GPL.
 */

#include "sysbus.h"
#include "qemu-timer.h"

typedef struct {
    SysBusDevice    busdev;
    MemoryRegion    iomem;
    uint32_t        timer_load;
    uint32_t        timer_last_load;
    uint32_t        timer_control;
    qemu_irq        irq;
    QEMUTimer*      timer;
    int64_t         time;
} ox820_rps_timer_state;


static uint32_t ox820_rps_timer_read_val(ox820_rps_timer_state* s)
{
    uint64_t distance;
    int64_t prescale;
    switch(s->timer_control & 0xC)
    {
        case 0x0: prescale = 1 * (int64_t)1000000000; break;
        case 0x4: prescale = 16 * (int64_t)1000000000; break;
        case 0x8: prescale = 256 * (int64_t)1000000000; break;
        default: prescale = 1 * (int64_t)1000000000; break; /* reserved but put a useful value here */
    }
    distance = qemu_get_clock_ns(vm_clock) - s->time;
    distance = muldiv64(distance, 6250000, prescale);

    if (distance >= s->timer_last_load)
    {
        return 0;
    }
    else
    {
        return s->timer_last_load - (uint32_t) distance;
    }
}

static void ox820_rps_timer_update(ox820_rps_timer_state* s)
{
    int64_t new_time;
    int64_t prescale;
    switch(s->timer_control & 0xC)
    {
        case 0x0: prescale = 1 * (int64_t)1000000000; break;
        case 0x4: prescale = 16 * (int64_t)1000000000; break;
        case 0x8: prescale = 256 * (int64_t)1000000000; break;
        default: prescale = 1 * (int64_t)1000000000; break; /* reserved but put a useful value here */
    }
    if(s->timer_control & 0x40)
    {
        s->timer_last_load = s->timer_load;
    }
    else
    {
        s->timer_last_load = 0xFFFFFF;
    }

    new_time = muldiv64(s->timer_last_load, prescale, 6250000) + s->time;
    qemu_mod_timer(s->timer, new_time);
    s->time = new_time;
    qemu_set_irq(s->irq, 1);
}

static void ox820_rps_timer_tick(void* opaque)
{
    ox820_rps_timer_state* s = opaque;
    ox820_rps_timer_update(s);
    qemu_set_irq(s->irq, 1);
}

static uint64_t ox820_rps_timer_read(void *opaque, target_phys_addr_t offset,
                           unsigned size)
{
    ox820_rps_timer_state *s = (ox820_rps_timer_state *)opaque;
    uint32_t c = 0;

    offset -= s->iomem.addr;
    switch (offset >> 2) {
    case 0x0000 >> 2:
        c = s->timer_load;
        break;

    case 0x0004 >> 2:
        c = ox820_rps_timer_read_val(s);
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
    uint32_t old_val;

    offset -= s->iomem.addr;
    switch(offset >> 2) {
    case 0x0000 >> 2:
        s->timer_load = value & 0xFFFFFF;
        break;

    case 0x0004 >> 2:
        break;

    case 0x0008 >> 2:
        old_val = s->timer_control;
        s->timer_control = value & 0xCC;
        if((value & 0x80) && !(old_val & 0x80))
        {
            s->time = qemu_get_clock_ns(vm_clock);
            ox820_rps_timer_update(s);
        }
        else if(!(value & 0x80) && (old_val & 0x80))
        {
            qemu_del_timer(s->timer);
        }
        break;

    case 0x000C >> 2:
        qemu_set_irq(s->irq, 0);
        break;

    default:
        break;
    }
}

static void ox820_rps_timer_reset(DeviceState *d)
{
    ox820_rps_timer_state *s = DO_UPCAST(ox820_rps_timer_state, busdev.qdev, d);

    qemu_del_timer(s->timer);

    s->timer_control = 0xC0;
    s->timer_last_load = 0;
    s->timer_load = 0xFFFF;
    qemu_set_irq(s->irq, 0);
    s->time = qemu_get_clock_ns(vm_clock);
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
        VMSTATE_UINT32(timer_last_load, ox820_rps_timer_state),
        VMSTATE_UINT32(timer_control, ox820_rps_timer_state),
        VMSTATE_END_OF_LIST()
    }
};

static int ox820_rps_timer_init(SysBusDevice *dev)
{
    ox820_rps_timer_state *s = FROM_SYSBUS(ox820_rps_timer_state, dev);

    memory_region_init_io(&s->iomem, &ox820_rps_timer_ops, s, "ox820-rps-timer", 0x10);
    sysbus_init_mmio(dev, &s->iomem);
    sysbus_init_irq(dev, &s->irq);

    s->timer_last_load = 0;
    s->timer = qemu_new_timer_ns(vm_clock, ox820_rps_timer_tick, s);
    vmstate_register(&dev->qdev, -1, &vmstate_ox820_rps_timer, s);
    s->time = qemu_get_clock_ns(vm_clock);
    if(s->timer_control & 0x80)
    {
        ox820_rps_timer_update(s);
    }
    return 0;
}

static Property ox820_rps_timer_properties[] = {
    DEFINE_PROP_UINT32("timer-control", ox820_rps_timer_state, timer_control, 0x00),
    DEFINE_PROP_UINT32("timer-load", ox820_rps_timer_state, timer_load, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void ox820_rps_timer_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->no_user = 1;
    sdc->init = ox820_rps_timer_init;
    dc->reset = ox820_rps_timer_reset;
    dc->props = ox820_rps_timer_properties;
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
