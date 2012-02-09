/*
 * ox820-sysctrl sema unit
 *
 * Written by Sven Bormann
 *
 * This code is licensed under the GPL.
 */

#include "sysbus.h"

typedef struct {
    SysBusDevice    busdev;
    MemoryRegion    iomem;
    uint32_t        sema_stat;
    uint32_t        sema_maska_ctrl;
    uint32_t        sema_maskb_ctrl;
    uint32_t        sema_maskc_ctrl;

    qemu_irq        irq_sema_a;
    qemu_irq        irq_sema_b;
    qemu_irq        irq_sema_c;
} ox820_sysctrl_sema_state;

static void ox820_sysctrl_sema_update(ox820_sysctrl_sema_state* s)
{
    qemu_set_irq(s->irq_sema_a, (s->sema_stat & s->sema_maska_ctrl) != 0);
    qemu_set_irq(s->irq_sema_b, (s->sema_stat & s->sema_maskb_ctrl) != 0);
    qemu_set_irq(s->irq_sema_c, (s->sema_stat & s->sema_maskc_ctrl) != 0);
}

static uint64_t ox820_sysctrl_sema_read(void *opaque, target_phys_addr_t offset,
                           unsigned size)
{
    ox820_sysctrl_sema_state *s = (ox820_sysctrl_sema_state *)opaque;
    uint32_t c = 0;

    switch (offset >> 2) {
    case 0x0000 >> 2:
        c = s->sema_stat;
        break;

    case 0x000C >> 2:
        c = s->sema_maska_ctrl;
        break;

    case 0x0010 >> 2:
        c = s->sema_maskb_ctrl;
        break;

    case 0x0014 >> 2:
        c = s->sema_maskc_ctrl;
        break;

    default:
        return 0;
    }
    return c;
}

static void ox820_sysctrl_sema_write(void *opaque, target_phys_addr_t offset,
                                uint64_t value, unsigned size)
{
    ox820_sysctrl_sema_state *s = (ox820_sysctrl_sema_state *)opaque;

    switch(offset >> 2) {
    case 0x0004 >> 2:
        s->sema_stat |= value;
        break;

    case 0x0008 >> 2:
        s->sema_stat &= (~value);
        break;

    case 0x000C >> 2:
        s->sema_maska_ctrl = value;
        ox820_sysctrl_sema_update(s);
        break;

    case 0x0010 >> 2:
        s->sema_maskb_ctrl = value;
        ox820_sysctrl_sema_update(s);
        break;

    case 0x0014 >> 2:
        s->sema_maskc_ctrl = value;
        ox820_sysctrl_sema_update(s);
        break;

    default:
        break;
    }
}

static void ox820_sysctrl_sema_reset(DeviceState *d)
{
    ox820_sysctrl_sema_state *s = DO_UPCAST(ox820_sysctrl_sema_state, busdev.qdev, d);

    s->sema_stat = 0;
    s->sema_maska_ctrl = 0;
    s->sema_maskb_ctrl = 0;
    s->sema_maskc_ctrl = 0;

    ox820_sysctrl_sema_update(s);
}

static const MemoryRegionOps ox820_sysctrl_sema_ops = {
    .read = ox820_sysctrl_sema_read,
    .write = ox820_sysctrl_sema_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_ox820_sysctrl_sema = {
    .name = "ox820-sysctrl-sema",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(sema_stat, ox820_sysctrl_sema_state),
        VMSTATE_UINT32(sema_maska_ctrl, ox820_sysctrl_sema_state),
        VMSTATE_UINT32(sema_maskb_ctrl, ox820_sysctrl_sema_state),
        VMSTATE_UINT32(sema_maskc_ctrl, ox820_sysctrl_sema_state),
        VMSTATE_END_OF_LIST()
    }
};

static int ox820_sysctrl_sema_init(SysBusDevice *dev)
{
    ox820_sysctrl_sema_state *s = FROM_SYSBUS(ox820_sysctrl_sema_state, dev);

    memory_region_init_io(&s->iomem, &ox820_sysctrl_sema_ops, s, "ox820-sysctrl-sema", 0x18);
    sysbus_init_mmio(dev, &s->iomem);
    sysbus_init_irq(dev, &s->irq_sema_a);
    sysbus_init_irq(dev, &s->irq_sema_b);
    sysbus_init_irq(dev, &s->irq_sema_c);

    s->sema_stat = 0;
    s->sema_maska_ctrl = 0;
    s->sema_maskb_ctrl = 0;
    s->sema_maskc_ctrl = 0;

    vmstate_register(&dev->qdev, -1, &vmstate_ox820_sysctrl_sema, s);
    return 0;
}

static void ox820_sysctrl_sema_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->no_user = 1;
    sdc->init = ox820_sysctrl_sema_init;
    dc->reset = ox820_sysctrl_sema_reset;
}

static TypeInfo ox820_sysctrl_sema_info = {
    .name          = "ox820-sysctrl-sema",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ox820_sysctrl_sema_state),
    .class_init    = ox820_sysctrl_sema_class_init,
};

static void ox820_sysctrl_sema_register_devices(void)
{
    type_register_static(&ox820_sysctrl_sema_info);
}

device_init(ox820_sysctrl_sema_register_devices)
