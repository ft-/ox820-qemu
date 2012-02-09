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
    uint32_t        cken_stat;
    uint32_t        rsten_stat;
} ox820_sysctrl_rstck_state;

static uint64_t ox820_sysctrl_rstck_read(void *opaque, target_phys_addr_t offset,
                           unsigned size)
{
    ox820_sysctrl_rstck_state *s = (ox820_sysctrl_rstck_state *)opaque;
    uint32_t c = 0;

    offset -= s->iomem.addr;
    switch (offset >> 2) {
    case 0x0000 >> 2:
        c = s->cken_stat;
        break;

    case 0x0004 >> 2:
        c = s->rsten_stat;
        break;

    default:
        return 0;
    }
    return c;
}

static void ox820_sysctrl_rstck_write(void *opaque, target_phys_addr_t offset,
                                uint64_t value, unsigned size)
{
    ox820_sysctrl_rstck_state *s = (ox820_sysctrl_rstck_state *)opaque;

    offset -= s->iomem.addr;
    switch(offset >> 2) {
    case 0x0008 >> 2:
        s->cken_stat |= value;
        break;

    case 0x000C >> 2:
        s->cken_stat &= (~value);
        break;

    case 0x0010 >> 2:
        s->rsten_stat |= value; /* TODO: how to pass on reset from here */
        break;

    case 0x0014 >> 2:
        s->rsten_stat &= (~value);
        break;

    default:
        break;
    }
}

static void ox820_sysctrl_rstck_reset(DeviceState *d)
{
    ox820_sysctrl_rstck_state *s = DO_UPCAST(ox820_sysctrl_rstck_state, busdev.qdev, d);

    s->cken_stat = 0x10000;
    s->rsten_stat = 0x8FFEFFF2;
}

static const MemoryRegionOps ox820_sysctrl_rstck_ops = {
    .read = ox820_sysctrl_rstck_read,
    .write = ox820_sysctrl_rstck_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_ox820_sysctrl_rstck = {
    .name = "ox820-sysctrl-rstck",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(cken_stat, ox820_sysctrl_rstck_state),
        VMSTATE_UINT32(rsten_stat, ox820_sysctrl_rstck_state),
        VMSTATE_END_OF_LIST()
    }
};

static int ox820_sysctrl_rstck_init(SysBusDevice *dev)
{
    ox820_sysctrl_rstck_state *s = FROM_SYSBUS(ox820_sysctrl_rstck_state, dev);

    memory_region_init_io(&s->iomem, &ox820_sysctrl_rstck_ops, s, "ox820-sysctrl-rstck", 0x18);
    sysbus_init_mmio(dev, &s->iomem);

    s->cken_stat = 0x10000;
    s->rsten_stat = 0x8FFEFFF2;

    vmstate_register(&dev->qdev, -1, &vmstate_ox820_sysctrl_rstck, s);
    return 0;
}

static void ox820_sysctrl_rstck_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->no_user = 1;
    sdc->init = ox820_sysctrl_rstck_init;
    dc->reset = ox820_sysctrl_rstck_reset;
}

static TypeInfo ox820_sysctrl_rstck_info = {
    .name          = "ox820-sysctrl-rstck",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ox820_sysctrl_rstck_state),
    .class_init    = ox820_sysctrl_rstck_class_init,
};

static void ox820_sysctrl_rstck_register_devices(void)
{
    type_register_static(&ox820_sysctrl_rstck_info);
}

device_init(ox820_sysctrl_rstck_register_devices)
