#define OPREGBASE 0x100
#define CUSTOM_USB_EHCI
#define CUSTOM_EHCI_DEVICE SysBusDevice
#include "sysbus.h"
#include "usb-ehci.c"

static int usb_ehci_ox820_initfn(SysBusDevice *dev)
{
    EHCIState *s = DO_UPCAST(EHCIState, dev, dev);
    int i;

    // 2.2 host controller interface version
    s->mmio[0x00] = (uint8_t) OPREGBASE;
    s->mmio[0x01] = 0x00;
    s->mmio[0x02] = 0x00;
    s->mmio[0x03] = 0x01;        // HC version
    s->mmio[0x04] = NB_PORTS;    // Number of downstream ports
    s->mmio[0x05] = 0x00;        // No companion ports at present
    s->mmio[0x06] = 0x00;
    s->mmio[0x07] = 0x00;
    s->mmio[0x08] = 0x80;        // We can cache whole frame, not 64-bit capable
    s->mmio[0x09] = 0x68;        // EECP
    s->mmio[0x0a] = 0x00;
    s->mmio[0x0b] = 0x00;

    usb_bus_new(&s->bus, &ehci_bus_ops, &s->dev.qdev);
    for(i = 0; i < NB_PORTS; i++) {
        usb_register_port(&s->bus, &s->ports[i], s, i, &ehci_port_ops,
                          USB_SPEED_MASK_HIGH);
        s->ports[i].dev = 0;
    }

    s->frame_timer = qemu_new_timer_ns(vm_clock, ehci_frame_timer, s);
    QTAILQ_INIT(&s->queues);

    qemu_register_reset(ehci_reset, s);

    memory_region_init_io(&s->mem, &ehci_mem_ops, s, "ehci", MMIO_SIZE);
    sysbus_init_mmio(dev, &s->mem);
    sysbus_init_irq(dev, &s->irq);

    fprintf(stderr, "*** EHCI support is under development ***\n");

    return 0;
}

static void ox820_ehci_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    sdc->init = usb_ehci_ox820_initfn;
    dc->vmsd = &vmstate_ehci;
    dc->props = ehci_properties;
}

static TypeInfo ox820_ehci_info = {
    .name          = "usb-ehci-ox820",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(EHCIState),
    .class_init    = ox820_ehci_class_init,
};

static void ox820_ehci_register(void)
{
    type_register_static(&ox820_ehci_info);
}
device_init(ox820_ehci_register);
