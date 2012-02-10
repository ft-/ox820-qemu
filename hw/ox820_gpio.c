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

    uint32_t        inputs;
    uint32_t        output_enable;
    uint32_t        irq_enable_mask;
    uint32_t        irq_event;
    uint32_t        output;
    uint32_t        debounce_enable;
    uint32_t        rising_edge_irq_enable;
    uint32_t        falling_edge_irq_enable;
    uint32_t        rising_edge_irq_events;
    uint32_t        falling_edge_irq_events;
    uint32_t        invert_enable;
    uint32_t        clock_divider;
    uint32_t        pwm_irq_timer;
    uint32_t        pull_enable;
    uint32_t        pull_sense;
    uint32_t        pwm_value[32];
    uint32_t        pwm_rr[32];

    qemu_irq        irq;
    qemu_irq        gpio_out[32];
    uint32_t        num_gpio;
} ox820_gpio_state;

static void ox820_gpio_irq_update(ox820_gpio_state* s)
{
    int irqset = 0;
    unsigned int n;
    uint32_t gpio_out = s->output_enable & s->output;
    if(s->rising_edge_irq_events & s->rising_edge_irq_enable)
    {
        irqset = 1;
    }
    if(s->falling_edge_irq_events & s->falling_edge_irq_enable)
    {
        irqset = 1;
    }
    qemu_set_irq(s->irq, irqset);
    for(n = 0; n < s->num_gpio; ++n)
    {
        qemu_set_irq(s->gpio_out[n], 0 != (gpio_out & (1u << n)));
    }
}

static uint64_t ox820_gpio_read(void *opaque, target_phys_addr_t offset,
                           unsigned size)
{
    ox820_gpio_state *s = (ox820_gpio_state *)opaque;
    uint32_t c = 0;

    switch (offset >> 2) {
    case 0x0000 >> 2:
        c = s->inputs;
        break;

    case 0x0004 >> 2:
        c = s->output_enable;
        break;

    case 0x0008 >> 2:
        c = s->irq_enable_mask;
        break;

    case 0x000C >> 2:
        c = s->rising_edge_irq_events | s->falling_edge_irq_events;
        break;

    case 0x0010 >> 2:
        c = s->output;
        break;

    case 0x0024 >> 2:
        c = s->debounce_enable;
        break;

    case 0x0028 >> 2:
        c = s->rising_edge_irq_enable;
        break;

    case 0x002C >> 2:
        c = s->falling_edge_irq_enable;
        break;

    case 0x0030 >> 2:
        c = s->rising_edge_irq_events;
        break;

    case 0x0034 >> 2:
        c = s->falling_edge_irq_events;
        break;

    case 0x003C >> 2:
        c = 0;
        if(s->rising_edge_irq_events & s->rising_edge_irq_enable)
        {
            c |= 0x1;
        }
        if(s->falling_edge_irq_events & s->falling_edge_irq_enable)
        {
            c |= 0x2;
        }
        break;

    case 0x0040 >> 2:
        c = s->invert_enable;
        break;

    case 0x0048 >> 2:
        c = s->clock_divider;
        break;

    case 0x004C >> 2:
        c = s->pwm_irq_timer;
        break;

    case 0x0050 >> 2:
        c = s->pull_enable;
        break;

    case 0x0054 >> 2:
        c = s->pull_sense;
        break;

    default:
        if(offset >= 0x0080 && offset <= 0x017F)
        {
            offset >>= 2;
            uint32_t gpio = (offset >> 1) & 31;
            switch(offset & 1)
            {
                case 0:
                    c = s->pwm_value[gpio];
                    break;

                case 1:
                    c = s->pwm_rr[gpio];
                    break;
            }
        }
        break;
    }
    return c;
}

static void ox820_gpio_write(void *opaque, target_phys_addr_t offset,
                                uint64_t value, unsigned size)
{
    ox820_gpio_state *s = (ox820_gpio_state *)opaque;

    switch (offset >> 2) {
    case 0x0004 >> 2:
        s->output_enable = value;
        ox820_gpio_irq_update(s);
        break;

    case 0x0008 >> 2:
        s->irq_enable_mask = value;
        ox820_gpio_irq_update(s);
        break;

    case 0x000C >> 2:
        s->rising_edge_irq_events &= ~value;
        s->falling_edge_irq_events &= ~value;
        ox820_gpio_irq_update(s);
        break;

    case 0x0010 >> 2:
        s->output = value;
        ox820_gpio_irq_update(s);
        break;

    case 0x0014 >> 2:
        s->output |= value;
        ox820_gpio_irq_update(s);
        break;

    case 0x0018 >> 2:
        s->output &= (~value);
        ox820_gpio_irq_update(s);
        break;

    case 0x001C >> 2:
        s->output_enable |= value;
        ox820_gpio_irq_update(s);
        break;

    case 0x0020 >> 2:
        s->output_enable &= (~value);
        ox820_gpio_irq_update(s);
        break;

    case 0x0024 >> 2:
        s->debounce_enable = value;
        break;

    case 0x0028 >> 2:
        s->rising_edge_irq_enable = value;
        ox820_gpio_irq_update(s);
        break;

    case 0x002C >> 2:
        s->falling_edge_irq_enable = value;
        ox820_gpio_irq_update(s);
        break;

    case 0x0030 >> 2:
        s->rising_edge_irq_events &= ~value;
        ox820_gpio_irq_update(s);
        break;

    case 0x0034 >> 2:
        s->falling_edge_irq_events &= ~value;
        ox820_gpio_irq_update(s);
        break;

    case 0x003C >> 2:
        break;

    case 0x0040 >> 2:
        s->invert_enable = value;
        ox820_gpio_irq_update(s);
        break;

    case 0x0048 >> 2:
        s->clock_divider = value;
        break;

    case 0x004C >> 2:
        s->pwm_irq_timer = value;
        break;

    case 0x0050 >> 2:
        s->pull_enable = value;
        break;

    case 0x0054 >> 2:
        s->pull_sense = value;
        break;

    default:
        if(offset >= 0x0080 && offset <= 0x017F)
        {
            offset >>= 2;
            uint32_t gpio = (offset >> 1) & 31;
            switch(offset & 1)
            {
                case 0:
                    s->pwm_value[gpio] = value;
                    break;

                case 1:
                    s->pwm_rr[gpio] = value;
                    break;
            }
        }
        break;
    }
}

static void ox820_gpio_set(void *opaque, int irq, int level)
{
    ox820_gpio_state *s = (ox820_gpio_state *)opaque;
    uint32_t new_val;
    uint32_t edge;

    if(level)
    {
        new_val = s->inputs | (1u << irq);
    }
    else
    {
        new_val = s->inputs &= ~(1u << irq);
    }
    edge = s->inputs ^ new_val;

    s->rising_edge_irq_events |= (edge & (new_val ^ s->invert_enable));
    s->falling_edge_irq_events |= (edge & ~(new_val ^ s->invert_enable));

    ox820_gpio_irq_update(s);
}

static void ox820_gpio_reset(DeviceState *d)
{
    ox820_gpio_state *s = DO_UPCAST(ox820_gpio_state, busdev.qdev, d);
    unsigned int i;

    s->output_enable = 0;
    s->irq_enable_mask = 0;
    s->irq_event = 0;
    s->output = 0;
    s->debounce_enable = 0;
    s->rising_edge_irq_enable = 0;
    s->falling_edge_irq_enable = 0;
    s->rising_edge_irq_events = 0;
    s->falling_edge_irq_events = 0;
    s->invert_enable = 0;
    s->clock_divider = 0;
    s->pwm_irq_timer = 0;
    s->pull_enable = 0;
    s->pull_sense = 0;

    for(i = 0; i < 32; ++i)
    {
        s->pwm_value[i] = 0;
        s->pwm_rr[i] = 0;
    }

    ox820_gpio_irq_update(s);
}

static const MemoryRegionOps ox820_gpio_ops = {
    .read = ox820_gpio_read,
    .write = ox820_gpio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_ox820_gpio = {
    .name = "ox820-gpio",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(inputs, ox820_gpio_state),
        VMSTATE_UINT32(output_enable, ox820_gpio_state),
        VMSTATE_UINT32(irq_enable_mask, ox820_gpio_state),
        VMSTATE_UINT32(irq_event, ox820_gpio_state),
        VMSTATE_UINT32(output, ox820_gpio_state),
        VMSTATE_UINT32(debounce_enable, ox820_gpio_state),
        VMSTATE_UINT32(rising_edge_irq_enable, ox820_gpio_state),
        VMSTATE_UINT32(falling_edge_irq_enable, ox820_gpio_state),
        VMSTATE_UINT32(rising_edge_irq_events, ox820_gpio_state),
        VMSTATE_UINT32(falling_edge_irq_events, ox820_gpio_state),
        VMSTATE_UINT32(invert_enable, ox820_gpio_state),
        VMSTATE_UINT32(clock_divider, ox820_gpio_state),
        VMSTATE_UINT32(pwm_irq_timer, ox820_gpio_state),
        VMSTATE_UINT32(pull_enable, ox820_gpio_state),
        VMSTATE_UINT32(pull_sense, ox820_gpio_state),
        VMSTATE_UINT32_ARRAY(pwm_value, ox820_gpio_state, 32),
        VMSTATE_UINT32_ARRAY(pwm_rr, ox820_gpio_state, 32),
        VMSTATE_END_OF_LIST()
    }
};

static int ox820_gpio_init(SysBusDevice *dev)
{
    ox820_gpio_state *s = FROM_SYSBUS(ox820_gpio_state, dev);
    unsigned int i;

    memory_region_init_io(&s->iomem, &ox820_gpio_ops, s, "ox820-gpio", 0x10000);
    sysbus_init_mmio(dev, &s->iomem);
    sysbus_init_irq(dev, &s->irq);
    if(s->num_gpio > 32)
    {
        s->num_gpio = 32;
    }
    qdev_init_gpio_in(&dev->qdev, ox820_gpio_set, s->num_gpio);
    for(i = 0; i < s->num_gpio; ++i)
    {
        sysbus_init_irq(dev, &s->gpio_out[i]);
    }


    s->output_enable = 0;
    s->irq_enable_mask = 0;
    s->irq_event = 0;
    s->output = 0;
    s->debounce_enable = 0;
    s->rising_edge_irq_enable = 0;
    s->falling_edge_irq_enable = 0;
    s->rising_edge_irq_events = 0;
    s->falling_edge_irq_events = 0;
    s->invert_enable = 0;
    s->clock_divider = 0;
    s->pwm_irq_timer = 0;
    s->pull_enable = 0;
    s->pull_sense = 0;
    s->inputs = 0;

    for(i = 0; i < 32; ++i)
    {
        s->pwm_value[i] = 0;
        s->pwm_rr[i] = 0;
    }

    vmstate_register(&dev->qdev, -1, &vmstate_ox820_gpio, s);
    return 0;
}

static Property ox820_gpio_properties[] = {
    DEFINE_PROP_UINT32("num-gpio", ox820_gpio_state, num_gpio, 32),
    DEFINE_PROP_END_OF_LIST(),
};


static void ox820_gpio_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->no_user = 1;
    sdc->init = ox820_gpio_init;
    dc->reset = ox820_gpio_reset;
    dc->props = ox820_gpio_properties;
}

static TypeInfo ox820_gpio_info = {
    .name          = "ox820-gpio",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ox820_gpio_state),
    .class_init    = ox820_gpio_class_init,
};

static void ox820_gpio_register_devices(void)
{
    type_register_static(&ox820_gpio_info);
}

device_init(ox820_gpio_register_devices)
