/*
 * ARM Integrator CP System emulation.
 *
 * Copyright (c) 2005-2007 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the GPL
 */

#include "sysbus.h"
#include "sysemu.h"
#include "pc.h"
#include "primecell.h"
#include "devices.h"
#include "boards.h"
#include "arm-misc.h"
#include "net.h"
#include "exec-memory.h"
#include "sysemu.h"


/* Board init.  */

static struct arm_boot_info ox820_binfo = {
    .loader_start = 0x60000000,
};

static void ox820_init(ram_addr_t ram_size,
                     const char *boot_device,
                     const char *kernel_filename, const char *kernel_cmdline,
                     const char *initrd_filename, const char *cpu_model)
{
    CPUState *env0;
    //CPUState *env1;
    //CPUState* leon;
    MemoryRegion *address_space_mem = get_system_memory();
    MemoryRegion *rom = g_new(MemoryRegion, 1);
    MemoryRegion *ram = g_new(MemoryRegion, 1);
    MemoryRegion *ram_alias1 = g_new(MemoryRegion, 1);
    MemoryRegion *ram_alias2 = g_new(MemoryRegion, 1);
    MemoryRegion *ram_alias3 = g_new(MemoryRegion, 1);
    qemu_irq rpsa_pic[32];
    qemu_irq rpsc_pic[32];
    qemu_irq *cpu_pic0;
    //qemu_irq *cpu_pic1;
    DeviceState *dev;
    //qemu_irq splitirq;
    int i;

    if (!cpu_model)
        cpu_model = "arm11mpcore";
    env0 = cpu_init(cpu_model);
    if (!env0) {
        fprintf(stderr, "Unable to find CPU definition\n");
        exit(1);
    }
    /*env1 = cpu_init(cpu_model);
    if(!env1) {
        fprintf(stderr, "Unable to find CPU definition\n");
        exit(1);
    }*/
    memory_region_init_ram(ram, "ox820.ram", ram_size);
    vmstate_register_ram_global(ram);
    memory_region_init_ram(rom, "ox820.rom", 32768);
    vmstate_register_ram_global(rom);

    /* address range 0x00000000--0x00007FFF is occupied by a 32kB Boot Rom */
    memory_region_add_subregion(address_space_mem, 0x00000000, rom);

    /* ??? On a real system the first 1Mb is mapped as SSRAM or boot flash.  */
    /* ??? RAM should repeat to fill physical memory space.  */
    /* SDRAM at address zero*/
    memory_region_add_subregion(address_space_mem, 0x20000000, ram);
    memory_region_init_alias(ram_alias1, "ram.alias1", ram, 0, ram_size);
    memory_region_add_subregion(address_space_mem, 0x60000000, ram_alias1);
    memory_region_init_alias(ram_alias2, "ram.alias2", ram, 0, ram_size);
    memory_region_add_subregion(address_space_mem, 0xA0000000, ram_alias2);
    memory_region_init_alias(ram_alias3, "ram.alias3", ram, 0, ram_size);
    memory_region_add_subregion(address_space_mem, 0xE0000000, ram_alias3);

    cpu_pic0 = arm_pic_init_cpu(env0);
    dev = sysbus_create_varargs("ox820-rps-irq", 0x44400000,
                                cpu_pic0[ARM_PIC_CPU_IRQ],
                                cpu_pic0[ARM_PIC_CPU_FIQ], NULL);
    for (i = 0; i < 32; i++) {
        rpsa_pic[i] = qdev_get_gpio_in(dev, i);
    }

    dev = sysbus_create_varargs("ox820-rps-irq", 0x44500000,
                                cpu_pic0[ARM_PIC_CPU_IRQ],
                                cpu_pic0[ARM_PIC_CPU_FIQ], NULL);
    for (i = 0; i < 32; i++) {
        rpsc_pic[i] = qdev_get_gpio_in(dev, i);
    }

    sysbus_create_simple("ox820-rps-timer", 0x44400200, rpsa_pic[4]);
    sysbus_create_simple("ox820-rps-timer", 0x44400220, rpsa_pic[5]);

    sysbus_create_simple("ox820-rps-timer", 0x44500200, rpsc_pic[4]);
    sysbus_create_simple("ox820-rps-timer", 0x44500220, rpsc_pic[5]);

    if (serial_hds[0]) {
        serial_mm_init(address_space_mem, 0x44200000, 0, rpsa_pic[23], 6250000/16,
                       serial_hds[0], DEVICE_NATIVE_ENDIAN);
    }
    if (serial_hds[1]) {
        serial_mm_init(address_space_mem, 0x44300000, 0, rpsa_pic[24], 6250000/16,
                       serial_hds[1], DEVICE_NATIVE_ENDIAN);
    }

#if 0
    //splitirq = qemu_irq_split(rpsa_pic[22], rpsc_pic[22]);
    sysbus_create_simple("ox820-rps-gpio", 0x44000000, rpsa_pic[22]);
    //splitirq = qemu_irq_split(rpsa_pic[23], rpsc_pic[23]);
    sysbus_create_simple("ox820-rps-gpio", 0x44100000, rpsa_pic[23]);
#endif
    //splitirq = qemu_irq_split(rpsa_pic[23], rpsc_pic[23]);
    sysbus_create_varargs("ox820-sysctrl", 0x44E00000, rpsa_pic[10], rpsa_pic[11], rpsa_pic[12], NULL);
    //splitirq = qemu_irq_split(rpsa_pic[23], rpsc_pic[23]);
    sysbus_create_simple("ox820-secctrl", 0x44F00000, NULL);

    ox820_binfo.ram_size = ram_size;
    ox820_binfo.kernel_filename = kernel_filename;
    ox820_binfo.kernel_cmdline = kernel_cmdline;
    ox820_binfo.initrd_filename = initrd_filename;
    arm_load_kernel(env0, &ox820_binfo);
}

static QEMUMachine ox820_machine = {
    .name = "ox820",
    .desc = "OX820 (ARM11MPCore)",
    .init = ox820_init,
    .is_default = 0,
};

static void ox820_machine_init(void)
{
    qemu_register_machine(&ox820_machine);
}

machine_init(ox820_machine_init);
