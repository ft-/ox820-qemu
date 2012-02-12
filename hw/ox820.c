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
#include "loader.h"
#include "cpu-common.h"

/* Board init.  */

static uint32_t smpboot[] = {
  0xe3a0064e, /* mov r0, #0x04e00000 */
  0xe59040c4, /* loop: ldr r4, [r0, #0xc4] */
  0xe3540000, /* cmp r4, #0 */
  0x0afffffc, /* beq loop */
  0xe590f0c8, /* ldr pc, [r0, #0xc8] */
};

static uint32_t emptyboot[] = {
  0xbffffffe
};

static void ox820_write_secondary(CPUState *env,
                                  const struct arm_boot_info *info)
{
    int n;
    for (n = 0; n < ARRAY_SIZE(smpboot); n++) {
        smpboot[n] = tswap32(smpboot[n]);
    }
    rom_add_blob_fixed("smpboot", smpboot, sizeof(smpboot),
                       0x8000);
}

static void ox820_reset_secondary(CPUState *env,
                                  const struct arm_boot_info *info)
{
    env->regs[15] = 0x8000;
}

static struct arm_boot_info ox820_binfo = {
    .loader_start = 0x60000000,
    .write_secondary_boot = ox820_write_secondary,
    .secondary_cpu_reset_hook = ox820_reset_secondary
};

static void ox820_add_mem_alias(MemoryRegion* aliasedregion, const char* name, target_phys_addr_t tgt, uint64_t size)
{
    MemoryRegion *alias = g_new(MemoryRegion, 1);
    memory_region_init_alias(alias, name, aliasedregion, 0, size);
    memory_region_add_subregion(get_system_memory(), tgt, alias);

}

static void ox820_reset(void* opaque, int irq, int level)
{
    if(level)
    {
        qemu_system_reset(0);
    }
}

static void ox820_init(ram_addr_t ram_size,
                     const char *boot_device,
                     const char *kernel_filename, const char *kernel_cmdline,
                     const char *initrd_filename, const char *cpu_model)
{
    int num_cpus = 1; /* 1, 2 */
    uint32_t chip_config;
    CPUState *env0;
    CPUState *env1;
    //CPUState* leon;
    SysBusDevice *busdev;
    MemoryRegion *address_space_mem = get_system_memory();
    MemoryRegion *rom = g_new(MemoryRegion, 1);
    MemoryRegion *scratch = g_new(MemoryRegion, 1);
    MemoryRegion *ram = g_new(MemoryRegion, 1);
    qemu_irq rpsa_pic[32];
    qemu_irq rpsc_pic[32];
    qemu_irq gic_pic[64];
    qemu_irq *cpu_pic0;
    qemu_irq *cpu_pic1;
    DeviceState *dev;
    qemu_irq splitirq[3];
    MemoryRegion* main_1gb_region = g_new(MemoryRegion, 1);
    MemoryRegion* rpsa_region = g_new(MemoryRegion, 1);
    MemoryRegion* rpsc_region = g_new(MemoryRegion, 1);
    MemoryRegion* sysctrl_region = g_new(MemoryRegion, 1);
    MemoryRegion* smpboot_ram = g_new(MemoryRegion, 1);
    qemu_irq* reset_irq;
    int i;

    reset_irq = qemu_allocate_irqs(ox820_reset,
                                   0, 1);

    cpu_model = "arm11mpcore";
    env0 = cpu_init(cpu_model);
    if (!env0) {
        fprintf(stderr, "Unable to find CPU definition\n");
        exit(1);
    }

    if(num_cpus > 1)
    {
        env1 = cpu_init(cpu_model);
        if(!env1) {
            fprintf(stderr, "Unable to find CPU definition\n");
            exit(1);
        }
    }

    memory_region_init(main_1gb_region, "main", 0x40000000);

    memory_region_init(rpsa_region, "ox820-rpsa", 0x100000);
    memory_region_add_subregion(main_1gb_region, 0x04400000, rpsa_region);
    memory_region_init(rpsc_region, "ox820-rpsc", 0x100000);
    memory_region_add_subregion(main_1gb_region, 0x04500000, rpsc_region);
    memory_region_init(sysctrl_region, "ox820-sysctrl", 0x100000);
    memory_region_add_subregion(main_1gb_region, 0x04E00000, sysctrl_region);


    memory_region_init_ram(scratch, "ox820.scratch", 65536);
    vmstate_register_ram_global(scratch);
    memory_region_init_ram(ram, "ox820.ram", ram_size);
    vmstate_register_ram_global(ram);
    memory_region_init_ram(rom, "ox820.rom", 32768);
    vmstate_register_ram_global(rom);
    memory_region_init_ram(smpboot_ram, "smpboot", 0x100);

    /* address range 0x00008000--0x0000801F is not used on real ox820 */
    memory_region_add_subregion(main_1gb_region, 0x00008000, smpboot_ram);
    /* address range 0x00000000--0x00007FFF is occupied by a 32kB Boot Rom */
    memory_region_add_subregion(main_1gb_region, 0x00000000, rom);
    /* address range 0x20000000--0x2FFFFFFF is SDRAM region */
    memory_region_add_subregion(main_1gb_region, 0x20000000, ram);
    memory_region_add_subregion(main_1gb_region, 0x10000000, scratch);

    cpu_pic0 = arm_pic_init_cpu(env0);
    if(num_cpus > 1)
    {
        cpu_pic1 = arm_pic_init_cpu(env1);
    }

    dev = qdev_create(NULL, "mpcore-periph");
    qdev_prop_set_uint32(dev, "num-cpu", num_cpus > 1 ? 2 : 1);
    qdev_prop_set_uint32(dev, "num-irq", 64);
    qdev_init_nofail(dev);
    busdev = sysbus_from_qdev(dev);
    memory_region_add_subregion(main_1gb_region, 0x07000000, sysbus_mmio_get_region(busdev, 0));
    sysbus_connect_irq(busdev, 0, cpu_pic0[ARM_PIC_CPU_FIQ]);
    if(num_cpus > 1)
    {
        sysbus_connect_irq(busdev, 1, cpu_pic1[ARM_PIC_CPU_FIQ]);
    }

    for (i = 32; i < 64; i++) {
        gic_pic[i] = qdev_get_gpio_in(dev, i - 32);
    }

    /*=========================================================================*/
    /* RPS-A */
    splitirq[0] = qemu_irq_split(gic_pic[36], cpu_pic0[ARM_PIC_CPU_FIQ]);
    dev = qdev_create(NULL, "ox820-rps-irq");
    qdev_init_nofail(dev);
    busdev = sysbus_from_qdev(dev);
    sysbus_connect_irq(busdev, 0, gic_pic[37]);
    sysbus_connect_irq(busdev, 1, splitirq[0]);
    memory_region_add_subregion(rpsa_region, 0x00000000, sysbus_mmio_get_region(busdev, 0));

    for (i = 0; i < 32; i++) {
        rpsa_pic[i] = qdev_get_gpio_in(dev, i);
    }

    if(num_cpus > 1)
    {
        splitirq[0] = qemu_irq_split(gic_pic[34], cpu_pic1[ARM_PIC_CPU_FIQ]);
    }
    else
    {
        splitirq[0] = gic_pic[34];
    }

    dev = qdev_create(NULL, "ox820-rps-timer");
    qdev_init_nofail(dev);
    busdev = sysbus_from_qdev(dev);
    sysbus_connect_irq(busdev, 0, rpsa_pic[4]);
    memory_region_add_subregion(rpsa_region, 0x00000200, sysbus_mmio_get_region(busdev, 0));

    dev = qdev_create(NULL, "ox820-rps-timer");
    qdev_init_nofail(dev);
    busdev = sysbus_from_qdev(dev);
    sysbus_connect_irq(busdev, 0, rpsa_pic[5]);
    memory_region_add_subregion(rpsa_region, 0x00000220, sysbus_mmio_get_region(busdev, 0));

    dev = qdev_create(NULL, "ox820-rps-misc");
    qdev_init_nofail(dev);
    busdev = sysbus_from_qdev(dev);
    memory_region_add_subregion(rpsa_region, 0x000003C0, sysbus_mmio_get_region(busdev, 0));

    /*=========================================================================*/
    /* RPS-C */
    dev = qdev_create(NULL, "ox820-rps-irq");
    qdev_init_nofail(dev);
    busdev = sysbus_from_qdev(dev);
    sysbus_connect_irq(busdev, 0, gic_pic[35]);
    sysbus_connect_irq(busdev, 1, splitirq[0]);
    memory_region_add_subregion(rpsc_region, 0x00000000, sysbus_mmio_get_region(busdev, 0));

    for (i = 0; i < 32; i++) {
        rpsc_pic[i] = qdev_get_gpio_in(dev, i);
    }

    dev = qdev_create(NULL, "ox820-rps-timer");
    qdev_init_nofail(dev);
    busdev = sysbus_from_qdev(dev);
    sysbus_connect_irq(busdev, 0, rpsc_pic[4]);
    memory_region_add_subregion(rpsc_region, 0x00000200, sysbus_mmio_get_region(busdev, 0));

    dev = qdev_create(NULL, "ox820-rps-timer");
    qdev_init_nofail(dev);
    busdev = sysbus_from_qdev(dev);
    sysbus_connect_irq(busdev, 0, rpsc_pic[5]);
    memory_region_add_subregion(rpsc_region, 0x00000220, sysbus_mmio_get_region(busdev, 0));

    dev = qdev_create(NULL, "ox820-rps-misc");
    chip_config = 0;
    if(num_cpus > 1)
    {
        chip_config |= 0x00000001;
    }
    qdev_prop_set_uint32(dev, "chip-configuration", chip_config);
    qdev_init_nofail(dev);
    busdev = sysbus_from_qdev(dev);
    memory_region_add_subregion(rpsc_region, 0x000003C0, sysbus_mmio_get_region(busdev, 0));

    /*=========================================================================*/
    /* STATIC */
    dev = qdev_create(NULL, "ox820-static");
    qdev_init_nofail(dev);
    busdev = sysbus_from_qdev(dev);
    memory_region_add_subregion(main_1gb_region, 0x01C00000, sysbus_mmio_get_region(busdev, 0));

    /*=========================================================================*/
    /* UARTs */
    if (serial_hds[0]) {
        splitirq[0] = qemu_irq_split(rpsa_pic[23], rpsc_pic[23]);
        splitirq[0] = qemu_irq_split(gic_pic[55], splitirq[0]);
        serial_mm_init(main_1gb_region, 0x04200000, 0, splitirq[0], 6250000/16,
                       serial_hds[0], DEVICE_NATIVE_ENDIAN);
    }
    if (serial_hds[1]) {
        splitirq[0] = qemu_irq_split(rpsa_pic[24], rpsc_pic[24]);
        splitirq[0] = qemu_irq_split(gic_pic[56], splitirq[0]);
        serial_mm_init(main_1gb_region, 0x04300000, 0, splitirq[0], 6250000/16,
                       serial_hds[1], DEVICE_NATIVE_ENDIAN);
    }

    /*=========================================================================*/
    /* GPIOA */
    splitirq[0] = qemu_irq_split(rpsa_pic[22], rpsc_pic[22]);
    splitirq[0] = qemu_irq_split(gic_pic[53], splitirq[0]);
    dev = qdev_create(NULL, "ox820-gpio");
    qdev_prop_set_uint32(dev, "num-gpio", 32);
    qdev_init_nofail(dev);
    busdev = sysbus_from_qdev(dev);
    sysbus_connect_irq(busdev, 0, splitirq[0]);
    memory_region_add_subregion(main_1gb_region, 0x04000000, sysbus_mmio_get_region(busdev, 0));

    /*=========================================================================*/
    /* GPIOB */
    splitirq[0] = qemu_irq_split(rpsa_pic[23], rpsc_pic[23]);
    splitirq[0] = qemu_irq_split(gic_pic[54], splitirq[0]);
    dev = qdev_create(NULL, "ox820-gpio");
    qdev_prop_set_uint32(dev, "num-gpio", 18);
    qdev_init_nofail(dev);
    busdev = sysbus_from_qdev(dev);
    sysbus_connect_irq(busdev, 0, splitirq[0]);
    memory_region_add_subregion(main_1gb_region, 0x04100000, sysbus_mmio_get_region(busdev, 0));

    /*=========================================================================*/
    /* SYSCTRL */
    splitirq[0] = qemu_irq_split(rpsa_pic[10], rpsc_pic[10]);
    splitirq[0] = qemu_irq_split(gic_pic[42], splitirq[0]);
    splitirq[1] = qemu_irq_split(rpsa_pic[11], rpsc_pic[11]);
    splitirq[1] = qemu_irq_split(gic_pic[43], splitirq[1]);
    splitirq[2] = qemu_irq_split(rpsa_pic[12], rpsc_pic[12]);
    splitirq[2] = qemu_irq_split(gic_pic[44], splitirq[2]);
    dev = qdev_create(NULL, "ox820-sysctrl-sema");
    qdev_init_nofail(dev);
    busdev = sysbus_from_qdev(dev);
    sysbus_connect_irq(busdev, 0, splitirq[0]);
    sysbus_connect_irq(busdev, 1, splitirq[1]);
    sysbus_connect_irq(busdev, 2, splitirq[2]);
    memory_region_add_subregion(sysctrl_region, 0x0000004C, sysbus_mmio_get_region(busdev, 0));

    dev = qdev_create(NULL, "ox820-sysctrl-rstck");
    qdev_init_nofail(dev);
    busdev = sysbus_from_qdev(dev);
    memory_region_add_subregion(sysctrl_region, 0x00000024, sysbus_mmio_get_region(busdev, 0));
    sysbus_connect_irq(busdev, 0, reset_irq[0]);

    dev = qdev_create(NULL, "ox820-sysctrl-plla");
    qdev_init_nofail(dev);
    busdev = sysbus_from_qdev(dev);
    memory_region_add_subregion(sysctrl_region, 0x000001F0, sysbus_mmio_get_region(busdev, 0));

    dev = qdev_create(NULL, "ox820-sysctrl-mfa");
    qdev_init_nofail(dev);
    busdev = sysbus_from_qdev(dev);
    memory_region_add_subregion(sysctrl_region, 0x00000014, sysbus_mmio_get_region(busdev, 0));
    memory_region_add_subregion(sysctrl_region, 0x0000008C, sysbus_mmio_get_region(busdev, 1));
    memory_region_add_subregion(sysctrl_region, 0x00000094, sysbus_mmio_get_region(busdev, 2));

    dev = qdev_create(NULL, "ox820-sysctrl-ref300");
    qdev_init_nofail(dev);
    busdev = sysbus_from_qdev(dev);
    memory_region_add_subregion(sysctrl_region, 0x000000F8, sysbus_mmio_get_region(busdev, 0));

    dev = qdev_create(NULL, "ox820-sysctrl-scratchword");
    qdev_init_nofail(dev);
    busdev = sysbus_from_qdev(dev);
    memory_region_add_subregion(sysctrl_region, 0x000000C4, sysbus_mmio_get_region(busdev, 0));

    /*=========================================================================*/
    /* SECCTRL */
    dev = qdev_create(NULL, "ox820-secctrl");
    qdev_init_nofail(dev);
    busdev = sysbus_from_qdev(dev);
    memory_region_add_subregion(main_1gb_region, 0x04F00000, sysbus_mmio_get_region(busdev, 0));

    memory_region_add_subregion(address_space_mem, 0x00000000, main_1gb_region);
    ox820_add_mem_alias(main_1gb_region, "main.alias", 0x40000000, 0x40000000);

    /*=========================================================================*/
    /* SECCTRL */
    dev = qdev_create(NULL, "ox820-nand");
    qdev_init_nofail(dev);
    busdev = sysbus_from_qdev(dev);
    memory_region_add_subregion(main_1gb_region, 0x01000000, sysbus_mmio_get_region(busdev, 0));

    /*=========================================================================*/
    /* Boot Config */
    for (i = 0; i < ARRAY_SIZE(emptyboot); i++) {
        emptyboot[i] = tswap32(emptyboot[i]);
    }
    rom_add_blob_fixed("emptyboot", emptyboot, sizeof(emptyboot),
                       0x0000);

    ox820_binfo.ram_size = ram_size;
    ox820_binfo.kernel_filename = kernel_filename;
    ox820_binfo.kernel_cmdline = kernel_cmdline;
    ox820_binfo.initrd_filename = initrd_filename;
    ox820_binfo.nb_cpus = num_cpus;
    arm_load_kernel(env0, &ox820_binfo);
}

static QEMUMachine ox820_machine = {
    .name = "ox820",
    .desc = "OX820 (ARM11MPCore)",
    .init = ox820_init,
    .is_default = 1,
};

static void ox820_machine_init(void)
{
    qemu_register_machine(&ox820_machine);
}

machine_init(ox820_machine_init);
