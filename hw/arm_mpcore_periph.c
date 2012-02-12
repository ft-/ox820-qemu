/*
 * arm-gic unit (new implementation using GIC 1.0 specification)
 *
 * Written by Sven Bormann
 *
 * This code is licensed under the GPL.
 *
 * not yet working as such
 */

#include "sysbus.h"
#include "cpu-all.h"

#if 0
#define DPRINTF(args...) printf(args)
#else
#define DPRINTF(args...) {}
#endif

#define MAX_MPCORE_CPUS 4
#define MAX_MPCORE_IRQS 1020
#define MAX_DIST_EXT_INT_STATE_VARS ((MAX_MPCORE_IRQS + 31) >> 5) - 1

#define SCU_OFS_CONTROL         0x00

#define GIC_OFS_ICCICR          0x00
#define GIC_OFS_ICCPMR          0x04
#define GIC_OFS_ICCBPR          0x08
#define GIC_OFS_ICCIAR          0x0C
#define GIC_OFS_ICCEOIR         0x10
#define GIC_OFS_ICCRPR          0x14
#define GIC_OFS_ICCHPIR         0x18
#define GIC_OFS_ICCABPR         0x1C
#define GIC_OFS_ICCIIDR         0xFC

#define DIST_OFS_DCR            0x000
#define DIST_OFS_ICTR           0x004
#define DIST_OFS_IIDR           0x008
#define DIST_OFS_ISR_START      0x080
#define DIST_OFS_ISR_END        0x0FF
#define DIST_OFS_ISER_START     0x100
#define DIST_OFS_ISER_END       0x17F
#define DIST_OFS_ICER_START     0x180
#define DIST_OFS_ICER_END       0x1FF
#define DIST_OFS_ISPR_START     0x200
#define DIST_OFS_ISPR_END       0x27F
#define DIST_OFS_ICPR_START     0x280
#define DIST_OFS_ICPR_END       0x2FF
#define DIST_OFS_ABR_START      0x300
#define DIST_OFS_ABR_END        0x37F
#define DIST_OFS_IPR_START      0x400
#define DIST_OFS_IPR_END        0x7FB
#define DIST_OFS_IPTR_START     0x800
#define DIST_OFS_IPTR_END       0x81F
#define DIST_OFS_ICFR_START     0xC00
#define DIST_OFS_ICFR_END       0xCFF
#define DIST_OFS_SGIR           0xF00


typedef struct {
    uint32_t        icr;
    uint32_t        pmr;
    uint32_t        bpr;
    uint32_t        apr;
    uint16_t        intid;
    uint16_t        last_irq[MAX_MPCORE_IRQS];
    uint8_t         last_apr[MAX_MPCORE_IRQS];
} gic_cpu_state;

typedef struct {
    uint32_t        dcr;
    uint32_t        ier_ext[MAX_DIST_EXT_INT_STATE_VARS];
    uint32_t        abr_ext[MAX_MPCORE_CPUS][MAX_DIST_EXT_INT_STATE_VARS];
    uint32_t        intline_ext[MAX_DIST_EXT_INT_STATE_VARS];
    uint32_t        old_intline_ext[MAX_DIST_EXT_INT_STATE_VARS];
    uint32_t        icfr_ext[MAX_DIST_EXT_INT_STATE_VARS];
    uint32_t        pending_irq_ext[MAX_DIST_EXT_INT_STATE_VARS];
    uint8_t         ipr_ext[MAX_MPCORE_IRQS - 32];
    uint8_t         iptr_ext[MAX_MPCORE_IRQS - 32];
    uint32_t        fiq_pending[4];
    uint32_t        ier_int[MAX_MPCORE_CPUS];
    uint8_t         ipr_int[MAX_MPCORE_CPUS][32];
    uint8_t         iptr_int[MAX_MPCORE_CPUS][32];
    uint32_t        pending_irq_int[MAX_MPCORE_CPUS];
    uint32_t        abr_int[MAX_MPCORE_CPUS];
    uint32_t        intline_int[MAX_MPCORE_CPUS];
} gic_dist_state;

typedef struct {
    SysBusDevice    busdev;
    MemoryRegion    iomem;
    MemoryRegion    scu_iomem;
    MemoryRegion    gic_iomem[5];
    MemoryRegion    dist_iomem;
    uint32_t        num_cpu;
    uint32_t        num_irq;
    uint32_t        scu_control;
    DeviceState*    mptimer;
    qemu_irq *      timer_irq;

    gic_cpu_state   cpu[MAX_MPCORE_CPUS];
    gic_dist_state  dist;

    qemu_irq        fiq_out[MAX_MPCORE_CPUS];
    qemu_irq        irq_out[MAX_MPCORE_CPUS];
    int             fiq_state[MAX_MPCORE_CPUS];
    int             irq_state[MAX_MPCORE_CPUS];
} periph_state;

static inline int
gic_current_cpu(void)
{
  return cpu_single_env->cpu_index;
}

static inline void gic_set_cpu_irq(periph_state* s, uint32_t cpu, int level)
{
    if(level != 0)
    {
        level = 1;
    }
    if(cpu < s->num_cpu)
    {
        if(s->irq_state[cpu] != level)
        {
            s->irq_state[cpu] = level;
            DPRINTF("GIC%u: IRQ => %u\n", (unsigned int) cpu, (unsigned int) level);
            qemu_set_irq(s->irq_out[cpu], level);
        }
    }
}

static inline void gic_set_cpu_fiq(periph_state* s, uint32_t cpu, int level)
{
    if(level != 0)
    {
        level = 1;
    }
    if(cpu < s->num_cpu)
    {
        if(s->fiq_state[cpu] != level)
        {
            s->fiq_state[cpu] = level;
            DPRINTF("GIC%u: FIQ => %u\n", (unsigned int) cpu, (unsigned int) level);
            qemu_set_irq(s->fiq_out[cpu], level);
        }
    }
}

static void gic_irq_update(periph_state* s)
{
    unsigned int cpu;
    unsigned int irq;
    unsigned int n;
    uint32_t pendirq;
    int irq_out[MAX_MPCORE_CPUS] = {0};

    for(irq = 0; irq < MAX_DIST_EXT_INT_STATE_VARS; ++irq)
    {
        /* Level IRQ when icfr bit set else edge-triggered */
        s->dist.pending_irq_ext[irq] |= (s->dist.intline_ext[irq] & (s->dist.icfr_ext[irq] |
                                        (s->dist.intline_ext[irq] ^ s->dist.old_intline_ext[irq])));
        if(s->dist.intline_ext[irq] ^ s->dist.old_intline_ext[irq])
        {
            for(n = 0; n < 32; ++n)
            {
                if((s->dist.intline_ext[irq] ^ s->dist.old_intline_ext[irq]) & (1u << n))
                {
                    DPRINTF("IRQ change %u => %u (pending %u, abr %u, ier %u)\n", 32 + n + irq * 32, 0 != (s->dist.intline_ext[irq] & (1u << n)),
                            (s->dist.pending_irq_ext[irq] & (1u << n)) != 0,
                            ((s->dist.abr_ext[0][irq] | s->dist.abr_ext[1][irq] | s->dist.abr_ext[2][irq] | s->dist.abr_ext[3][irq]) & (1u << n)) != 0,
                            ((s->dist.ier_ext[irq] & (1u << n))) != 0);
                }
            }
        }
        s->dist.old_intline_ext[irq] = s->dist.intline_ext[irq];
    }

    for(cpu = 0; cpu < s->num_cpu; ++cpu)
    {
        s->dist.pending_irq_int[cpu] |= s->dist.ier_int[cpu] & s->dist.intline_int[cpu];
        pendirq = s->dist.pending_irq_int[cpu] & s->dist.ier_int[cpu] & (~s->dist.abr_int[cpu]);
        if(0 != pendirq)
        {
            for(irq = 0; irq < 32; ++irq)
            {
                if(pendirq & (1u << irq))
                {
                    uint32_t ipr = s->dist.ipr_int[cpu][irq];
                    if(ipr < s->cpu[cpu].pmr && ipr < s->cpu[cpu].apr)
                    {
                        irq_out[cpu] = 1;
                    }
                }
            }
        }

        for(irq = 0; irq < MAX_DIST_EXT_INT_STATE_VARS; ++irq)
        {
            pendirq = s->dist.ier_ext[irq] & s->dist.pending_irq_ext[irq] &
                            (~s->dist.abr_ext[0][irq]) &
                            (~s->dist.abr_ext[1][irq]) &
                            (~s->dist.abr_ext[2][irq]) &
                            (~s->dist.abr_ext[3][irq]);
            if(pendirq != 0 && (s->dist.dcr & 1))
            {
                for(n = 0; n < 32; ++n)
                {
                    uint32_t ipr = s->dist.ipr_ext[irq * 32 + n];
                    if((s->dist.iptr_ext[irq * 32 + n] & (1 << cpu)) &&
                        ipr < s->cpu[cpu].pmr &&
                        ipr < s->cpu[cpu].apr)
                    {
                        irq_out[cpu] = 1;
                    }
                }
            }
        }
    }

    for(cpu = 0; cpu < s->num_cpu; ++cpu)
    {
        if(s->cpu[cpu].icr & 1)
        {
            gic_set_cpu_irq(s, cpu, irq_out[cpu]);
        }
        else
        {
            gic_set_cpu_irq(s, cpu, 0);
        }
        gic_set_cpu_fiq(s, cpu, s->dist.fiq_pending[cpu]);
    }
}

static uint32_t gic_hpir_irq(periph_state* s, int cpu)
{
    uint16_t ipr = s->cpu[cpu].pmr;
    uint32_t intid = 0x3FF;
    unsigned int irq, n;
    uint32_t pendirq;

    if(ipr > s->cpu[cpu].apr)
    {
        ipr = s->cpu[cpu].apr;
    }

    for(cpu = 0; cpu < s->num_cpu; ++cpu)
    {
        pendirq = s->dist.pending_irq_int[cpu] & s->dist.ier_int[cpu] & (~s->dist.abr_int[cpu]);
        if(0 != pendirq)
        {
            for(n = 0; n < 32; ++n)
            {
                if(pendirq & (1u << n))
                {
                    if(ipr > s->dist.ipr_int[cpu][n])
                    {
                        ipr = s->dist.ipr_int[cpu][n];
                        intid = n;
                    }
                }
            }
        }

        for(irq = 0; irq < MAX_DIST_EXT_INT_STATE_VARS; ++irq)
        {
            pendirq = s->dist.ier_ext[irq] & s->dist.pending_irq_ext[irq] &
                            (~s->dist.abr_ext[0][irq]) &
                            (~s->dist.abr_ext[1][irq]) &
                            (~s->dist.abr_ext[2][irq]) &
                            (~s->dist.abr_ext[3][irq]);
            if(0 != pendirq)
            {
                for(n = 0; n < 32; ++n)
                {
                    if(pendirq & (1u << n))
                    {
                        if(ipr > s->dist.ipr_ext[irq * 32 + n])
                        {
                            ipr = s->dist.ipr_ext[irq * 32 + n];
                            intid = irq * 32 + n + 32;
                        }
                    }
                }
            }
        }
    }

    return intid;
}

static uint32_t gic_ack_irq(periph_state* s, int cpu)
{
    uint32_t intid = gic_hpir_irq(s, cpu);

    if(intid < s->num_irq)
    {
        s->cpu[cpu].last_irq[intid] = s->cpu[cpu].intid;
        s->cpu[cpu].last_apr[intid] = s->cpu[cpu].apr;
    }

    if(intid < 32)
    {
        s->dist.abr_int[cpu] |= (1u << intid);
        s->dist.pending_irq_int[cpu] &= ~(1u << intid);
        s->cpu[cpu].apr = s->dist.ipr_int[cpu][intid];
        s->cpu[cpu].intid = intid;
    }
    else if(intid < s->num_irq)
    {
        s->dist.abr_ext[cpu][intid / 32 - 1] |= (1u << (intid & 31));
        s->dist.pending_irq_ext[intid / 32 - 1] &= ~(1u << (intid & 31));
        s->cpu[cpu].apr = s->dist.ipr_ext[intid - 32];
        s->cpu[cpu].intid = intid;
    }
    else
    {
        intid = 0x3ff;
    }

    return intid;
}

static void gic_end_of_irq(periph_state* s, int cpu, uint32_t irq)
{
    if(s->cpu[cpu].intid != irq || irq == 0x3ff)
    {
        return;
    }
    if(irq < 32)
    {
        s->dist.abr_int[cpu] &= ~(1u << irq);
    }
    else
    {
        s->dist.abr_ext[cpu][irq / 32 - 1] &= ~(1u << (irq & 31));
    }
    s->cpu[cpu].apr = s->cpu[cpu].last_apr[irq];
    s->cpu[cpu].intid = s->cpu[cpu].last_irq[irq];

    gic_irq_update(s);
}

static uint64_t gic_read(periph_state* s, int cpu, target_phys_addr_t offset,
                         unsigned size)
{
    uint32_t c = 0;
    if(cpu < 0)
    {
        cpu = gic_current_cpu();
    }
    else if(!(s->scu_control & (0x20 << cpu)) || cpu >= s->num_cpu)
    {
        return 0;
    }

    offset &= 0xFF;
    switch (offset >> 2)
    {
        case GIC_OFS_ICCICR >> 2:
            c = s->cpu[cpu].icr;
            break;

        case GIC_OFS_ICCPMR >> 2:
            c = s->cpu[cpu].pmr;
            break;

        case GIC_OFS_ICCBPR >> 2:
            c = s->cpu[cpu].bpr;
            break;

        case GIC_OFS_ICCIAR >> 2:
            c = gic_ack_irq(s, cpu);
            DPRINTF("gic_read: CPU=%u IAR=%u\n", (unsigned int) cpu, (unsigned int) c);
            break;

        case GIC_OFS_ICCRPR >> 2:
            c = s->cpu[cpu].apr;
            break;

        case GIC_OFS_ICCHPIR >> 2:
            c = gic_hpir_irq(s, cpu);
            break;

        case GIC_OFS_ICCABPR >> 2:
            break;

        case GIC_OFS_ICCIIDR >> 2:
            c = 0x00010000;
            break;

        default:
            return 0;
    }
    return c;
}

static void gic_write(periph_state* s, int cpu, target_phys_addr_t offset,
                      uint64_t value, unsigned size)
{
    if(cpu < 0)
    {
        cpu = gic_current_cpu();
    }
    else if(!(s->scu_control & (0x20 << cpu)) || cpu >= s->num_cpu)
    {
        return;
    }
    offset &= 0xFF;
    switch(offset >> 2)
    {
        case GIC_OFS_ICCICR >> 2:
            s->cpu[cpu].icr = value & 0x01;
            DPRINTF("gic_write: CPU=%u, ICR=%x\n", cpu, (unsigned int) s->cpu[cpu].icr);
            break;

        case GIC_OFS_ICCPMR >> 2:
            s->cpu[cpu].pmr = value;
            DPRINTF("gic_write: CPU=%u, PMR=%x\n", cpu, (unsigned int) s->cpu[cpu].pmr);
            break;

        case GIC_OFS_ICCBPR >> 2:
            s->cpu[cpu].bpr = value & 3;
            DPRINTF("gic_write: CPU=%u, BPR=%x\n", cpu, (unsigned int) s->cpu[cpu].bpr);
            break;

        case GIC_OFS_ICCEOIR >> 2:
            DPRINTF("gic_write: CPU=%u, EOIR=%u\n", cpu, (unsigned int) value);
            gic_end_of_irq(s, cpu, value);
            break;

        case GIC_OFS_ICCABPR >> 2:
            break;
    }
}

#define define_gic_funcs(fnc, cpu)                                                                          \
static uint64_t gic_##fnc##_read(void* opaque, target_phys_addr_t offset, unsigned size)                    \
{                                                                                                           \
    return gic_read(opaque, cpu, offset, size);                                                             \
}                                                                                                           \
                                                                                                            \
static void gic_##fnc##_write(void* opaque, target_phys_addr_t offset, uint64_t value, unsigned size)       \
{                                                                                                           \
    gic_write(opaque, cpu, offset, value, size);                                                            \
}                                                                                                           \
static const MemoryRegionOps gic_##fnc##_ops = {                                                            \
    .read = gic_##fnc##_read,                                                                               \
    .write = gic_##fnc##_write,                                                                             \
    .endianness = DEVICE_NATIVE_ENDIAN,                                                                     \
}

define_gic_funcs(this, -1);
define_gic_funcs(0, 0);
define_gic_funcs(1, 1);
define_gic_funcs(2, 2);
define_gic_funcs(3, 3);


static uint64_t scu_read(void* opaque, target_phys_addr_t offset, unsigned size)
{
    periph_state* s = opaque;
    uint32_t c = 0;
    offset &= 0xFF;
    switch(offset >> 2)
    {
        case SCU_OFS_CONTROL >> 2:
            c = s->scu_control;
            break;
    }

    return c;
}

static void scu_write(void* opaque, target_phys_addr_t offset, uint64_t value, unsigned size)
{
    periph_state* s = opaque;
    offset &= 0xFF;
    if(!(s->scu_control & (2u << gic_current_cpu())))
    {
        return;
    }
    switch(offset >> 2)
    {
        case SCU_OFS_CONTROL >> 2:
            DPRINTF("scu_write: CPU=%u, CONTROL=%x\n", gic_current_cpu(), (unsigned int) value & 0xFFF);
            s->scu_control = value & 0x0FFF;
            break;
    }
}

static const MemoryRegionOps scu_ops = {
    .read = scu_read,
    .write = scu_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static uint64_t dist_read(void* opaque, target_phys_addr_t offset, unsigned size)
{
    int cpu = gic_current_cpu();
    periph_state* s = opaque;
    uint32_t c = 0;
    offset &= 0xFFF;
    switch(offset >> 2)
    {
        case DIST_OFS_DCR >> 2:
            break;

        case DIST_OFS_ICTR >> 2:
            c = (s->num_cpu - 1) << 5;
            c |= ((s->num_irq / 32) - 1);
            break;

        case DIST_OFS_IIDR >> 2:
            break;

        default:
            if(offset < DIST_OFS_ISR_START)
            {
                /* ignore */
            }
            else if(offset <= DIST_OFS_ISR_END)
            {
            }
            else if(offset <= DIST_OFS_ISER_END)
            {
                uint32_t bank = (offset - DIST_OFS_ISER_START) >> 2;
                if(bank == 0)
                {
                    c = s->dist.ier_int[cpu];
                }
                else
                {
                    c = s->dist.ier_ext[bank - 1];
                }
                DPRINTF("dist_read: CPU=%u, Bank=%u, IER: Value=%x\n", cpu, (unsigned int)bank, (unsigned int)c);
            }
            else if(offset <= DIST_OFS_ICER_END)
            {
            }
            else if(offset <= DIST_OFS_ISPR_END)
            {
                uint32_t bank = (offset - DIST_OFS_ISPR_START) >> 2;
                if(bank == 0)
                {
                    c = s->dist.pending_irq_int[cpu];
                }
                else
                {
                    c = s->dist.pending_irq_ext[bank - 1];
                }
                DPRINTF("dist_read: CPU=%u, Bank=%u, IPR: Value=%x\n", cpu, (unsigned int)bank, (unsigned int)c);
            }
            else if(offset <= DIST_OFS_ICPR_END)
            {

            }
            else if(offset <= DIST_OFS_ABR_END)
            {
                uint32_t bank = (offset - DIST_OFS_ABR_START) >> 2;
                if(bank == 0)
                {
                    c = s->dist.abr_int[cpu];
                }
                else
                {
                    c = s->dist.abr_ext[cpu][bank - 1];
                }
            }
            else if(offset < DIST_OFS_IPR_START)
            {
                /* ignore */
            }
            else if(offset <= DIST_OFS_IPR_END)
            {
                uint32_t bank = (offset - DIST_OFS_IPR_START) / 32;
                uint8_t* iptr;
                if(offset - DIST_OFS_IPR_START < MAX_DIST_EXT_INT_STATE_VARS)
                {
                    if(bank == 0)
                    {
                        iptr = &s->dist.ipr_int[cpu][(offset & ~3)];
                    }
                    else
                    {
                        iptr = &s->dist.ipr_ext[(offset & ~3) - 32];
                    }

                    c = iptr[0];
                    c |= (iptr[1] << 8u);
                    c |= (iptr[2] << 16u);
                    c |= (iptr[3] << 24u);
                }
            }
            else if(offset < DIST_OFS_IPTR_START)
            {
                /* ignore */
            }
            else if(offset <= DIST_OFS_IPTR_END)
            {
                uint32_t bank = (offset - DIST_OFS_IPTR_START) / 32;
                uint8_t* iptr;
                if(offset - DIST_OFS_IPTR_START < MAX_DIST_EXT_INT_STATE_VARS)
                {
                    if(bank == 0)
                    {
                        iptr = &s->dist.iptr_int[cpu][(offset & ~3)];
                    }
                    else
                    {
                        iptr = &s->dist.iptr_ext[(offset & ~3) - 32];
                    }

                    c = iptr[0];
                    c |= (iptr[1] << 8u);
                    c |= (iptr[2] << 16u);
                    c |= (iptr[3] << 24u);
                }
            }
            else if(offset < DIST_OFS_ICFR_START)
            {
                /* ignore */
            }
            else if(offset <= DIST_OFS_ICFR_END)
            {
                uint32_t bank = (offset - DIST_OFS_ICFR_START) >> 3;
                unsigned int n;
                if(0 != bank && bank < MAX_DIST_EXT_INT_STATE_VARS)
                {
                    --bank;
                    for(n = (offset & 2) * 8; n < (offset & 2) * 8 + 16; ++n)
                    {
                        c |= ((s->dist.icfr_ext[bank] & (0x00000001 << n)) << (1 + n));
                    }
                }
            }
            break;
    }
    return c;
}

static void dist_write(void* opaque, target_phys_addr_t offset, uint64_t value, unsigned size)
{
    periph_state* s = opaque;
    int cpu = gic_current_cpu();
    offset &= 0xFFF;
    DPRINTF("dist_write: CPU=%u, Offset=%x, Value=%x\n", cpu, (unsigned int) offset, (unsigned int) value);
    switch(offset >> 2)
    {
        case DIST_OFS_DCR >> 2:
            s->dist.dcr = value & 1;
            DPRINTF("dist_write: CPU=%u, DCR=%x\n", cpu, (unsigned int) value);
            break;

        case DIST_OFS_ICTR >> 2:
            break;

        case DIST_OFS_IIDR >> 2:
            break;

        case DIST_OFS_SGIR >> 2:
            {
                unsigned int n;
                switch((value >> 24) & 0x3)
                {
                    case 0:
                        for(n = 0; n < s->num_cpu; ++n)
                        {
                            s->dist.pending_irq_int[n] |= (1u << (value & 0xF));
                        }
                        break;

                    case 1:
                        for(n = 0; n < s->num_cpu; ++n)
                        {
                            if(cpu != n)
                            {
                                s->dist.pending_irq_int[n] |= (1u << (value & 0xF));
                            }
                        }
                        break;

                    case 2:
                        s->dist.pending_irq_int[cpu] |= (1u << (value & 0xF));
                        break;
                }
                gic_irq_update(s);
            }
            break;

        default:
            if(offset < DIST_OFS_ISR_START)
            {
                /* ignore */
            }
            else if(offset <= DIST_OFS_ISR_END)
            {
            }
            else if(offset <= DIST_OFS_ISER_END)
            {
                uint32_t bank = (offset - DIST_OFS_ISER_START) >> 2;
                DPRINTF("dist_write: CPU=%u, ISER%u=%x\n", cpu, (unsigned int) bank, (unsigned int) value);
                if(bank == 0)
                {
                    s->dist.ier_int[cpu] |= value;
                }
                else
                {
                     s->dist.ier_ext[bank - 1] |= value;
                }
                gic_irq_update(s);
            }
            else if(offset <= DIST_OFS_ICER_END)
            {
                uint32_t bank = (offset - DIST_OFS_ICER_START) >> 2;
                DPRINTF("dist_write: CPU=%u, ICER%u=%x\n", cpu, (unsigned int) bank, (unsigned int) value);
                if(bank == 0)
                {
                    s->dist.ier_int[cpu] &= ~value;
                }
                else
                {
                     s->dist.ier_ext[bank - 1] &= ~value;
                }
                gic_irq_update(s);
            }
            else if(offset <= DIST_OFS_ISPR_END)
            {
                uint32_t bank = (offset - DIST_OFS_ISPR_START) >> 2;
                DPRINTF("dist_write: CPU=%u, ISPR%u=%x\n", cpu, (unsigned int) bank, (unsigned int) value);
                if(bank == 0)
                {
//                    s->dist.pending_irq_int[cpu] |= value;
                }
                else
                {
                     s->dist.pending_irq_ext[bank - 1] |= value;
                }
                gic_irq_update(s);
            }
            else if(offset <= DIST_OFS_ICPR_END)
            {
                uint32_t bank = (offset - DIST_OFS_ICPR_START) >> 2;
                DPRINTF("dist_write: CPU=%u, ICPR%u=%x\n", cpu, (unsigned int) bank, (unsigned int) value);
                if(bank == 0)
                {
                    s->dist.pending_irq_int[cpu] &= ~value;
                }
                else
                {
                     s->dist.pending_irq_ext[bank - 1] &= ~value;
                }
                gic_irq_update(s);
            }
            else if(offset <= DIST_OFS_ABR_END)
            {
            }
            else if(offset < DIST_OFS_IPR_START)
            {
                /* ignore */
            }
            else if(offset <= DIST_OFS_IPR_END)
            {
                uint32_t bank = (offset - DIST_OFS_IPR_START) / 32;
                uint8_t* iptr;
                DPRINTF("dist_write: CPU=%u, IPR%u=%x\n", cpu, (unsigned int) (offset - DIST_OFS_IPR_START) / 4, (unsigned int) value);
                if(bank == 0)
                {
                    iptr = &s->dist.ipr_int[cpu][offset - DIST_OFS_IPR_START];
                }
                else
                {
                    iptr = &s->dist.ipr_ext[offset - DIST_OFS_IPR_START - 32];
                }

                while(size-- != 0)
                {
                    *(iptr++) = (uint8_t) value;
                    value >>= 8;
                }
                gic_irq_update(s);
            }
            else if(offset < DIST_OFS_IPTR_START)
            {
                /* ignore */
            }
            else if(offset <= DIST_OFS_IPTR_END)
            {
                uint32_t bank = (offset - DIST_OFS_IPTR_START) / 32;
                uint8_t* iptr;
                DPRINTF("dist_write: CPU=%u, IPTR%u=%x\n", cpu, (unsigned int) (offset - DIST_OFS_IPTR_START) / 4, (unsigned int) value);
                if(bank == 0)
                {
                }
                else
                {
                    iptr = &s->dist.iptr_ext[offset - DIST_OFS_IPTR_START - 32];

                    while(size-- != 0)
                    {
                        *(iptr++) = (uint8_t) value;
                        value >>= 8;
                    }
                    gic_irq_update(s);
                }
            }
            else if(offset < DIST_OFS_ICFR_START)
            {
                /* ignore */
            }
            else if(offset <= DIST_OFS_ICFR_END)
            {
                uint32_t bank = (offset - DIST_OFS_ICFR_START) >> 2;
                unsigned int n;
                DPRINTF("dist_write: CPU=%u, ICFR%u=%x\n", cpu, (unsigned int) (offset - DIST_OFS_ICFR_START) / 4, (unsigned int) value);
                if(bank < 2)
                {

                }
                else
                {
                    bank -= 2;
                    if(bank & 1)
                    {
                        s->dist.icfr_ext[bank >> 1] &= 0x0000FFFF;
                        for(n = 0; n < 16; ++n)
                        {
                            if(value & (2 << (2 * n)))
                            {
                                s->dist.icfr_ext[bank >> 1] |= (0x10000 << n);
                            }
                        }
                    }
                    else
                    {
                        s->dist.icfr_ext[bank >> 1] &= 0xFFFF0000;
                        for(n = 0; n < 16; ++n)
                        {
                            if(value & (2 << (2 * n)))
                            {
                                s->dist.icfr_ext[bank >> 1] |= (0x1 << n);
                            }
                        }
                    }
                }
            }
            break;
    }
}

static const MemoryRegionOps dist_ops = {
    .read = dist_read,
    .write = dist_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void periph_set_gic_irq(periph_state* s, int cpu, int irq, int level)
{
    if(irq < 0)
    {
    }
    else if(irq < 32)
    {
        if(cpu > MAX_MPCORE_CPUS)
        {
            return;
        }
        if(level)
        {
            s->dist.intline_int[cpu] |= (1u << irq);
        }
        else
        {
            s->dist.intline_int[cpu] &= ~(1u << irq);
        }
    }
    else if(irq < MAX_MPCORE_IRQS)
    {
        irq -= 32;
        if(level)
        {
            s->dist.intline_ext[irq >> 5] |= (1u << (irq & 0x1f));
        }
        else
        {
            s->dist.intline_ext[irq >> 5] &= ~(1u << (irq & 0x1f));
        }
    }
    gic_irq_update(s);
}

static void periph_set_legacy_fiq(void *opaque, int irq, int level)
{
    periph_state *s = (periph_state *)opaque;
    if(irq >= 0 && irq < s->num_cpu)
    {
        s->dist.fiq_pending[irq] = level;
    }
    gic_irq_update(s);
}

static void periph_set_irq(void *opaque, int irq, int level)
{
    periph_state *s = (periph_state *)opaque;
    unsigned int num_ext_irq = s->num_irq - 32;
    if(irq >= num_ext_irq + s->num_cpu * 2)
    {
    }
    else if(irq >= num_ext_irq + s->num_cpu)
    {
        periph_set_legacy_fiq(s, irq - num_ext_irq - s->num_cpu, level);
    }
    else if(irq >= num_ext_irq)
    {
        periph_set_gic_irq(s, irq - num_ext_irq, 31, level);
    }
    else
    {
        irq += 32;
        periph_set_gic_irq(s, 0, irq, level);
    }
}

static void periph_mptimer_set_irq(void *opaque, int irq, int level)
{
    periph_state* s = (periph_state*) opaque;
    periph_set_gic_irq(s, irq >> 1, 29 + (irq & 1), level);
}

static void periph_reset(DeviceState *d)
{
    unsigned int n;
    periph_state *s = DO_UPCAST(periph_state, busdev.qdev, d);

    s->scu_control = 0xFFE;

    for(n = 0; n < s->num_cpu; ++n)
    {
        s->cpu[n].icr = 0;
        s->cpu[n].pmr = 0;
        s->cpu[n].bpr = 0;
        s->dist.ier_int[n] = 0;
    }
    for(n = 0; n < MAX_DIST_EXT_INT_STATE_VARS; ++n)
    {
        s->dist.ier_ext[n] = 0;
        s->dist.ipr_ext[n] = 0;
        s->dist.icfr_ext[n] = 0;
    }

    /* make iptr default to all valid cpu interfaces set */
    for(n = 32; n < s->num_irq; ++n)
    {
        s->dist.iptr_ext[n - 32] = (1 << s->num_cpu) - 1;
    }

    gic_irq_update(s);
}

#if 0
static const VMStateDescription vmstate_periph_irq = {
    .name = "mpcore-periph",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(int_raw_source, ox820_rps_irq_state),
        VMSTATE_UINT32(int_enabled, ox820_rps_irq_state),
        VMSTATE_UINT32(fiq_enabled, ox820_rps_irq_state),
        VMSTATE_UINT32(fiq_select, ox820_rps_irq_state),
        VMSTATE_END_OF_LIST()
    }
};
#endif

static int periph_init(SysBusDevice *dev)
{
    periph_state *s = FROM_SYSBUS(periph_state, dev);
    SysBusDevice *busdev;
    unsigned int n, j;

    if(s->num_cpu < 1 || s->num_cpu > MAX_MPCORE_CPUS)
    {
        hw_error("Unsupported number of cpus %u\n", s->num_cpu);
        return 1;
    }

    if(s->num_irq < 32 || s->num_irq > MAX_MPCORE_IRQS)
    {
        hw_error("Unsupported number of irqs %u\n", s->num_irq);
        return 1;
    }

    memory_region_init(&s->iomem, "mpcore-periph", 0x2000);
    memory_region_init_io(&s->scu_iomem, &scu_ops, s, "scu", 0x100);
    memory_region_init_io(&s->gic_iomem[4], &gic_this_ops, s, "gic_local", 0x100);
    memory_region_init_io(&s->gic_iomem[0], &gic_0_ops, s, "gic_cpu0", 0x100);
    memory_region_init_io(&s->gic_iomem[1], &gic_1_ops, s, "gic_cpu1", 0x100);
    memory_region_init_io(&s->gic_iomem[2], &gic_2_ops, s, "gic_cpu2", 0x100);
    memory_region_init_io(&s->gic_iomem[3], &gic_3_ops, s, "gic_cpu3", 0x100);
    memory_region_init_io(&s->dist_iomem, &dist_ops, s, "dist", 0x1000);
    memory_region_add_subregion(&s->iomem, 0x0000, &s->scu_iomem);
    memory_region_add_subregion(&s->iomem, 0x0100, &s->gic_iomem[4]);
    for(n = 0; n < 4; ++n)
    {
        memory_region_add_subregion(&s->iomem, 0x0100 * (n + 2), &s->gic_iomem[n]);
    }
    memory_region_add_subregion(&s->iomem, 0x1000, &s->dist_iomem);

    s->mptimer = qdev_create(NULL, "arm_mptimer");
    qdev_prop_set_uint32(s->mptimer, "num-cpu", s->num_cpu);
    qdev_init_nofail(s->mptimer);

    busdev = sysbus_from_qdev(s->mptimer);
    /* Add the regions for timer and watchdog for "current CPU" and
     * for each specific CPU.
     */
    s->timer_irq = qemu_allocate_irqs(periph_mptimer_set_irq,
                                      s, (s->num_cpu) * 2);
    for (n = 0; n < (s->num_cpu + 1) * 2; n++) {
        /* Timers at 0x600, 0x700, ...; watchdogs at 0x620, 0x720, ... */
        target_phys_addr_t offset = 0x600 + (n >> 1) * 0x100 + (n & 1) * 0x20;
        memory_region_add_subregion(&s->iomem, offset,
                                    sysbus_mmio_get_region(busdev, n));
    }
    for (n = 0; n < s->num_cpu * 2; n++) {
        sysbus_connect_irq(busdev, n, s->timer_irq[n]);
    }

    sysbus_init_mmio(dev, &s->iomem);
    /* Routed irqs at 0 - n + IRQ / FIQ */
    qdev_init_gpio_in(&dev->qdev, periph_set_irq, s->num_irq - 32 + s->num_cpu * 2);

    for(n = 0; n < s->num_cpu; ++n)
    {
        sysbus_init_irq(dev, &s->irq_out[n]);
        s->irq_state[n] = 2;
        s->fiq_state[n] = 2;
        s->cpu[n].intid = 0x3FF;
        s->cpu[n].apr = 0xFF;
    }
    for(n = 0; n < s->num_cpu; ++n)
    {
        sysbus_init_irq(dev, &s->fiq_out[n]);
    }

    s->scu_control = 0xFFE;

    for(n = 0; n < s->num_cpu; ++n)
    {
        s->cpu[n].icr = 0;
        s->cpu[n].pmr = 0;
        s->cpu[n].bpr = 0;
        s->dist.ier_int[n] = 0;
        /* IPTR of interrupts 0-31 are fixed */
        for(j = 0; j < 32; ++j)
        {
            s->dist.iptr_int[n][j] = (1u << n);
        }
    }
    for(n = 0; n < MAX_DIST_EXT_INT_STATE_VARS; ++n)
    {
        s->dist.ier_ext[n] = 0;
        s->dist.ipr_ext[n] = 0;
        s->dist.icfr_ext[n] = 0;
    }

    /* make iptr default to all valid cpu interfaces set */
    for(n = 32; n < s->num_irq; ++n)
    {
        s->dist.iptr_ext[n - 32] = (1 << s->num_cpu) - 1;
    }


    return 0;
}

static Property periph_properties[] = {
    DEFINE_PROP_UINT32("num-cpu", periph_state, num_cpu, 1),
    DEFINE_PROP_UINT32("num-irq", periph_state, num_irq, 64),
    DEFINE_PROP_END_OF_LIST(),
};

static void periph_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->no_user = 1;
    sdc->init = periph_init;
    dc->reset = periph_reset;
    dc->props = periph_properties;
}

static TypeInfo periph_info = {
    .name          = "mpcore-periph",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(periph_state),
    .class_init    = periph_class_init,
};

static void periph_register_devices(void)
{
    type_register_static(&periph_info);
}

device_init(periph_register_devices)
