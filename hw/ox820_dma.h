#ifndef _ox820_dma_h
#define _ox820_dma_h

#include "sysbus.h"

DeviceState*
ox820_dma_initialize(unsigned int num_channel,
                     uint32_t cken, uint32_t rsten, uint16_t start_stop,
                     void (* readblock_busb)(void* opaque, target_phys_addr_t, void* buf, int len),
                     void (* writeblock_busb)(void* opaque, target_phys_addr_t, const void* buf, int len),
                     void* opaque);

#endif
