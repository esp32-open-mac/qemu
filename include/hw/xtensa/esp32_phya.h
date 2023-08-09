#pragma once

#include "hw/hw.h"
#include "hw/registerfields.h"
#include "hw/sysbus.h"
#include "hw/xtensa/esp32_reg.h"


#define TYPE_ESP32_PHYA "misc.esp32.phya"
#define ESP32_PHYA(obj) OBJECT_CHECK(Esp32PhyaState, (obj), TYPE_ESP32_PHYA)

typedef struct Esp32PhyaState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t mem[1024];
} Esp32PhyaState;


REG32(PHYA_TX_PLCP1_0, 0x258);
REG32(PHYA_TX_ANTENNA_0, 0x25C);
REG32(PHYA_TX_HTSIG_FST_0, 0x260);
REG32(PHYA_TX_HTSIG_SND_0, 0x264);
REG32(PHYA_TX_DURATION_0, 0x268);
REG32(PHYA_TXQ_PMD, 0x270);
