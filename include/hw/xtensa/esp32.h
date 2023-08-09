#pragma once

#include "qemu/osdep.h"
#include "hw/hw.h"
#include "target/xtensa/cpu.h"
#include "hw/xtensa/esp32_reg.h"
#include "hw/xtensa/esp32_uart.h"
#include "hw/xtensa/esp32_gpio.h"
#include "hw/xtensa/esp32_dport.h"
#include "hw/xtensa/esp32_rtc_cntl.h"
#include "hw/xtensa/esp32_rng.h"
#include "hw/xtensa/esp32_sha.h"
#include "hw/xtensa/esp32_aes.h"
#include "hw/xtensa/esp32_ledc.h"
#include "hw/xtensa/esp32_rsa.h"
#include "hw/xtensa/esp32_unknown.h"
#include "hw/xtensa/esp32_ana.h"
#include "hw/xtensa/esp32_wifi.h"
#include "hw/xtensa/esp32_phya.h"
#include "hw/xtensa/esp32_fe.h"
#include "hw/xtensa/esp32_frc_timer.h"
#include "hw/xtensa/esp32_timg.h"
#include "hw/xtensa/esp32_crosscore_int.h"
#include "hw/xtensa/esp32_spi.h"
#include "hw/xtensa/esp32_i2c.h"
#include "hw/xtensa/esp32_efuse.h"
#include "hw/xtensa/esp32_intc.h"
#include "hw/xtensa/esp32_flash_enc.h"
#include "hw/xtensa/dwc_sdmmc.h"

typedef struct Esp32SocState {
    /*< private >*/
    DeviceState parent_obj;

    /*< public >*/
    XtensaCPU cpu[ESP32_CPU_COUNT];
    Esp32DportState dport;
    Esp32IntMatrixState intmatrix;
    Esp32CrosscoreInt crosscore_int;
    ESP32UARTState uart[ESP32_UART_COUNT];
    Esp32GpioState gpio;
    Esp32RngState rng;
    Esp32RtcCntlState rtc_cntl;
    Esp32FrcTimerState frc_timer[ESP32_FRC_COUNT];
    Esp32TimgState timg[ESP32_TIMG_COUNT];
    Esp32SpiState spi[ESP32_SPI_COUNT];
    Esp32I2CState i2c[ESP32_I2C_COUNT];
    Esp32ShaState sha;
    Esp32AesState aes;
    Esp32RsaState rsa;
    Esp32LEDCState ledc;
    Esp32EfuseState efuse;
    Esp32FlashEncryptionState flash_enc;
    Esp32UnknownState unknown;
    Esp32AnaState ana;
    Esp32WifiState wifi;
    Esp32PhyaState phya;
    Esp32FeState fe;

    DWCSDMMCState sdmmc;
    DeviceState *eth;
    DeviceState *wifi_dev;


    BusState rtc_bus;
    BusState periph_bus;

    MemoryRegion cpu_specific_mem[ESP32_CPU_COUNT];

    uint32_t requested_reset;
} Esp32SocState;
