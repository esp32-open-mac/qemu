xtensa_ss = ss.source_set()
xtensa_ss.add(files(
  'mx_pic.c',
  'pic_cpu.c',
  'xtensa_memory.c',
))
xtensa_ss.add(when: 'CONFIG_XTENSA_SIM', if_true: files('sim.c'))
xtensa_ss.add(when: 'CONFIG_XTENSA_VIRT', if_true: files('virt.c'))
xtensa_ss.add(when: 'CONFIG_XTENSA_XTFPGA', if_true: files('xtfpga.c'))

xtensa_ss.add(when: 'CONFIG_XTENSA_ESP32', if_true: files('xtensa_trace_mmio.c'))

xtensa_ss.add(when: 'CONFIG_XTENSA_ESP32', if_true: files(
  'esp32.c',
  'esp32_intc.c',
  'esp32_ana.c',
  'esp32_crosscore_int.c',
  'esp32_dport.c',
  'esp32_rng.c',
  'esp32_rtc_cntl.c',
  'esp32_sha.c',
  'esp32_aes.c',
  'esp32_ledc.c',
  'esp32_flash_enc.c',
  'esp32_unknown.c',
  'esp32_wifi.c',
  'esp32_wifi_ap.c',
  'esp32_wlan_packet.c',
  'esp32_phya.c',
  'esp32_fe.c',
  'ssi_psram.c'
))

xtensa_ss.add(when: 'CONFIG_XTENSA_ESP32', if_true: files('esp32_frc_timer.c', 'esp32_timg.c'))

xtensa_ss.add(when: 'CONFIG_XTENSA_ESP32', if_true: files('esp32_spi.c'))

xtensa_ss.add(when: 'CONFIG_DWC_SDMMC', if_true: files('dwc_sdmmc.c'))

xtensa_ss.add(when: 'CONFIG_XTENSA_ESP32', if_true: files('esp32_efuse.c'))

xtensa_ss.add(when: 'CONFIG_XTENSA_ESP32', if_true: files('esp32_i2c.c'))

xtensa_ss.add(when: 'CONFIG_XTENSA_ESP32', if_true: files('esp32_gpio.c'))

xtensa_ss.add(when: 'CONFIG_XTENSA_ESP32', if_true: files('esp32_uart.c'))


if gcrypt.found()
  xtensa_ss.add(when: [gcrypt, 'CONFIG_XTENSA_ESP32'], if_true: files(
    'esp32_rsa.c',
  ))
endif

hw_arch += {'xtensa': xtensa_ss}
