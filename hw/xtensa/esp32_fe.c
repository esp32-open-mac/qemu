
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/guest-random.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/xtensa/esp32_fe.h"

#include "xtensa_trace_mmio.h"
#include "hw/xtensa/esp32_reg.h"

// RF Frontend

static uint64_t esp32_fe_read(void *opaque, hwaddr addr, unsigned int size)
{
    uint32_t r = 0;
    Esp32FeState *s = ESP32_FE(opaque);
    r = s->mem[addr/4];
    if (addr == 0x7c) {
        r = 0xffffffff;
    }
    qemu_log_mask(LOG_UNIMP, "fe: unimplemented device read %08x\n", (uint32_t) addr + DR_REG_FE_BASE);

    log_mmio_access(addr + DR_REG_FE_BASE, r, false);
    return r;
}

static void esp32_fe_write(void *opaque, hwaddr addr, uint64_t value, unsigned int size) {
  Esp32FeState *s = ESP32_FE(opaque);
  s->mem[addr/4] = (uint32_t)value;
  qemu_log_mask(LOG_UNIMP, "fe: unimplemented device write %08x = %08x\n", (uint32_t) addr+DR_REG_FE_BASE, (uint32_t) value);
  log_mmio_access(addr + DR_REG_FE_BASE, value, true);
}

static const MemoryRegionOps esp32_fe_ops = {
    .read =  esp32_fe_read,
    .write = esp32_fe_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_fe_init(Object *obj)
{
    Esp32FeState *s = ESP32_FE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_fe_ops, s, TYPE_ESP32_FE, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    memset(s->mem, 0, sizeof(s->mem));
}


static const TypeInfo esp32_fe_info = {
    .name = TYPE_ESP32_FE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32FeState),
    .instance_init = esp32_fe_init,
};

static void esp32_fe_register_types(void)
{
    type_register_static(&esp32_fe_info);
}

type_init(esp32_fe_register_types)
