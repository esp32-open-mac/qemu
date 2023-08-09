#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/guest-random.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/xtensa/esp32_ana.h"
#include "sysemu/reset.h"
#include "sysemu/runstate.h"
#include "exec/address-spaces.h"
#include "hw/core/cpu.h"
#include "target/xtensa/cpu.h"

#include "xtensa_trace_mmio.h"
#include "hw/xtensa/esp32_reg.h"


int esp32_wifi_channel=0;


static uint64_t esp32_ana_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32AnaState *s = ESP32_ANA(opaque);
    qemu_log_mask(LOG_UNIMP, "analog: unimplemented device read %08x\n", (uint32_t) addr + DR_REG_ANA_BASE);
    
    uint32_t r = s->mem[addr/4];
    switch(addr) {
        case 4: r= 0xFDFFFFFF;
        break;
        case 68:
        case 76:
        case 0x0c4: r=0xFFFFFFFF;
        break;
    }
    log_mmio_access(addr + DR_REG_ANA_BASE, r, false);

    return r;
}

static void esp32_ana_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned int size) {
    Esp32AnaState *s = ESP32_ANA(opaque);
    log_mmio_access(addr + DR_REG_ANA_BASE, value, true);
    if  (addr == 0x0c4) {
        int v = value&255;
        if ((v % 10)==4) {
            esp32_wifi_channel=(v/10)-1;
            printf("esp32 wifi channel register = %d\n", esp32_wifi_channel);
            // vm_stop(RUN_STATE_DEBUG);
        }
    }
    s->mem[addr/4]=value;
    qemu_log_mask(LOG_UNIMP, "analog: unimplemented device write %08x = %08x\n", (uint32_t) addr + DR_REG_ANA_BASE, (uint32_t) value);
}

static const MemoryRegionOps esp32_ana_ops = {
    .read =  esp32_ana_read,
    .write = esp32_ana_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_ana_init(Object *obj)
{
    Esp32AnaState *s = ESP32_ANA(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_ana_ops, s,
                          TYPE_ESP32_ANA, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    memset(s->mem,0,sizeof(s->mem));
}


static const TypeInfo esp32_ana_info = {
    .name = TYPE_ESP32_ANA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32AnaState),
    .instance_init = esp32_ana_init,
};

static void esp32_ana_register_types(void)
{
    type_register_static(&esp32_ana_info);
}

type_init(esp32_ana_register_types)
