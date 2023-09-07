#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/guest-random.h"
#include "qapi/error.h"
#include "sysemu/sysemu.h"
#include "hw/hw.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/xtensa/esp32_wifi.h"
#include "sysemu/reset.h"
#include "sysemu/runstate.h"
#include "exec/address-spaces.h"
#include "esp32_wlan_packet.h"
#include "hw/qdev-properties.h"

#include "xtensa_trace_mmio.h"
#include "hw/xtensa/esp32_reg.h"

static uint64_t esp32_wifi_read(void *opaque, hwaddr addr, unsigned int size)
{

    Esp32WifiState *s = ESP32_WIFI(opaque);
    uint32_t r = s->mem[addr/4];

    switch(addr) {
        case A_WIFI_TXRX_INIT_10C:
        case A_WIFI_TXRX_INIT_114:
        case A_WIFI_TXRX_INIT_C1C:
        case A_WIFI_TXRX_INIT_C20:
        case A_WIFI_TXRX_INIT_C24:
        case A_WIFI_TXRX_INIT_C54:
        case A_WIFI_TXRX_INIT_C5C:
        case A_WIFI_TXRX_INIT_C6C:
        case A_WIFI_TXRX_INIT_C74:
        case A_WIFI_TXRX_INIT_C78:
        case A_WIFI_TXRX_INIT_C88:
        case A_WIFI_TXRX_INIT_CAC:
        case A_WIFI_TXRX_INIT_D78:
        case A_WIFI_TXRX_INIT_288:
            qemu_log_mask(LOG_UNIMP, "wifi TXRX INIT read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_BSSID_ADDR_FST_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_ADDR_FST_0 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_BSSID_ADDR_SND_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_ADDR_SND_0 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_BSSID_ADDR_FST_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_ADDR_FST_1 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_BSSID_ADDR_SND_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_ADDR_SND_1 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_BSSID_FILTER_FST_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_FILTER_FST_0 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_BSSID_FILTER_SND_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_FILTER_SND_0 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_BSSID_FILTER_FST_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_FILTER_FST_1 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_BSSID_FILTER_SND_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_FILTER_SND_1 read %08x\n", (uint32_t) addr);
            break;

        case A_WIFI_MAC_ADDR_FST_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_ADDR_FST_0 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_MAC_ADDR_SND_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_ADDR_SND_0 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_MAC_ADDR_FST_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_ADDR_FST_1 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_MAC_ADDR_SND_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_ADDR_SND_1 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_MAC_FILTER_FST_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_FILTER_FST_0 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_MAC_FILTER_SND_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_FILTER_SND_0 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_MAC_FILTER_FST_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_FILTER_FST_1 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_MAC_FILTER_SND_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_FILTER_SND_1 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_RXBUF_INIT_BITMASK:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_RXBUF_INIT_BITMASK read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_DMA_IN_STATUS:
            r=0;
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_DMA_IN_STATUS read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_DMA_INLINK:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_DMA_INLINK read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_NEXT_RX_DSCR:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_NEXT_RX_DSCR read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_LAST_RX_DSCR:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_LAST_RX_DSCR read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_RX_POLICY_0:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_RX_POLICY_0 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_RX_POLICY_1:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_RX_POLICY_1 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_RX_POLICY_2:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_RX_POLICY_2 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_RX_POLICY_3:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_RX_POLICY_3 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_PROMISC_MISC_BITMASK0:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_PROMISC_MISC_BITMASK0 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_PROMISC_MISC_BITMASK1:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_PROMISC_MISC_BITMASK1 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_PROMISC_MISC_BITMASK2:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_PROMISC_MISC_BITMASK2 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_PROMISC_MISC_BITMASK3:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_PROMISC_MISC_BITMASK3 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_RXBUF_INIT_HIGH_ADDR_0:
        case A_WIFI_RXBUF_INIT_LOW_ADDR_0:
        case A_WIFI_RXBUF_INIT_HIGH_ADDR_1:
        case A_WIFI_RXBUF_INIT_LOW_ADDR_1:
            qemu_log_mask(LOG_UNIMP, "wifi RXBUF_INIT high and low addresses read (unexpected!) %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_LAST_RXBUF_INIT_09C:
        case A_WIFI_LAST_RXBUF_INIT_148:
        case A_WIFI_LAST_RXBUF_INIT_14C:
        case A_WIFI_LAST_RXBUF_INIT_158:
        case A_WIFI_LAST_RXBUF_INIT_164:
            qemu_log_mask(LOG_UNIMP, "wifi LAST_RXBUF_INIT registers read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_ANTENNA_INIT_284:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_ANTENNA_INIT_284 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_AUTOACK_INIT_400:
        case A_WIFI_AUTOACK_INIT_404:
        case A_WIFI_AUTOACK_INIT_408:
        case A_WIFI_AUTOACK_INIT_40C:
        case A_WIFI_AUTOACK_INIT_410:
        case A_WIFI_AUTOACK_INIT_414:
            qemu_log_mask(LOG_UNIMP, "wifi AUTOACK_INIT registers read (unexpected!) %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_LOW_RATE_418:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_LOW_RATE_418 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_LOW_RATE_41C:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_LOW_RATE_41C read %08x\n", (uint32_t) addr);
            break;
        case 0x800 ... 0x814:
            qemu_log_mask(LOG_UNIMP, "esp32_wifi_read crypto %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_MAYBE_TIMESTAMP:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_MAYBE_TIMESTAMP read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_PROMISC_CONTROL_PKT:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_PROMISC_CONTROL_PKT read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_DMA_INT_STATUS:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_DMA_INT_STATUS read %08x\n", (uint32_t) addr);
            r=s->raw_interrupt;
            break;
        case A_WIFI_DMA_INT_CLR:
            r=s->raw_interrupt;
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_DMA_INT_CLR read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_MAYBE_PWR_CTL:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_MAYBE_PWR_CTL read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_TXQ_CLR_STATE_COLL_TIMEOUT:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_TXQ_CLR_STATE_COLL_TIMEOUT read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_TXQ_STATE_COLL_TIMEOUT:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_TXQ_STATE_COLL_TIMEOUT read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_TXQ_CLR_STATE_COMPLETE:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_TXQ_CLR_STATE_COMPLETE read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_TXQ_STATE_COMPLETE:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_TXQ_STATE_COMPLETE read %08x\n", (uint32_t) addr);
            r=1;
            break;
        case A_WIFI_TX_CONFIG_0:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_TX_CONFIG_0 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_DMA_OUTLINK:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_DMA_OUTLINK read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_DMA_OUT_STATUS:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_DMA_OUT_STATUS read %08x\n", (uint32_t) addr);
            r=1;
            break;
        case 0xd30:
        case 0xd38:
        case 0xd40:
            // vm_stop(RUN_STATE_DEBUG)
            qemu_log_mask(LOG_UNIMP, "wifi phy_enable() init registers read %08x\n", (uint32_t) addr);
            break;
        default:
            qemu_log_mask(LOG_UNIMP, "wifi: unimplemented device read %08x\n", (uint32_t) addr + DR_REG_WIFI_BASE);
            break;
    }

    log_mmio_access(addr + DR_REG_WIFI_BASE, r, false);
    // Stop VM to debug in GDB
    // vm_stop(RUN_STATE_DEBUG);
    return r;
}
static void set_interrupt(Esp32WifiState *s, int e) {
    s->raw_interrupt |= e;
    qemu_set_irq(s->irq, 1);
}

static void esp32_wifi_write(void *opaque, hwaddr addr, uint64_t v, unsigned int size) {
    Esp32WifiState *s = ESP32_WIFI(opaque);
    uint32_t value = (uint32_t) v;
    log_mmio_access(addr + DR_REG_WIFI_BASE, value, true);
    switch (addr) {
        case A_WIFI_TXRX_INIT_10C:
        case A_WIFI_TXRX_INIT_114:
        case A_WIFI_TXRX_INIT_C1C:
        case A_WIFI_TXRX_INIT_C20:
        case A_WIFI_TXRX_INIT_C24:
        case A_WIFI_TXRX_INIT_C54:
        case A_WIFI_TXRX_INIT_C5C:
        case A_WIFI_TXRX_INIT_C6C:
        case A_WIFI_TXRX_INIT_C74:
        case A_WIFI_TXRX_INIT_C78:
        case A_WIFI_TXRX_INIT_C88:
        case A_WIFI_TXRX_INIT_CAC:
        case A_WIFI_TXRX_INIT_D78:

        case A_WIFI_TXRX_INIT_288: // also called in hal_deinit
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_TXRX_INIT write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_BSSID_ADDR_FST_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_ADDR_FST_0 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_BSSID_ADDR_SND_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_ADDR_SND_0 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_BSSID_ADDR_FST_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_ADDR_FST_1 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_BSSID_ADDR_SND_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_ADDR_SND_1 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_BSSID_FILTER_FST_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_FILTER_FST_0 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_BSSID_FILTER_SND_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_FILTER_SND_0 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_BSSID_FILTER_FST_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_FILTER_FST_1 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_BSSID_FILTER_SND_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_FILTER_SND_1 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_MAC_ADDR_FST_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_ADDR_FST_0 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_MAC_ADDR_SND_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_ADDR_SND_0 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_MAC_ADDR_FST_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_ADDR_FST_1 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_MAC_ADDR_SND_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_ADDR_SND_1 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_MAC_FILTER_FST_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_FILTER_FST_0 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_MAC_FILTER_SND_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_FILTER_SND_0 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_MAC_FILTER_FST_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_FILTER_FST_1 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_MAC_FILTER_SND_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_FILTER_SND_1 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_RXBUF_INIT_BITMASK:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_RXBUF_INIT_BITMASK write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_DMA_IN_STATUS:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_DMA_IN_STATUS write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_DMA_INLINK:
            s->dma_inlink_address = value;
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_DMA_INLINK write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_NEXT_RX_DSCR:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_NEXT_RX_DSCR write (unexpected!) %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_LAST_RX_DSCR:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_LAST_RX_DSCR write (unexpected!) %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_RX_POLICY_0:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_RX_POLICY_0 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_RX_POLICY_1:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_RX_POLICY_1 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_RX_POLICY_2:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_RX_POLICY_2 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_RX_POLICY_3:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_RX_POLICY_3 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_PROMISC_MISC_BITMASK0:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_PROMISC_MISC_BITMASK0 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_PROMISC_MISC_BITMASK1:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_PROMISC_MISC_BITMASK1 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_PROMISC_MISC_BITMASK2:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_PROMISC_MISC_BITMASK2 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_PROMISC_MISC_BITMASK3:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_PROMISC_MISC_BITMASK3 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_RXBUF_INIT_HIGH_ADDR_0:
        case A_WIFI_RXBUF_INIT_LOW_ADDR_0:
        case A_WIFI_RXBUF_INIT_HIGH_ADDR_1:
        case A_WIFI_RXBUF_INIT_LOW_ADDR_1:
            qemu_log_mask(LOG_UNIMP, "wifi RXBUF_INIT high and low addresses %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_LAST_RXBUF_INIT_09C:
        case A_WIFI_LAST_RXBUF_INIT_148:
        case A_WIFI_LAST_RXBUF_INIT_14C:
        case A_WIFI_LAST_RXBUF_INIT_158:
        case A_WIFI_LAST_RXBUF_INIT_164:
            qemu_log_mask(LOG_UNIMP, "wifi LAST_RXBUF_INIT registers write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_ANTENNA_INIT_284:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_ANTENNA_INIT_284 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_AUTOACK_INIT_400:
        case A_WIFI_AUTOACK_INIT_404:
        case A_WIFI_AUTOACK_INIT_408:
        case A_WIFI_AUTOACK_INIT_40C:
        case A_WIFI_AUTOACK_INIT_410:
        case A_WIFI_AUTOACK_INIT_414:
            qemu_log_mask(LOG_UNIMP, "wifi AUTOACK_INIT registers write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_LOW_RATE_418:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_LOW_RATE_418 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_LOW_RATE_41C:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_LOW_RATE_41C write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case 0x800 ... 0x814:
            qemu_log_mask(LOG_UNIMP, "esp32_wifi_write crypto %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_MAYBE_TIMESTAMP:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_MAYBE_TIMESTAMP write (unexpected!) %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_PROMISC_CONTROL_PKT:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_PROMISC_CONTROL_PKT write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_DMA_INT_STATUS:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_DMA_INT_STATUS write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_DMA_INT_CLR:
            s->raw_interrupt &= ~value;
            if (s->raw_interrupt == 0)
                qemu_set_irq(s->irq, 0);
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_DMA_INT_CLR write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_MAYBE_PWR_CTL:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_MAYBE_PWR_CTL write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_TXQ_CLR_STATE_COLL_TIMEOUT:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_TXQ_CLR_STATE_COLL_TIMEOUT write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_TXQ_STATE_COLL_TIMEOUT:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_TXQ_STATE_COLL_TIMEOUT write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_TXQ_CLR_STATE_COMPLETE:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_TXQ_CLR_STATE_COMPLETE write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_TXQ_STATE_COMPLETE:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_TXQ_STATE_COMPLETE write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_TX_CONFIG_0:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_TX_CONFIG_0 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_DMA_OUTLINK:
            if (value & 0xc0000000) {
                // do a DMA transfer to the hardware from esp32 memory
                mac80211_frame frame;
                dma_list_item item;
                unsigned memaddr = (0x3ff00000 | (value & 0xfffff));
                address_space_read(&address_space_memory, memaddr, MEMTXATTRS_UNSPECIFIED, &item, 12);
                qemu_log_mask(LOG_UNIMP, "wifi TX: size=%d length=%d unknown=%d eof=%d owner=%d address=%08x next=%08x\n", item.size, item.length, item.unknown, item.eof, item.owner, item.address, item.next);
  
                address_space_read(&address_space_memory, item.address, MEMTXATTRS_UNSPECIFIED, &frame, item.length);
                // frame from esp32 to ap
                frame.frame_length=item.length;
                frame.next_frame=0;
                Esp32_WLAN_handle_frame(s, &frame);
                set_interrupt(s, 0x80);
            }
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_DMA_OUTLINK write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_DMA_OUT_STATUS:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_DMA_OUT_STATUS write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case 0xd30:
        case 0xd38:
        case 0xd40:
            qemu_log_mask(LOG_UNIMP, "wifi phy_enable() init registers write %08x=%08x\n", (uint32_t) addr, value);
            break;
        default:
            qemu_log_mask(LOG_UNIMP, "wifi: unimplemented device write %08x = %08x\n", (uint32_t) addr + DR_REG_WIFI_BASE, value);
            break;
    }
    s->mem[addr/4]=value;
}

static int match_mac_address(uint8_t *a1,uint8_t *a2) {
    if(!memcmp(a1,a2,6)) return 1;
    if(!memcmp(a1,BROADCAST,6)) return 1;
    return 0;
}

// frame from QEMU to ESP32
void Esp32_sendFrame(Esp32WifiState *s, mac80211_frame *frame, int length, int signal_strength) {

    if(s->dma_inlink_address == 0) {
        return;
    }
    uint8_t header[28+length];
    wifi_pkt_rx_ctrl_t *pkt=(wifi_pkt_rx_ctrl_t *)header;
    *pkt=(wifi_pkt_rx_ctrl_t){
        .rssi=(signal_strength+(rand()%10)+96),
        .rate=11,
        .sig_len=length,
        .sig_len_copy=length,
        .legacy_length=length,
        .noise_floor=-97,
        .channel=esp32_wifi_channel,
        .timestamp=qemu_clock_get_ns(QEMU_CLOCK_REALTIME)/1000,
    };
    // These 4 bits are set if the mac addresses previously stored at 0x40 and 0x48
    // match the destination or bssid addresses in the frame
    if(match_mac_address(frame->receiver_address,(uint8_t *)s->mem+0x40))
        pkt->damatch0=1;
    if(match_mac_address(frame->receiver_address,(uint8_t *)s->mem+0x48))
        pkt->damatch1=1;
    if(match_mac_address(frame->address_3,(uint8_t *)s->mem+0x40))
        pkt->bssidmatch0=1;
    if(match_mac_address(frame->address_3,(uint8_t *)s->mem+0x48))
        pkt->bssidmatch1=1;
    //printf("...%x %x\n",header[3],frame->receiver_address[0]);

    memcpy(header+28, frame, length);
    length += 28;
    // do a DMA transfer from the hardware to esp32 memory
    dma_list_item item;
    address_space_read(&address_space_memory, s->dma_inlink_address, MEMTXATTRS_UNSPECIFIED, &item, 12);
    address_space_write(&address_space_memory, item.address, MEMTXATTRS_UNSPECIFIED, header, length);
    item.length=length;
    item.eof=1;
    address_space_write(&address_space_memory, s->dma_inlink_address, MEMTXATTRS_UNSPECIFIED,&item,4);
    s->dma_inlink_address=item.next;
    set_interrupt(s, 0x1000024);
}

static const MemoryRegionOps esp32_wifi_ops = {
    .read =  esp32_wifi_read,
    .write = esp32_wifi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_wifi_realize(DeviceState *dev, Error **errp)
{
    Esp32WifiState *s = ESP32_WIFI(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    s->dma_inlink_address = 0;

    memory_region_init_io(&s->iomem, OBJECT(dev), &esp32_wifi_ops, s,
                          TYPE_ESP32_WIFI, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    memset(s->mem,0,sizeof(s->mem));
    Esp32_WLAN_setup_ap(dev, s);

}
static Property esp32_wifi_properties[] = {
    DEFINE_NIC_PROPERTIES(Esp32WifiState, conf),
    DEFINE_PROP_END_OF_LIST(),
};
static void esp32_wifi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = esp32_wifi_realize;
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
    dc->desc = "Esp32 WiFi";
    device_class_set_props(dc, esp32_wifi_properties);
}


static const TypeInfo esp32_wifi_info = {
    .name = TYPE_ESP32_WIFI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32WifiState),
    .class_init    = esp32_wifi_class_init,
};

static void esp32_wifi_register_types(void)
{
    type_register_static(&esp32_wifi_info);
}

type_init(esp32_wifi_register_types)
