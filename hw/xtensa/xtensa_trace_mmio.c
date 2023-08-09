#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/guest-random.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "sysemu/reset.h"
#include "sysemu/runstate.h"
#include "exec/address-spaces.h"
#include "hw/core/cpu.h"
#include "target/xtensa/cpu.h"

// #define DEBUG_UNWINDER

FILE* functiontrace_fd = NULL;

#define MAKE_PC_FROM_RA(ra,sp)    (((ra) & 0x3fffffff) | ((sp) & 0xc0000000))

void log_mmio_access(uint32_t address, uint32_t value, bool access_is_write);


// stack unwinding code adapted from the Linux kernel
//  see https://elixir.bootlin.com/linux/latest/source/arch/xtensa/kernel/stacktrace.c
// NOTE: this only works correctly **inside** functions
//  it specifically does not work when halted on an ENTRY instruction:
//  the unwinder then leaves out one function from the callstack
void log_mmio_access(uint32_t address, uint32_t value, bool access_is_write) {

    CPUState *cs = first_cpu;
    // Only log when we're in single-step mode, otherwise the CPU state is not
    // up-to-date at all times (likely only at the beginning of a TGC basic block)
    if (!singlestep) {
        return;
    }
    // Logs the address, value read or written and stacktrace to a file
    if (!functiontrace_fd) {
        functiontrace_fd = fopen("/tmp/qemu_mmio_log.txt", "wb");
        if (!functiontrace_fd) {
            exit(23);
        }
        fprintf(functiontrace_fd, "mode address value stacktrace[...]");
    }
    fprintf(functiontrace_fd, "\n%c %08x %08x", "RW"[access_is_write], address, value);

    

    XtensaCPU *cpu = XTENSA_CPU(cs);
    int depth = 255;

    uint32_t windowstart =  cpu->env.sregs[WINDOW_START];
    uint32_t windowbase = cpu->env.sregs[WINDOW_BASE];
    uint32_t pc = cpu->env.pc;
    uint32_t a0 = cpu->env.regs[0];
    uint32_t a1 = cpu->env.regs[1];
    #ifdef DEBUG_UNWINDER
    printf("stacktrace[pc]: %08x\n", pc);
    #endif
    fprintf(functiontrace_fd, " %08x", pc);
    uint32_t wsbits = cpu->env.config->nareg / 4;

    for (int index = 0; (index < wsbits) && depth; index++) {
        if (windowstart & (1 << ((windowbase - index) & (wsbits - 1)))) {
            depth--;
            /* Read a0 from the
             * corresponding position in AREGs.
             */
            a0 = cpu->env.phys_regs[((windowbase - index) & (wsbits - 1)) * 4];
            a1 = cpu->env.phys_regs[((windowbase - index) & (wsbits - 1)) * 4 + 1];
            /* Get the PC from a0 */
            if (a0 == 0)
                return;
            pc = MAKE_PC_FROM_RA(a0, pc);
            #ifdef DEBUG_UNWINDER
            printf("stacktrace[window]: %08x\n", pc);
            #endif
            fprintf(functiontrace_fd, " %08x", pc);
        }
    }
    /* Step 2. */
    /* We are done with the register window, we need to
     * look through the stack.
     */
    if (!depth)
        return;

    /* Start from the a1 register. */
    /* a1 = regs->areg[1]; */
    while (a0 != 0 && depth--) {
        /* Check if the region is OK to access. */
        if ((a1 & 0xfff00000) != 0x3ff00000)
            return;

        // copy a0, a1 from the stack
        address_space_read(&address_space_memory, a1 + 4 * (-4 + 0), MEMTXATTRS_UNSPECIFIED, &a0, 4);
        address_space_read(&address_space_memory, a1 + 4 * (-4 + 1), MEMTXATTRS_UNSPECIFIED, &a1, 4);    
        if (a0 == 0)
            return;
        pc = MAKE_PC_FROM_RA(a0, pc);
        #ifdef DEBUG_UNWINDER
        printf("stacktrace[stack]: %08x\n", pc);
        #endif
        fprintf(functiontrace_fd, " %08x", pc);
    }
}
