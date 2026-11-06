#!/usr/bin/env python3
"""Exercise real GPIO code and extracted motor-stop callbacks with fake IRQs.

Usage: python3 tests/test_irq_shutdown.py [path/to/AM32]
This checks software ordering and nesting, not hardware interrupt latency.
"""
from pathlib import Path
import re
import subprocess
import sys
import tempfile

root = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path(__file__).resolve().parents[1]
main = (root / 'Src/main.c').read_text()


def block(marker):
    # Ignore braces in comments; this block has no alternative preprocessor arms.
    lex = re.sub(r'/\*.*?\*/|//[^\n]*', lambda m: ' ' * len(m.group()), main, flags=re.S)
    start = main.index(marker)
    depth = 0
    for i in range(lex.index('{', start), len(lex)):
        if lex[i] == '{':
            depth += 1
        elif lex[i] == '}':
            depth -= 1
            if depth == 0:
                return main[start:i + 1]
    raise ValueError(marker)


callback = block('void PeriodElapsedCallback()')
crossing = main[main.index('void interruptRoutine()'):main.index('void startMotor()')]
stop = block('if ((bemf_timeout_happened > bemf_timeout) && eepromBuffer.stuck_rotor_protection)')
header = r'''
#include <stdint.h>
typedef struct { unsigned unused; } gpio_type;
typedef struct { uint32_t gpio_mode, gpio_pins, gpio_pull; } gpio_init_type;
typedef struct { uint16_t cval; } timer_type;
extern timer_type utility_timer;
#define UTILITY_TIMER (&utility_timer)
#define GPIO_MODE_MUX 2
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);
void gpio_default_para_init(gpio_init_type *p);
void gpio_init(gpio_type *p, gpio_init_type *i);
'''
prefix = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include "functions.h"
timer_type utility_timer;
static unsigned primask, comp_enabled, com_enabled, bridge_on, inject;
static unsigned commutation_interval=400, zero_crosses=100, advance;
static unsigned timer_count=400, com_delay;
static unsigned lastzctime=400, thiszctime=400, waitTime=160;
static unsigned input, bemf_timeout_happened=101, bemf_timeout=100;
static unsigned temp_advance=6, auto_advance_level=6;
static char old_routine, rising=1, filter_level=2;
static struct { unsigned auto_advance, stuck_rotor_protection; } eepromBuffer={0,1};
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask=1; }
void __enable_irq(void) { primask=0; }
void gpio_default_para_init(gpio_init_type *p) { *p=(gpio_init_type){0}; }
void gpio_init(gpio_type *p, gpio_init_type *i) {
    (void)p; (void)i;
    assert(primask); /* read/modify/write remains protected */
}
#define DISABLE_COM_TIMER_INT() (com_enabled=0)
#define COM_TIMER_CLEAR_PENDING() ((void)0)
#define INTERVAL_TIMER_COUNT timer_count
#define SET_INTERVAL_TIMER_COUNT(v) (timer_count=(v))
#define SET_AND_ENABLE_COM_INT(v) (com_delay=(v),com_enabled=1)
static bool demag_comp_timer(void) { return false; } /* stock path */
static bool demag_comp_scan(void) { return false; }
static unsigned demag_comp_advance(unsigned level) { return level; }
static char demag_comp_expect(char r) { return r; }
static bool demag_comp_clamp_end(void) { return false; }
static void demag_comp_disarm(void) {}
static void demag_comp_zc(void) {}
static char getCompOutputLevel(void) { return 0; }
static void enableCompInterrupts(void) { comp_enabled=1; }
static void commutate(void) { bridge_on=1; }
static void allOff(void) {
    gpio_type gpio={0};
    for (unsigned i=0; i<6; i++) gpio_mode_QUICK(&gpio,1,0,1U<<i);
    bridge_on=0;
}
void PeriodElapsedCallback(void);
static void maskPhaseInterrupts(void) {
    comp_enabled=0;
    /* A higher-priority COM interrupt immediately after comparator masking. */
    if (inject && !primask && com_enabled) {
        inject=0;
        PeriodElapsedCallback();
    }
}
'''
suffix = r'''
int main(void) {
    gpio_type gpio={0};
    for (unsigned masked=0; masked<2; masked++) {
        primask=masked;
        gpio_mode_QUICK(&gpio,1,0,1);
        assert(primask==masked);
    }
    puts("PASS F415 GPIO mode changes preserve the incoming IRQ mask");
    for (unsigned masked=0; masked<2; masked++) {
        for (unsigned inject_callback=0; inject_callback<2; inject_callback++) {
            input=1000; bridge_on=comp_enabled=com_enabled=1;
            bemf_timeout_happened=101; inject=inject_callback; primask=masked;
            stop();
            assert(primask==masked);
            assert(!comp_enabled && !com_enabled && !bridge_on && !input);
            if (comp_enabled) interruptRoutine();
            if (com_enabled) PeriodElapsedCallback();
            assert(!bridge_on);
        }
    }
    puts("PASS stuck-rotor stop excludes COM interleaving and preserves an outer mask");
    return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='irq-shutdown-test-') as tmp:
    d = Path(tmp)
    # Keep the real inline helpers, resolving their platform includes to mocks.
    (d / 'functions.h').write_bytes((root / 'Inc/functions.h').read_bytes())
    (d / 'main.h').write_text(header)
    (d / 'targets.h').write_text('#define ARTERY\n#define MCU_AT415\n')
    (d / 'test.c').write_text(prefix + callback + '\n' + crossing
                            + '\nstatic void stop(void) {\n' + stop + '\n}\n' + suffix)
    subprocess.run(['cc', '-std=c99', '-Wall', '-Wextra', '-Werror', '-O2',
                    '-I' + str(d), '-I' + str(root / 'Inc'),
                    str(root / 'Src/functions.c'), str(d / 'test.c'),
                    '-o', str(d / 'test')], check=True)
    subprocess.run([str(d / 'test')], check=True)
