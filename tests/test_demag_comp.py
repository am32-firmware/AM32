#!/usr/bin/env python3
"""Compile the real demag controller with fake peripherals; no motor-model feedback.
Usage: python3 tests/test_demag_comp.py [path/to/AM32]

These are controller arithmetic/state tests. Hardware interrupt timing is
covered separately by Renode; the mock does not emulate NVIC or timer wrap.
"""
from pathlib import Path
import subprocess
import sys
import tempfile
root = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path(__file__).resolve().parents[1]
headers = {
    'targets.h': '#define DEMAG_COMP_ENABLED 1\n',
    'common.h': '#include <stdint.h>\n#define __disable_irq() ((void)0)\n#define __enable_irq() ((void)0)\n',
    'comparator.h': 'char getCompOutputLevel(void);\nvoid maskPhaseInterrupts(void);\nvoid enableCompInterrupts(void);\n',
    'phaseouts.h': 'void allOff(void);\n',
    'peripherals.h': '''extern unsigned timer_count, com_delay, com_enabled;
#define INTERVAL_TIMER_COUNT timer_count
#define SET_INTERVAL_TIMER_COUNT(v) (timer_count = (v))
#define SET_AND_ENABLE_COM_INT(v) (com_delay = (v), com_enabled = 1)
#define DISABLE_COM_TIMER_INT() (com_enabled = 0)
#define COM_TIMER_CLEAR_PENDING() ((void)0)
''',
}
source = r'''
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include "targets.h"
#include "demag_comp.h"
unsigned timer_count, com_delay, com_enabled, off, crossings;
#define cuts demag_comp_cuts
volatile char rising = 1;
char step = 1, old_routine, prop_brake_active, stepper_sine;
uint8_t running = 1, filter_level = 2;
uint16_t input = 1000;
volatile uint16_t duty_cycle = 1000, thiszctime, lastzctime, waitTime = 160;
volatile uint32_t commutation_interval = 400, average_interval = 400, zero_crosses = 100;
static char comp;
char getCompOutputLevel(void) { return comp; }
void maskPhaseInterrupts(void) {}
void enableCompInterrupts(void) {}
void setCompEdge(char r) { (void)r; }
void allOff(void) { off++; }
void interruptRoutine(void) { crossings++; }
static void reset(void) {
    input = duty_cycle = 1000; old_routine = 1; demag_comp_commutate(); old_routine = 0;
    demag_comp_metric = 120;
    cuts = off = crossings = 0; comp = 0;
    thiszctime = 390; lastzctime = 410;
    commutation_interval = average_interval = 400;
    waitTime = 160; timer_count = 160;
}
static void scan(void) {
    timer_count = 160; /* the commutation wait has elapsed */
    assert(demag_comp_scan());
    timer_count = 202; assert(demag_comp_timer());
}
static void blind(unsigned lateness) {
    timer_count = 400 + lateness; assert(demag_comp_timer());
    assert(thiszctime == 400 && lastzctime == 400);
    assert(timer_count == lateness);
    assert(com_delay == (lateness < 160 ? 161-lateness : 2));
    /* The actual main.c filter must leave the speed estimate unchanged. */
    commutation_interval = (commutation_interval + ((lastzctime + thiszctime) >> 1)) >> 1;
    assert(commutation_interval == 400);
}
int main(void) {
#ifdef TEST_CUT_DISABLED
    reset(); duty_cycle = 1400; thiszctime = lastzctime = 400;
    demag_comp_zc(); demag_comp_commutate();
    for (unsigned i = 0; i < 23; i++) {
        scan(); blind(0);
        timer_count = 160; assert(!demag_comp_timer()); demag_comp_commutate();
    }
    scan(); timer_count = 400; assert(demag_comp_timer());
    assert(!cuts && off == 1 && demag_comp_desync_pending() && !com_enabled);
    puts("PASS disabled relief retains persistent-clamp shutdown");
#else
    reset(); scan();
    assert(com_delay == 198); /* predicted crossing, not 7/8 of it */
    blind(7);
    assert(demag_comp_timer()); /* queued early timer cannot commutate */
    timer_count = 160; assert(!demag_comp_timer());
    demag_comp_commutate();
    puts("PASS blind estimate and late/early timer handling");

    reset(); scan(); blind(200);
    assert(!demag_comp_timer()); /* already beyond the commutation deadline */
    puts("PASS overdue commutation remains bounded");

    reset(); demag_comp_metric = 180; scan();
    assert(cuts == 1 && off == 1); /* relief starts at scan, before blind deadline */
    blind(0); assert(cuts == 1 && off == 1); /* same sector counted/cut only once */
    timer_count = 160; assert(!demag_comp_timer()); demag_comp_commutate();
    scan(); assert(cuts == 2 && off == 2); /* the next sector can cut again */
    puts("PASS early power relief and one cut per sector");

    reset();
    for (unsigned i = 0; i < 23; i++) {
        timer_count = 160; scan(); blind(0);
        timer_count = 160; assert(!demag_comp_timer()); demag_comp_commutate();
    }
    timer_count = 160; scan(); timer_count = 400; assert(demag_comp_timer());
    assert(off == cuts + 1 && demag_comp_desync_pending() && !com_enabled);
    assert(demag_comp_blind_limit == 1);
    puts("PASS persistent clamp still powers off at blind-run limit");

    reset(); comp = 1; scan(); /* pre-crossing level, listen for a real crossing */
    timer_count = 202 + 6400; assert(demag_comp_timer());
    assert(off == cuts + 1 && demag_comp_desync_pending() && !com_enabled);
    assert(demag_comp_timeouts == 1);
    puts("PASS missing crossing still powers off at extended timeout");
    reset(); input = 0;
    assert(!demag_comp_scan()); /* commanded stop must use normal tracking */
    reset(); scan(); input = 0;
    timer_count = 400; assert(demag_comp_timer());
    assert(!demag_comp_state && !demag_comp_invert && !com_enabled);
    assert(!demag_comp_desync_pending() && !off);
    puts("PASS stop cancels pending demag without requesting a desync");

    reset(); duty_cycle = 1100; scan();
    assert(cuts == 1 && demag_comp_metric == 120);
    puts("PASS rising duty relieves the first clamp without boosting the metric");

    reset(); duty_cycle = 1400; thiszctime = lastzctime = 400;
    demag_comp_zc(); /* the last duty increase, then 24 measured crossings */
    for (unsigned i = 0; i < 24; i++) {
        demag_comp_commutate(); scan();
        assert(cuts == i + 1);
        thiszctime = lastzctime = 400; demag_comp_zc();
    }
    demag_comp_commutate(); scan();
    assert(cuts == 24 && demag_comp_metric == 120);
    puts("PASS relief survives the duty plateau and expires after recovered feedback");

    reset(); thiszctime = 380; lastzctime = 400; demag_comp_zc();
    demag_comp_commutate(); scan(); assert(cuts == 1);
    for (unsigned i = 0; i < 23; i++) {
        blind(0);
        timer_count = 160; assert(!demag_comp_timer()); demag_comp_commutate();
        scan();
    }
    timer_count = 400; assert(demag_comp_timer());
    assert(off == cuts + 1 && demag_comp_desync_pending() && !com_enabled);
    puts("PASS acceleration at constant duty requests relief but retains blind shutdown");

    reset(); thiszctime = 399; lastzctime = 400; demag_comp_zc();
    demag_comp_commutate(); scan(); assert(!cuts);
    reset(); duty_cycle = 900; scan(); assert(!cuts);
    puts("PASS small timing changes and falling duty do not request transient relief");

    reset(); thiszctime = lastzctime = 400;
    for (unsigned i = 0; i < 12; i++) {
        duty_cycle = (i & 1) ? 1000 : 1062;
        demag_comp_commutate(); scan(); assert(!cuts);
        demag_comp_zc();
    }
    duty_cycle = 1063; demag_comp_commutate(); scan(); assert(cuts == 1);
    puts("PASS duty jitter cannot rearm relief but a larger rise can");

    reset(); duty_cycle = 100;
    old_routine = 1; demag_comp_commutate(); old_routine = 0;
    thiszctime = lastzctime = 400;
    duty_cycle = 108; scan(); assert(!cuts); demag_comp_zc();
    duty_cycle = 117; demag_comp_commutate(); scan(); assert(cuts == 1);
    puts("PASS low-duty margin ignores eight counts and responds to nine");

    reset(); duty_cycle = 1400; thiszctime = lastzctime = 400; demag_comp_zc();
    old_routine = 1; demag_comp_commutate(); old_routine = 0;
    scan(); assert(!cuts);
    puts("PASS restart clears transient relief and rebases its duty reference");
#endif
    return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='demag-control-test-') as tmp:
    d = Path(tmp)
    for name, text in headers.items(): (d/name).write_text(text)
    (d/'test.c').write_text(source)
    for flags in ([], ['-DTEST_CUT_DISABLED', '-DDEMAG_COMP_POWER_OFF=256'],
                  ['-DTEST_CUT_DISABLED', '-DPWM_ENABLE_BRIDGE']):
        subprocess.run(['cc','-std=c99','-Wall','-Wextra','-Werror','-O2',*flags,'-I'+str(d),'-I'+str(root/'Inc'),str(root/'Src/demag_comp.c'),str(d/'test.c'),'-o',str(d/'test')],check=True)
        subprocess.run([str(d/'test')],check=True)
