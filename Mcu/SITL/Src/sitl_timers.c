/*
  sitl_timers.c - emulation of the timers AM32 uses on a G431:
    TIM1  PWM generation, 160MHz, ARR/CCR preloaded
    TIM2  INTERVAL_TIMER, 2MHz, 32 bit free running
    TIM6  TEN_KHZ_TIMER, periodic update interrupt (20kHz loop)
    TIM16 COM_TIMER, 2MHz, one-shot style update interrupt
    TIM17 UTILITY_TIMER, 1MHz, 16 bit free running

  Counters are derived from simulated time. Field writes from the firmware
  thread are plain volatile stores; consistency relies on x86 store
  ordering plus enable-last write order in the arm helpers.
 */

#include "sitl.h"
#include <stdio.h>

static SITL_TIM_TypeDef tims[SITL_NUM_TIMS];

// nanoseconds per CPU clock tick numerator/denominator: 160MHz = 6.25ns
#define CPU_TICK_NS_NUM 25
#define CPU_TICK_NS_DEN 4

static struct {
    // active (shadow) registers, only updated at update events
    uint32_t psc_act, arr_act, ccr_act[3];
    // preload values written by the firmware
    volatile uint32_t psc_pre, arr_pre, ccr_pre[3];
    volatile uint64_t period_start_ns;
} tim1;

static struct {
    volatile uint64_t base_ns;
    volatile uint32_t cnt_base;
} tim2, tim16;

static struct {
    volatile uint64_t next_due_ns;
    volatile bool enabled;
} tim6;

static uint64_t tim1_period_ns(void)
{
    const uint64_t ticks = (uint64_t)(tim1.psc_act + 1) * (tim1.arr_act + 1);
    return (ticks * CPU_TICK_NS_NUM) / CPU_TICK_NS_DEN;
}

static void tim1_latch(void)
{
    tim1.psc_act = tim1.psc_pre;
    tim1.arr_act = tim1.arr_pre;
    for (int i = 0; i < 3; i++) {
        tim1.ccr_act[i] = tim1.ccr_pre[i];
    }
}

void sitl_timers_init(void)
{
    tim1.psc_pre = 0;
    tim1.arr_pre = 1999; // overwritten by MX_TIM1_Init
    for (int i = 0; i < 3; i++) {
        tim1.ccr_pre[i] = 0;
    }
    tim1_latch();
    tim1.period_start_ns = 0;
    tims[SITL_TIM6_IDX].ARR = 50;
}

// current TIM1 counter position in timer ticks
static uint32_t tim1_cnt(uint64_t now_ns)
{
    const uint64_t elapsed = now_ns - tim1.period_start_ns;
    const uint64_t tick_ns_num = (uint64_t)CPU_TICK_NS_NUM * (tim1.psc_act + 1);
    // elapsed ticks = elapsed_ns * DEN / (NUM*(psc+1))
    return (uint32_t)((elapsed * CPU_TICK_NS_DEN) / tick_ns_num);
}

bool sitl_tim1_pwm_out(int chan, uint64_t now_ns)
{
    return tim1_cnt(now_ns) < tim1.ccr_act[chan];
}

/*
  dead time from the DTG field of the emulated TIM1 BDTR, decoded with
  the STM32 encoding at t_DTS = one 160MHz CPU tick (CKD is never
  changed by the firmware). The firmware sets this the same way as on
  hardware: DEAD_TIME at init plus dead_time_override ORed in from
  loadEEpromSettings
 */
uint32_t sitl_tim1_dead_time_ns(void)
{
    static uint32_t last_bdtr = 0xffffffff;
    static uint32_t dead_ns;
    const uint32_t bdtr = tims[SITL_TIM1_IDX].BDTR;
    if (bdtr != last_bdtr) {
        last_bdtr = bdtr;
        const uint32_t dtg = bdtr & 0xFF;
        uint32_t ticks;
        if ((dtg & 0x80) == 0) {
            ticks = dtg;
        } else if ((dtg & 0xC0) == 0x80) {
            ticks = (64 + (dtg & 0x3F)) * 2;
        } else if ((dtg & 0xE0) == 0xC0) {
            ticks = (32 + (dtg & 0x1F)) * 8;
        } else {
            ticks = (32 + (dtg & 0x1F)) * 16;
        }
        dead_ns = (ticks * CPU_TICK_NS_NUM) / CPU_TICK_NS_DEN;
        fprintf(stderr, "SITL: dead time %uns (BDTR 0x%02x)\n", dead_ns, dtg);
    }
    return dead_ns;
}

void sitl_tim1_set_duty_all(uint16_t duty)
{
    tim1.ccr_pre[0] = duty;
    tim1.ccr_pre[1] = duty;
    tim1.ccr_pre[2] = duty;
}

void sitl_tim1_set_duty(int chan, uint16_t duty)
{
    tim1.ccr_pre[chan] = duty;
}

void sitl_tim1_set_psc(uint16_t psc)
{
    tim1.psc_pre = psc;
}

void sitl_tim1_set_arr(uint16_t arr)
{
    tim1.arr_pre = arr;
}

// latched (active) TIM1 state, for tone detection in sitl_state.c
void sitl_tim1_get_active(uint32_t* psc, uint32_t* arr, uint32_t ccr[3])
{
    *psc = tim1.psc_act;
    *arr = tim1.arr_act;
    for (int i = 0; i < 3; i++) {
        ccr[i] = tim1.ccr_act[i];
    }
}

void sitl_tim1_force_update(void)
{
    tim1_latch();
    tim1.period_start_ns = sitl_time_ns();
}

/*
  INTERVAL_TIMER (TIM2), 2MHz
 */
static uint32_t interval_ticks_since(uint64_t now_ns, uint64_t base_ns)
{
    return (uint32_t)((now_ns - base_ns) / 500U);
}

uint32_t sitl_interval_timer_count(void)
{
    if (sitl_in_sim_thread()) {
        // interrupt context: busy waits on the timer must see time advance
        sitl_isr_read_tick();
    } else {
        sitl_fw_read_tick();
    }
    return tim2.cnt_base + interval_ticks_since(sitl_time_ns(), tim2.base_ns);
}

void sitl_interval_timer_set(uint32_t cnt)
{
    tim2.base_ns = sitl_time_ns();
    tim2.cnt_base = cnt;
}

/*
  COM_TIMER (TIM16), 2MHz, update interrupt used to time commutation
 */
void sitl_com_int_arm(uint32_t time)
{
    extern void motor_log_event(int kind, uint32_t a, uint32_t b, uint32_t c);
    motor_log_event(4 /*MEV_ZC_ACCEPT*/, time, 0, 0);
    SITL_TIM_TypeDef* t = &tims[SITL_TIM16_IDX];
    tim16.base_ns = sitl_time_ns();
    tim16.cnt_base = 0;
    t->ARR = time;
    t->SR = 0;
    // enable last: the sim thread only evaluates when DIER is set
    t->DIER |= 1;
}

void sitl_com_int_disable(void)
{
    tims[SITL_TIM16_IDX].DIER &= ~1U;
}

void sitl_com_int_enable(void)
{
    tims[SITL_TIM16_IDX].DIER |= 1;
}

void sitl_tenkhz_enable(void)
{
    tim6.next_due_ns = sitl_time_ns() + (tims[SITL_TIM6_IDX].ARR + 1) * 1000ULL;
    tim6.enabled = true;
}

/*
  called by the sim thread after each physics step
 */
void sitl_timers_step(uint64_t now_ns)
{
    // TIM1 update events latch the preload registers
    uint64_t period = tim1_period_ns();
    while (now_ns - tim1.period_start_ns >= period) {
        tim1.period_start_ns += period;
        tim1_latch();
        period = tim1_period_ns();
    }

    // 20kHz loop timer
    if (tim6.enabled && now_ns >= tim6.next_due_ns) {
        tim6.next_due_ns += (tims[SITL_TIM6_IDX].ARR + 1) * 1000ULL;
        sitl_irq_pend(SITL_IRQ_TENKHZ);
    }

    // commutation timer
    SITL_TIM_TypeDef* t16 = &tims[SITL_TIM16_IDX];
    if (t16->DIER & 1) {
        const uint32_t cnt = tim16.cnt_base + interval_ticks_since(now_ns, tim16.base_ns);
        if (cnt >= t16->ARR) {
            // counter wraps to zero and keeps running
            tim16.base_ns = now_ns;
            tim16.cnt_base = 0;
            t16->SR |= 1;
            sitl_irq_pend(SITL_IRQ_COM);
        }
    }
}

/*
  register style dereference, syncing CNT with simulated time
 */
SITL_TIM_TypeDef* sitl_tim_deref(int idx)
{
    SITL_TIM_TypeDef* t = &tims[idx];
    if (sitl_in_sim_thread()) {
        // interrupt context: firmware busy waiting on a timer (delayMicros
        // inside an IRQ handler) must still see time advance, as on real
        // hardware where the timers keep counting during an interrupt
        sitl_isr_read_tick();
    } else {
        sitl_fw_read_tick();
    }
    const uint64_t now = sitl_time_ns();
    switch (idx) {
    case SITL_TIM1_IDX:
        t->CNT = tim1_cnt(now);
        break;
    case SITL_TIM2_IDX:
        t->CNT = tim2.cnt_base + interval_ticks_since(now, tim2.base_ns);
        break;
    case SITL_TIM16_IDX:
        t->CNT = tim16.cnt_base + interval_ticks_since(now, tim16.base_ns);
        break;
    case SITL_TIM17_IDX:
        t->CNT = (uint32_t)((now / 1000U) & 0xffff);
        break;
    default:
        break;
    }
    return t;
}
