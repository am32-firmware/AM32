/*
  sitl_sched.c - simulated time, interrupt delivery and pacing for AM32 SITL

  The sim thread advances simulated time in fixed physics steps. Emulated
  interrupts are delivered by suspending the firmware thread with SIGUSR1
  (parking it on a semaphore) and running the handler in the sim thread,
  which reproduces the run-to-completion, mainline-frozen semantics of real
  interrupts. __disable_irq()/__enable_irq() map onto an atomic PRIMASK
  flag; while set, events stay pending exactly as on hardware.
 */

#include "sitl.h"
#include "sitl_config.h"

#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef __linux__
#include <sys/prctl.h>
#endif
#include <time.h>
#include <unistd.h>

#include "motor.h"

static volatile uint64_t sim_time_ns_v;
static pthread_t sim_thread_id;
static pthread_t fw_thread_id;

static volatile int primask; // 1 = interrupts disabled, atomic stores only

// simulated time granted by firmware-thread timer reads while it holds
// PRIMASK (a startup tune busy-waits on the utility timer under
// __disable_irq, so time must still advance for it). Reset when the
// critical section ends
static volatile uint64_t fw_grant_ns;
// sim time granted so far within the current PRIMASK section, and a
// counter identifying the section. Distinguishes a micros64-style
// disable/read/enable (sub-us) from a deliberate irqs-off delay loop
static volatile uint32_t section_grant_ns;
static volatile uint32_t primask_set_count;

/*
  firmware mainline progress lease. Real silicon can only freeze the
  mainline during interrupt execution; a host-descheduled firmware
  thread would otherwise let simulated time run milliseconds ahead of
  the mainline, which the firmware observes as impossible timing (and
  reports as desyncs). Every firmware-thread interception point renews
  the lease; the sim thread refuses to advance simulated time past it
  while the mainline is nominally runnable. ISR delivery and PRIMASK
  sections are exempt by construction (dispatch parks the firmware
  synchronously; grant-freeze handles PRIMASK)
 */
static volatile uint64_t fw_alive_until_ns;
static volatile int sim_gate_waiting;
static volatile uint32_t gate_stall_count;
static volatile uint64_t gate_stall_wall_ns;
static volatile uint64_t fw_max_gap_ns; // max sim time between mainline loops
static volatile uint32_t irq_pending;
static volatile uint32_t irq_enabled;
static volatile uint8_t irq_prio[SITL_IRQ_MAX];


/*
  park/resume semaphores. macOS has no unnamed POSIX semaphores or
  sem_timedwait, so it uses GCD semaphores instead
 */
#ifdef __APPLE__
#include <dispatch/dispatch.h>
typedef dispatch_semaphore_t sitl_sem_t;
static void sitl_sem_init(sitl_sem_t* s) { *s = dispatch_semaphore_create(0); }
static void sitl_sem_post(sitl_sem_t* s) { dispatch_semaphore_signal(*s); }
static void sitl_sem_wait(sitl_sem_t* s)
{
    dispatch_semaphore_wait(*s, DISPATCH_TIME_FOREVER);
}
static bool sitl_sem_wait_2s(sitl_sem_t* s)
{
    return dispatch_semaphore_wait(*s, dispatch_time(DISPATCH_TIME_NOW, 2LL * NSEC_PER_SEC)) == 0;
}
static bool sitl_sem_wait_10ms(sitl_sem_t* s)
{
    return dispatch_semaphore_wait(*s, dispatch_time(DISPATCH_TIME_NOW, 10LL * 1000 * 1000)) == 0;
}
#else
typedef sem_t sitl_sem_t;
static void sitl_sem_init(sitl_sem_t* s) { sem_init(s, 0, 0); }
static void sitl_sem_post(sitl_sem_t* s) { sem_post(s); }
static void sitl_sem_wait(sitl_sem_t* s)
{
    while (sem_wait(s) == -1 && errno == EINTR) {
    }
}
static bool sitl_sem_wait_2s(sitl_sem_t* s)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += 2;
    for (;;) {
        if (sem_timedwait(s, &ts) == 0) {
            return true;
        }
        if (errno != EINTR) {
            return false;
        }
    }
}
static bool sitl_sem_wait_10ms(sitl_sem_t* s)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_nsec += 10000000;
    if (ts.tv_nsec >= 1000000000) {
        ts.tv_sec++;
        ts.tv_nsec -= 1000000000;
    }
    for (;;) {
        if (sem_timedwait(s, &ts) == 0) {
            return true;
        }
        if (errno != EINTR) {
            return false;
        }
    }
}
#endif

static sitl_sem_t park_sem, resume_sem, gate_sem;

static void gate_wake(void)
{
    if (__atomic_exchange_n(&sim_gate_waiting, 0, __ATOMIC_SEQ_CST)) {
        sitl_sem_post(&gate_sem);
    }
}

/*
  small progress grant from an ordinary firmware-thread interception
  (timer read, comparator poll, PRIMASK set/clear). Extends the
  deadline by one loop quantum, capped at fw_lag_max_ns ahead of
  simulated now. The full lease is only re-armed at the loop boundary
  (sitl_fw_loop_tick): a firmware thread preempted mid-loop by the
  host must not stretch one loop iteration across many full leases
  of simulated time - that distortion is what still desynced the
  motor after the basic gate went in. A hot polling spin re-grants
  every read, so polled commutation never chokes
 */
void sitl_fw_progress(void)
{
    if (sitl_cfg.sim.fw_lag_max_ns == 0) {
        return;
    }
    const uint64_t now = sim_time_ns_v;
    const uint64_t dl = __atomic_load_n(&fw_alive_until_ns, __ATOMIC_SEQ_CST);
    uint64_t nd = (dl > now ? dl : now) + sitl_cfg.sim.loop_time_ns;
    const uint64_t cap = now + sitl_cfg.sim.fw_lag_max_ns;
    if (nd > cap) {
        nd = cap;
    }
    if (nd > dl) {
        __atomic_store_n(&fw_alive_until_ns, nd, __ATOMIC_SEQ_CST);
    }
    gate_wake();
}

// full lease re-arm at the mainline loop boundary
void sitl_fw_loop_tick(void)
{
    if (sitl_cfg.sim.fw_lag_max_ns == 0) {
        return;
    }
    __atomic_store_n(&fw_alive_until_ns,
        sim_time_ns_v + sitl_cfg.sim.fw_lag_max_ns, __ATOMIC_SEQ_CST);
    gate_wake();
}

static volatile uint64_t watchdog_last_reload_ns;
static volatile bool watchdog_running;
#define WATCHDOG_TIMEOUT_NS 2000000000ULL

char** sitl_saved_argv;
void sitl_coverage_flush(void);

// implemented in sitl_it.c
extern void sitl_irq_handler(int irq);
// implemented in sys_can_SITL.c when DroneCAN is compiled in
void sitl_can_poll(void) __attribute__((weak));
void sitl_can_poll(void) { }

uint64_t sitl_time_ns(void)
{
    return sim_time_ns_v;
}

bool sitl_in_sim_thread(void)
{
    return pthread_equal(pthread_self(), sim_thread_id);
}

static uint64_t wallclock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

uint64_t sitl_wallclock_ns(void)
{
    return wallclock_ns();
}

/*
  NVIC emulation
 */
void sitl_nvic_set_priority(int irq, uint32_t prio)
{
    irq_prio[irq] = prio;
}

void sitl_nvic_enable_irq(int irq)
{
    __atomic_fetch_or(&irq_enabled, 1U << irq, __ATOMIC_SEQ_CST);
}

void sitl_nvic_disable_irq(int irq)
{
    __atomic_fetch_and(&irq_enabled, ~(1U << irq), __ATOMIC_SEQ_CST);
}

/*
  dispatch block diagnostic: records why a pending COMP/COM interrupt is
  not being delivered, classified per simulation step, reported when the
  delivery latency exceeds 20us (bounded number of reports)
 */
static uint64_t irq_pend_ns[SITL_IRQ_MAX];
static int current_irq = -1;
static struct {
    uint32_t steps_primask;
    uint32_t steps_active[SITL_IRQ_MAX];
    uint32_t steps_disabled;
    uint32_t steps_free;
} blocked;

void sitl_irq_pend(int irq)
{
    const uint32_t old = __atomic_fetch_or(&irq_pending, 1U << irq, __ATOMIC_SEQ_CST);
    if ((old & (1U << irq)) == 0) {
        irq_pend_ns[irq] = sim_time_ns_v;
    }
}

void sitl_primask_set(void)
{
    // plain atomic store: the dispatcher re-checks primask after parking
    // the firmware thread, so no lock is needed. Critical sections are
    // extremely frequent (micros64 runs one per call) and must be cheap
    if (!sitl_in_sim_thread()) {
        sitl_fw_progress();
        __atomic_store_n(&section_grant_ns, 0, __ATOMIC_SEQ_CST);
        __atomic_fetch_add(&primask_set_count, 1, __ATOMIC_SEQ_CST);
    }
    __atomic_store_n(&primask, 1, __ATOMIC_SEQ_CST);
}

uint32_t sitl_primask_get(void)
{
    return (uint32_t)__atomic_load_n(&primask, __ATOMIC_SEQ_CST);
}

void sitl_primask_clear(void)
{
    if (!sitl_in_sim_thread()) {
        // grants are per critical section: unconsumed ones must not
        // accumulate into a reservoir that lets the simulation run
        // through a later, host-stretched critical section
        __atomic_store_n(&fw_grant_ns, 0, __ATOMIC_SEQ_CST);
        sitl_fw_progress();
    }
    __atomic_store_n(&primask, 0, __ATOMIC_SEQ_CST);
}

/*
  firmware thread suspension. SIGUSR1 parks the firmware thread on
  resume_sem. Blocking in a handler is not formally async-signal-safe,
  but these are bare syscall wrappers taking no library locks; the one
  real hazard is parking this thread inside a locked stdio call while
  the sim thread also prints, so diagnostics prints are kept rare
 */
static void sigusr1_handler(int sig)
{
    (void)sig;
    const int saved_errno = errno;
    sitl_sem_post(&park_sem);
    sitl_sem_wait(&resume_sem);
    errno = saved_errno;
}

static bool suspend_firmware(void)
{
    pthread_kill(fw_thread_id, SIGUSR1);
    // a timed wait so a firmware thread that is exiting or execing
    // (NVIC_SystemReset) cannot deadlock the simulation
    return sitl_sem_wait_2s(&park_sem);
}

static void resume_firmware(void)
{
    sitl_sem_post(&resume_sem);
}

// priority of the currently executing handler, NVIC style (lower value
// is higher priority). 1000 = thread level, nothing active
static int active_irq_prio = 1000;


// run any pending interrupt with higher priority than the active one.
// Called from the dispatch loop and re-entrantly from sitl_isr_read_tick
// so a long running handler (eg tenKhzRoutine busy waiting on a timer)
// can be preempted by the comparator, as the NVIC does on real hardware
static void run_pending_irqs(void)
{
    for (;;) {
        if (primask) {
            return;
        }
        const uint32_t active = irq_pending & irq_enabled;
        if (active == 0) {
            return;
        }
        int best = -1;
        for (int irq = 0; irq < SITL_IRQ_MAX; irq++) {
            if ((active & (1U << irq)) == 0) {
                continue;
            }
            if (best < 0 || irq_prio[irq] < irq_prio[best]) {
                best = irq;
            }
        }
        if (irq_prio[best] >= active_irq_prio) {
            // equal or lower priority does not preempt
            return;
        }
        __atomic_fetch_and(&irq_pending, ~(1U << best), __ATOMIC_SEQ_CST);
        if (best == SITL_IRQ_COMP || best == SITL_IRQ_COM) {
            const uint64_t lat = sim_time_ns_v - irq_pend_ns[best];
            static int prints;
            if (lat > 20000 && prints < 12) {
                prints++;
                fprintf(stderr,
                    "SITL: irq %d blocked %.1fus at t=%.4f: primask=%u dis=%u free=%u"
                    " act=[%u,%u,%u,%u,%u,%u]\n",
                    best, lat * 1e-3, sim_time_ns_v * 1e-9,
                    blocked.steps_primask, blocked.steps_disabled, blocked.steps_free,
                    blocked.steps_active[0], blocked.steps_active[1],
                    blocked.steps_active[2], blocked.steps_active[3],
                    blocked.steps_active[4], blocked.steps_active[5]);
            }
            memset(&blocked, 0, sizeof(blocked));
        }
        const int saved_prio = active_irq_prio;
        const int saved_irq = current_irq;
        active_irq_prio = irq_prio[best];
        current_irq = best;
        const uint64_t isr_t0 = sim_time_ns_v;
        sitl_irq_handler(best);
        current_irq = saved_irq;
        active_irq_prio = saved_prio;
        if (sitl_cfg.sim.fw_lag_max_ns != 0 && saved_irq < 0) {
            /*
              base-level handler complete. Its execution time is
              lease-exempt: push the mainline progress deadline out by
              exactly the sim time it consumed (including any nested
              preemptions, which is why nested levels do not add).
              If the lease is expired, stop tail-chaining so the parked
              firmware is resumed and the gate can wait for mainline
              progress before the next handler - a dense comparator
              edge storm must not starve the mainline for its whole
              duration, which the firmware only survives when the host
              keeps its thread running (SCHED_FIFO behaviour)
             */
            __atomic_fetch_add(&fw_alive_until_ns, sim_time_ns_v - isr_t0, __ATOMIC_SEQ_CST);
            if (sim_time_ns_v >= __atomic_load_n(&fw_alive_until_ns, __ATOMIC_SEQ_CST)) {
                return;
            }
        }
    }
}

/*
  deliver pending enabled interrupts, called from the sim thread between
  physics steps
 */
static void sitl_dispatch(void)
{
    if ((irq_pending & irq_enabled) == 0 || primask) {
        return;
    }
    if (!suspend_firmware()) {
        return;
    }
    // the firmware may have entered a critical section between our check
    // and it parking; if so it is now parked inside the section and we
    // must not run handlers (an IRQ arriving just after cpsid is held
    // pending on real hardware too)
    if (!primask) {
        run_pending_irqs();
    }
    resume_firmware();
}

/*
  watchdog
 */
void sitl_watchdog_reload(void)
{
    extern void motor_log_mainloop(void);
    motor_log_mainloop();
    if (!sitl_in_sim_thread()) {
        const uint64_t gap = sim_time_ns_v - watchdog_last_reload_ns;
        if (watchdog_last_reload_ns != 0 && gap > fw_max_gap_ns) {
            fw_max_gap_ns = gap;
        }
        sitl_fw_loop_tick();
    }
    watchdog_last_reload_ns = sim_time_ns_v;
    if (!sitl_in_sim_thread() && sitl_cfg.sim.loop_time_ns > 0 && sitl_cfg.speedup > 0) {
        // approximate the real main loop execution time so the firmware
        // thread does not spin flat out
        const uint64_t delay_ns = (uint64_t)(sitl_cfg.sim.loop_time_ns / sitl_cfg.speedup);
        if (sitl_cfg.nosleep) {
            // yield in the busy wait: still far tighter than the 50us
            // sleep slack, and on a host with fewer cores than busy
            // threads a pure spin here starves the sim thread
            const uint64_t deadline = wallclock_ns() + delay_ns;
            while (wallclock_ns() < deadline) {
                sched_yield();
            }
        } else {
            struct timespec ts = { 0, (long)delay_ns };
            nanosleep(&ts, NULL);
        }
        // the pacing sleep may have been stretched by the host
        sitl_fw_progress();
    }
}

void sitl_watchdog_enable(void)
{
    watchdog_last_reload_ns = sim_time_ns_v;
    watchdog_running = sitl_cfg.sim.watchdog_enabled;
}

static void watchdog_check(void)
{
    if (watchdog_running && sim_time_ns_v - watchdog_last_reload_ns > WATCHDOG_TIMEOUT_NS) {
        fprintf(stderr, "SITL: watchdog reset at t=%.3fs\n", sim_time_ns_v * 1.0e-9);
        sitl_reset_with_cause("watchdog");
    }
}

/*
  exec the bootloader elf, passing the shared config and our own argv
  after -- as the application exec vector, so its jump_to_application
  lands back here. Used at startup (hardware boots into the bootloader
  first) and on reset when --bootloader is configured
 */
void sitl_exec_bootloader(const char* cause)
{
    static char portbuf[16], speedbuf[24];
    const char* argv[64];
    int n = 0;
    argv[n++] = sitl_cfg.bootloader_path;
    argv[n++] = "--eeprom";
    argv[n++] = sitl_cfg.eeprom_path;
    snprintf(portbuf, sizeof(portbuf), "%d", sitl_cfg.input_port);
    argv[n++] = "--input-port";
    argv[n++] = portbuf;
    argv[n++] = "--can-uri";
    argv[n++] = sitl_cfg.can_uri;
    snprintf(speedbuf, sizeof(speedbuf), "%g", (double)sitl_cfg.speedup);
    argv[n++] = "--speedup";
    argv[n++] = speedbuf;
    if (sitl_cfg.bind_any) {
        argv[n++] = "--bind-any";
    }
    if (sitl_cfg.uid != NULL) {
        argv[n++] = "--uid";
        argv[n++] = sitl_cfg.uid;
    }
    argv[n++] = "--reset-cause";
    argv[n++] = cause;
    argv[n++] = "--";
    for (int i = 0; sitl_saved_argv[i] != NULL && n < (int)(sizeof(argv) / sizeof(argv[0])) - 1; i++) {
        argv[n++] = sitl_saved_argv[i];
    }
    argv[n] = NULL;
    fflush(NULL);
    sitl_coverage_flush();
    execv(sitl_cfg.bootloader_path, (char* const*)argv);
    fprintf(stderr, "SITL: exec bootloader %s failed: %s\n",
        sitl_cfg.bootloader_path, strerror(errno));
    _exit(1);
}

void sitl_reset_with_cause(const char* cause)
{
    // block the suspension signal so a concurrent interrupt delivery
    // cannot park this thread on the way into exec
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGUSR1);
    pthread_sigmask(SIG_BLOCK, &set, NULL);
    fprintf(stderr, "SITL: reset (%s) at t=%.3fs\n", cause, sim_time_ns_v * 1.0e-9);
    if (sitl_cfg.bootloader_path != NULL) {
        // a reset lands in the bootloader, as on hardware
        sitl_exec_bootloader(cause);
    }
#ifdef __APPLE__
    sitl_coverage_flush();
    execv(sitl_saved_argv[0], sitl_saved_argv);
#else
    sitl_coverage_flush();
    execv("/proc/self/exe", sitl_saved_argv);
#endif
    fprintf(stderr, "SITL: execv failed: %s\n", strerror(errno));
    _exit(1);
}

void sitl_system_reset(void)
{
    sitl_reset_with_cause("software");
}

/*
  advance simulation by one physics step. Called from the sim thread main
  loop and re-entrantly from interrupt handlers that busy wait on time
  (delayMicros, comparator filter reads)
 */
static void sim_step_once(void)
{
    const uint32_t dt = sitl_cfg.sim.physics_dt_ns;
    motor_step(sim_time_ns_v, dt);
    sim_time_ns_v += dt;
    sitl_timers_step(sim_time_ns_v);
    sitl_state_step(sim_time_ns_v);

    // dispatch block tracer: classify this step if a priority 0 interrupt
    // has been pending for a while
    for (int irq = SITL_IRQ_COMP; irq <= SITL_IRQ_COM; irq++) {
        if ((irq_pending & (1U << irq)) == 0) {
            continue;
        }
        if (sim_time_ns_v - irq_pend_ns[irq] <= 20000) {
            continue;
        }
        if (current_irq >= 0) {
            // a handler is executing (possibly holding primask itself)
            blocked.steps_active[current_irq]++;
        } else if (primask) {
            blocked.steps_primask++;
        } else if ((irq_enabled & (1U << irq)) == 0) {
            blocked.steps_disabled++;
        } else {
            blocked.steps_free++;
        }
        break;
    }
}

void sitl_step_from_isr(void)
{
    sim_step_once();
}

void sitl_fw_read_tick(void)
{
    if (!sitl_in_sim_thread()) {
        sitl_fw_progress();
        if (primask) {
            __atomic_fetch_add(&fw_grant_ns, sitl_cfg.sim.isr_read_ns, __ATOMIC_SEQ_CST);
            __atomic_fetch_add(&section_grant_ns, sitl_cfg.sim.isr_read_ns, __ATOMIC_SEQ_CST);
        }
    }
}

void sitl_isr_read_tick(void)
{
    // only called from the sim thread (interrupt context), no locking
    // needed. Advancing a full physics step per register read would make
    // handler busy loops take ~10x longer in simulated time than on real
    // hardware, which breaks the comparator filter in interruptRoutine()
    static uint32_t accum_ns;
    accum_ns += sitl_cfg.sim.isr_read_ns;
    while (accum_ns >= sitl_cfg.sim.physics_dt_ns) {
        accum_ns -= sitl_cfg.sim.physics_dt_ns;
        sim_step_once();
        // let higher priority interrupts preempt the current handler
        run_pending_irqs();
    }
}

// try to switch the calling thread to SCHED_FIFO, warning on failure
static void set_realtime(const char* what)
{
    if (!sitl_cfg.realtime) {
        return;
    }
    struct sched_param sp = { .sched_priority = 50 };
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
        fprintf(stderr, "SITL: SCHED_FIFO failed for %s thread: %s\n",
            what, strerror(errno));
        return;
    }
    // with --nosleep the threads never block, so default RT throttling
    // (kernel.sched_rt_runtime_us=950000) would stall them 50ms/second
    if (sitl_cfg.nosleep) {
        FILE* f = fopen("/proc/sys/kernel/sched_rt_runtime_us", "r");
        if (f) {
            long v = 0;
            if (fscanf(f, "%ld", &v) == 1 && v != -1) {
                fprintf(stderr,
                    "SITL: WARNING: RT throttling is enabled and will stall "
                    "--nosleep --realtime; run "
                    "'sysctl -w kernel.sched_rt_runtime_us=-1'\n");
            }
            fclose(f);
        }
    }
    fprintf(stderr, "SITL: %s thread using SCHED_FIFO\n", what);
}

static void* sim_thread_main(void* arg)
{
    (void)arg;
    set_realtime("sim");
#ifdef __linux__
    // timer slack defaults to 50us which ruins short pacing sleeps
    prctl(PR_SET_TIMERSLACK, 1UL);
#endif
    const uint64_t wall0 = wallclock_ns();
    uint64_t next_can_poll_ns = 0;
    uint64_t next_pace_check_ns = 0;
    uint64_t verbose_last_ns = 0;
    uint64_t verbose_last_wall = wall0;
    uint64_t pace_wall_ref = wall0;
    uint64_t pace_sim_ref = 0;
    float pace_speedup = sitl_cfg.speedup;

    uint32_t grant_accum_ns = 0;
    for (;;) {
        /*
          while the firmware thread holds PRIMASK outside of interrupt
          context, simulated time may only advance as granted by the
          firmware's own timer reads. On the real MCU a critical section
          lasts nanoseconds; if the host deschedules the firmware thread
          inside one (OS preemption, hard/soft irqs on its core), a free
          running clock would block interrupt delivery for hundreds of
          microseconds of simulated time and lose commutation timing.
          current_irq is only ever set by this thread, so the check does
          not race: ISR-context PRIMASK always has current_irq >= 0
         */
        if (primask && current_irq < 0) {
            /*
              a deliverable interrupt is pending and the critical section
              is still short: almost certainly a micros64-style
              disable/read/enable window. On silicon the interrupt is
              delivered at the enable, nanoseconds away. Consuming the
              read's grant NOW would advance simulated time with PRIMASK
              set and block delivery - harmless when the sim runs ahead
              of the firmware (the grant is consumed later, after the
              enable), but when the sim is starved and hungry it eats
              every grant inside the window, stretching interrupt latency
              to the wall-clock length of the firmware's read run. That
              was the desync mechanism under host load. Wait for the
              enable instead; a genuine irqs-off delay loop grows
              section_grant_ns past the threshold and is consumed
              normally (interrupts blocked - as on silicon)
             */
            if ((irq_pending & irq_enabled) != 0 &&
                __atomic_load_n(&section_grant_ns, __ATOMIC_SEQ_CST) < 2000) {
                static uint32_t gave_up_section;
                const uint32_t sect = __atomic_load_n(&primask_set_count, __ATOMIC_SEQ_CST);
                if (sect != gave_up_section) {
                    const uint64_t w0 = wallclock_ns();
                    while (primask && current_irq < 0 &&
                           (irq_pending & irq_enabled) != 0 &&
                           __atomic_load_n(&section_grant_ns, __ATOMIC_SEQ_CST) < 2000 &&
                           __atomic_load_n(&primask_set_count, __ATOMIC_SEQ_CST) == sect) {
                        if (wallclock_ns() - w0 > 10000000) {
                            gave_up_section = sect;
                            break;
                        }
                        sched_yield();
                    }
                    continue;
                }
            }
            const uint64_t g = fw_grant_ns;
            if (g > 0) {
                __atomic_fetch_sub(&fw_grant_ns, g, __ATOMIC_SEQ_CST);
                grant_accum_ns += (uint32_t)g;
            }
            if (grant_accum_ns < sitl_cfg.sim.physics_dt_ns) {
                continue;
            }
            grant_accum_ns -= sitl_cfg.sim.physics_dt_ns;
        } else {
            grant_accum_ns = 0;
            /*
              mainline progress gate: refuse to advance simulated time
              past the firmware's lease. Pending deliverable interrupts
              are still dispatched (the mainline may be waiting on state
              only a handler can change); the stall keeps UDP polls
              alive on the wall clock. After a stall the pacer rebases
              instead of sprinting - wall divergence is acceptable,
              starvation is not
             */
            if (sitl_cfg.sim.fw_lag_max_ns != 0) {
                bool stalled = false;
                uint64_t this_stall_wall_ns = 0;
                uint64_t next_stall_poll = 0;
                while (sim_time_ns_v >= __atomic_load_n(&fw_alive_until_ns, __ATOMIC_SEQ_CST)) {
                    __atomic_store_n(&sim_gate_waiting, 1, __ATOMIC_SEQ_CST);
                    if (sim_time_ns_v < __atomic_load_n(&fw_alive_until_ns, __ATOMIC_SEQ_CST)) {
                        __atomic_store_n(&sim_gate_waiting, 0, __ATOMIC_SEQ_CST);
                        break;
                    }
                    if (!stalled) {
                        stalled = true;
                        gate_stall_count++;
                    }
                    const uint64_t w0 = wallclock_ns();
                    bool renewed;
                    if (sitl_cfg.nosleep) {
                        // this wait is FOR the firmware thread: yield so
                        // it can run when it shares our core
                        while (!(renewed = sim_time_ns_v < __atomic_load_n(&fw_alive_until_ns, __ATOMIC_SEQ_CST)) && wallclock_ns() - w0 < 10000000) {
                            sched_yield();
                        }
                    } else {
                        renewed = sitl_sem_wait_10ms(&gate_sem);
                    }
                    this_stall_wall_ns += wallclock_ns() - w0;
                    gate_stall_wall_ns += wallclock_ns() - w0;
                    if (!renewed && (irq_pending & irq_enabled) != 0 && !primask) {
                        // liveness fallback: the mainline may be spinning on
                        // an ISR-set flag without any intercepted reads.
                        // Deliver pending interrupts (this extends the
                        // deadline by the handler time, so a comparator
                        // edge storm still cannot refuel the lease)
                        sitl_dispatch();
                    }
                    if (wallclock_ns() >= next_stall_poll) {
                        next_stall_poll = wallclock_ns() + 1000000;
                        sitl_can_poll();
                        sitl_input_poll();
                        sitl_state_poll();
                    }
                }
                // rebase the wall pacer only after a genuine stall:
                // micro-stalls happen thousands of times per second and
                // rebasing on each would forgive the pacer's 100us sleep
                // threshold every time, letting simulated time run ahead
                // of the wall clock and starve the mainline
                if (stalled && this_stall_wall_ns > 1000000) {
                    pace_wall_ref = wallclock_ns();
                    pace_sim_ref = sim_time_ns_v;
                }
            }
        }
        sim_step_once();
        const uint64_t now = sim_time_ns_v;

        if (now >= next_can_poll_ns) {
            next_can_poll_ns = now + 100000; // 100us
            sitl_can_poll();
            sitl_input_poll();
            sitl_state_poll();
        }
        watchdog_check();
        sitl_dispatch();

        // pace simulated time against the wall clock, sleeping to an
        // absolute deadline so overshoot does not accumulate. The
        // references rebase when the speedup changes at runtime (GUI
        // slow motion control) so the mapping stays continuous
        if (sitl_cfg.speedup != pace_speedup) {
            pace_speedup = sitl_cfg.speedup;
            pace_wall_ref = wallclock_ns();
            pace_sim_ref = now;
        }
        if (pace_speedup > 0 && now >= next_pace_check_ns) {
            next_pace_check_ns = now + 50000; // check every 50us of sim time
            const uint64_t target_wall = pace_wall_ref + (uint64_t)((double)(now - pace_sim_ref) / pace_speedup);
            const uint64_t wall = wallclock_ns();
            // a wall clock DISCONTINUITY between consecutive pacing
            // checks means the whole process was stopped (a debugger,
            // SIGSTOP, a laptop suspend): rebase and carry on at the
            // configured pace instead of sprinting through the backlog,
            // which would compress it into a burst of impossible
            // timing. A merely slow host accumulates its deficit in
            // small per-check deltas and keeps the normal catch-up
            // behaviour
            static uint64_t last_pace_wall;
            const bool stopped = last_pace_wall != 0 && wall - last_pace_wall > 250000000ULL;
            last_pace_wall = wall;
            if (stopped && wall > target_wall) {
                fprintf(stderr, "SITL: wall clock jumped %.1fs, pacing rebased at t=%.3fs\n",
                    (wall - target_wall) * 1e-9, now * 1e-9);
                pace_wall_ref = wall;
                pace_sim_ref = now;
            } else if (sitl_cfg.nosleep) {
                while (wallclock_ns() < target_wall) {
                    sched_yield();
                }
            } else if (target_wall > wall + 100000) {
#ifdef __APPLE__
                // no clock_nanosleep on macOS: relative sleep. Drift free
                // because the absolute target is recomputed each pass
                const uint64_t delta = target_wall - wall;
                struct timespec ts = { delta / 1000000000ULL, delta % 1000000000ULL };
                nanosleep(&ts, NULL);
#else
                struct timespec ts = { target_wall / 1000000000ULL, target_wall % 1000000000ULL };
                clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
#endif
            }
        }

        if (sitl_cfg.verbose && now - verbose_last_ns >= 1000000000ULL) {
            const uint64_t wall = wallclock_ns();
            const float ratio = (float)(now - verbose_last_ns) / (float)(wall - verbose_last_wall);
            verbose_last_ns = now;
            verbose_last_wall = wall;
            motor_print_state(now, ratio);
            if (gate_stall_count > 0) {
                fprintf(stderr, "SITL: gate stalls=%u stalled=%.1fms max_gap=%.1fus\n",
                    gate_stall_count, gate_stall_wall_ns * 1e-6,
                    fw_max_gap_ns * 1e-3);
            }
        }
    }
    return NULL;
}

void sitl_start_sim_thread(void)
{
    fw_thread_id = pthread_self();
    set_realtime("firmware");
#ifdef __linux__
    // timer slack is per thread and defaults to 50us
    prctl(PR_SET_TIMERSLACK, 1UL);
#endif
    sitl_sem_init(&park_sem);
    sitl_sem_init(&resume_sem);
    sitl_sem_init(&gate_sem);

    // sitl_system_reset blocks SIGUSR1 on the way into execv; both the
    // mask and a possibly pending SIGUSR1 survive exec. Discard any
    // pending instance and unblock before installing the real handler
    signal(SIGUSR1, SIG_IGN);
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGUSR1);
    pthread_sigmask(SIG_UNBLOCK, &set, NULL);

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = sigusr1_handler;
    sa.sa_flags = SA_RESTART;
    sigaction(SIGUSR1, &sa, NULL);

    pthread_create(&sim_thread_id, NULL, sim_thread_main, NULL);
}
