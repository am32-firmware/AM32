/*
  motor.c - trapezoidal BEMF BLDC motor, 3 phase bridge and battery model
  for AM32 SITL.

  The electrical model follows the approach of open-bldc-csim
  (https://github.com/open-bldc/open-bldc-csim, GPLv3+, Piotr
  Esden-Tempski), which in turn implements Kang & Yoo, "Switching
  Pattern-Independent Simulation Model for Brushless DC Motors", Journal
  of Power Electronics 11-2, 2011:
  https://jpels.org/digital-library/manuscript/file/17706/8_JPE-10238.pdf

  As in the paper, each step finds the conducting phases from the switch
  and diode states, computes the motor neutral as the average of (v - e)
  over the conducting phases (paper eq 10) and integrates
  v = R*i + (L-M)*di/dt + e + v_m for each of them (paper eq 1), so the
  floating phase terminal voltage (and therefore the BEMF zero crossing
  seen by the comparator) is physical for any switching pattern.

  Differences from the paper:
   - gate states come from the emulated TIM1 compare outputs and the AM32
     phase modes rather than switching functions, which adds dead time
     windows with body diode conduction
   - fets have Rds_on, and the diode Vf appears in the clamped terminal
     voltage, not only in the diode on/off conditions
   - the battery has internal resistance, so Vdc sags with load instead
     of being stiff
   - the load model adds a k*omega^2 propeller torque and static friction
     to the paper's viscous damping
   - torque uses the flux linkage form kt*sum(shape*i) (the second form
     of paper eq 2), which is valid at omega = 0
   - integration is forward euler at 500ns steps rather than ode45 at
     2.5us

  Conventions:
   - phase currents are positive INTO the motor terminal
   - terminal voltages are referenced to battery negative
   - theta is the mechanical rotor angle in radians
 */

#include "motor.h"

#include <math.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "sitl.h"
#include "sitl_config.h"
#include "eeprom.h"

#define TWO_PI 6.283185307179586

// which EXTI line the active comparator drives, owned by comparator.c
extern uint32_t current_EXTI_LINE;

static void check_desync_dump(void);
static void dump_ring(void);

static struct {
    double theta; // mechanical angle, rad
    double omega; // mechanical speed, rad/s
    double i[3]; // phase currents, A
    double ke; // V/(rad/s) mechanical
    double vbus; // battery terminal voltage after sag
    double vc; // bus capacitor voltage (battery.capacitance > 0)
    double ibus; // battery current, previous step (breaks the loop)
    // dead time tracking: per phase last PWM level and the end of the
    // current both-off window
    bool pwm_last[3];
    uint64_t dead_until[3];
    // driven set of the previous step, for commutation detection
    bool driven_last[3];
    // last terminal voltages for the state stream
    double v_term[3];
    // comparator analog front end: RC-filtered phase divider nodes and
    // the filtered virtual-neutral node, plus band-limited input noise
    double v_cmp[3];
    double vn_cmp;
    double noise_cmp;
    // persistent per-phase body-diode conduction state: 0 = off,
    // +1 = high-side diode (terminal at vbus+vf, current <= 0),
    // -1 = low-side diode (terminal at -vf, current >= 0). A diode
    // stays on until its current reaches zero (found by sub-step
    // event interpolation) and re-arms only on a driven->open
    // transition with current or on a rail-crossing terminal voltage.
    // The old per-step |i|>10mA reclassification produced an off/on
    // limit cycle that chattered the comparator at MHz rates
    int8_t diode_state[3];
    // solver diagnostics, printed with --verbose
    struct {
        uint32_t diode_on_i; // diode armed by freewheel current
        uint32_t diode_on_v; // diode armed by rail-crossing voltage
        uint32_t diode_off; // zero-current turn-off events
        uint32_t sign_viol; // conducting diode with wrong-sign current
        uint32_t ev_overflow; // sub-step event budget exhausted
        uint32_t comp_edges; // comparator output toggles
    } diag;
    // last electromagnetic torque, for the audio stream
    double tau;
    // cumulative energy ledger (joules), reported in the physics log:
    // electromagnetic conversion, copper, bridge (fet+diode), bus
    // exchange, sink dissipation, mechanical drag
    double e_em, e_cu, e_br, e_bus, e_sink, e_drag;
    // sensor averaging accumulators
    double acc_v, acc_i;
    uint32_t acc_n;
    unsigned rand_seed;
    // seqlock for the sensor snapshot
    volatile uint32_t seq;
    sitl_sensors_t sensors;
} m;

void motor_config_changed(void)
{
    // per phase BEMF constant [V s/rad] from Kv [rpm/V]. Kv is defined
    // line to line and two phases conduct in series, so halve it. All
    // other motor parameters are read from sitl_cfg on every step
    m.ke = 0.5 * 60.0 / (TWO_PI * sitl_cfg.motor.kv);
}

void motor_add_signals(double acc[8])
{
    for (int p = 0; p < 3; p++) {
        acc[p] += m.i[p];
        acc[3 + p] += m.v_term[p];
    }
    acc[6] += m.vbus;
    acc[7] += m.ibus;
}

// per-step signals for the physics audio stream: what a motor radiates
// acoustically is dominated by torque ripple (rotor/mount vibration)
// and radial magnetic force, which tracks the phase current magnitude
void motor_add_audio(double acc[2])
{
    acc[0] += m.tau;
    acc[1] += fabs(m.i[0]) + fabs(m.i[1]) + fabs(m.i[2]);
}

void motor_get_live_state(float* omega, float* theta, float* theta_e,
                          float i[3], float v[3], float* vbus, float* ibus)
{
    for (int p = 0; p < 3; p++) {
        v[p] = (float)m.v_term[p];
    }
    const int pole_pairs = sitl_cfg.motor.poles / 2;
    *omega = (float)m.omega;
    double th = fmod(m.theta, TWO_PI);
    if (th < 0) {
        th += TWO_PI;
    }
    *theta = (float)th;
    double the = fmod(m.theta * pole_pairs, TWO_PI);
    if (the < 0) {
        the += TWO_PI;
    }
    *theta_e = (float)the;
    for (int p = 0; p < 3; p++) {
        i[p] = (float)m.i[p];
    }
    *vbus = (float)m.vbus;
    *ibus = (float)m.ibus;
}

void motor_init(void)
{
    memset(&m, 0, sizeof(m));
    motor_config_changed();
    m.vbus = sitl_cfg.battery.voltage;
    m.vc = m.vbus;
    m.rand_seed = 12345;
    m.sensors.bus_voltage = m.vbus;
    m.sensors.temperature_c = sitl_cfg.esc.temperature_c;
}

/*
  normalised trapezoidal BEMF shape over one electrical revolution.
  Rising zero crossing at 0, falling at pi, flat top from pi/6..5pi/6
 */
static double trap_shape(double thetae)
{
    thetae = fmod(thetae, TWO_PI);
    if (thetae < 0) {
        thetae += TWO_PI;
    }
    const double s = M_PI / 6; // 30 degree ramp
    if (thetae < s) {
        return thetae / s;
    }
    if (thetae < M_PI - s) {
        return 1.0;
    }
    if (thetae < M_PI + s) {
        return (M_PI - thetae) / s;
    }
    if (thetae < TWO_PI - s) {
        return -1.0;
    }
    return (thetae - TWO_PI) / s;
}

void sitl_sensors_write(const sitl_sensors_t* in)
{
    m.seq++;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    m.sensors = *in;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    m.seq++;
}

void sitl_sensors_read(sitl_sensors_t* out)
{
    for (;;) {
        const uint32_t s1 = m.seq;
        __atomic_thread_fence(__ATOMIC_SEQ_CST);
        *out = m.sensors;
        __atomic_thread_fence(__ATOMIC_SEQ_CST);
        if (s1 == m.seq && (s1 & 1) == 0) {
            return;
        }
    }
}

/*
  physics log writer thread: the sim thread formats lines into a ring
  and this thread writes them out, so disk latency never stalls the
  physics. Overflow drops lines (counted) instead of blocking
 */
#define PLOG_LINE_MAX 384
#define PLOG_RING 1024
static struct {
    char lines[PLOG_RING][PLOG_LINE_MAX];
    _Atomic unsigned head; // written by sim thread
    _Atomic unsigned tail; // written by logger thread
    _Atomic unsigned dropped;
    bool started;
    bool failed;
    pthread_t thread;
} plog;

static void* plog_thread(void* arg)
{
    (void)arg;
    FILE* f = fopen(sitl_cfg.physics_log, "a");
    if (!f) {
        fprintf(stderr, "SITL: failed to open physics log %s\n", sitl_cfg.physics_log);
        return NULL;
    }
    unsigned last_dropped = 0;
    for (;;) {
        unsigned tail = atomic_load_explicit(&plog.tail, memory_order_relaxed);
        const unsigned head = atomic_load_explicit(&plog.head, memory_order_acquire);
        while (tail != head) {
            fputs(plog.lines[tail % PLOG_RING], f);
            tail++;
            atomic_store_explicit(&plog.tail, tail, memory_order_release);
        }
        fflush(f);
        const unsigned dropped = atomic_load_explicit(&plog.dropped, memory_order_relaxed);
        if (dropped != last_dropped) {
            fprintf(stderr, "SITL: physics log dropped %u lines\n",
                dropped - last_dropped);
            last_dropped = dropped;
        }
        usleep(20000);
    }
    return NULL;
}

static void plog_push(const char* line)
{
    if (!plog.started) {
        plog.started = true;
        if (pthread_create(&plog.thread, NULL, plog_thread, NULL) != 0) {
            fprintf(stderr, "SITL: failed to start physics log thread\n");
            plog.failed = true;
        }
    }
    if (plog.failed) {
        return;
    }
    const unsigned head = atomic_load_explicit(&plog.head, memory_order_relaxed);
    const unsigned tail = atomic_load_explicit(&plog.tail, memory_order_acquire);
    if (head - tail >= PLOG_RING) {
        atomic_fetch_add_explicit(&plog.dropped, 1, memory_order_relaxed);
        return;
    }
    strncpy(plog.lines[head % PLOG_RING], line, PLOG_LINE_MAX - 1);
    plog.lines[head % PLOG_RING][PLOG_LINE_MAX - 1] = 0;
    atomic_store_explicit(&plog.head, head + 1, memory_order_release);
}

void motor_step(uint64_t now_ns, uint32_t dt_ns)
{
    const double dt = dt_ns * 1e-9;
    const int pole_pairs = sitl_cfg.motor.poles / 2;
    const double R = sitl_cfg.motor.resistance;
    const double L_eff = sitl_cfg.motor.inductance - sitl_cfg.motor.mutual_inductance;
    const double rds = sitl_cfg.esc.rds_on;
    const double vf = sitl_cfg.esc.diode_vf;

    // battery sag from last step's bus current
    double vbus;
    if (sitl_cfg.battery.capacitance > 0) {
        // bus capacitor fed from the supply through its internal
        // resistance. Above the set voltage the supply cannot absorb
        // regen; it only sinks through sink_resistance (a bench PSU
        // downprogrammer plus other bus loads; 0 = no sink at all), so
        // braking pumps the bus up, as measured on real hardware
        const double cap = sitl_cfg.battery.capacitance;
        const double vset = sitl_cfg.battery.voltage;
        if (m.vc <= vset) {
            const double r_src = sitl_cfg.battery.resistance > 1e-4 ? sitl_cfg.battery.resistance : 1e-4;
            // move vc toward the set voltage by the RC fraction, capped
            // at 1 so a tiny r*C stays stable at any physics step
            double alpha = dt / (r_src * cap);
            if (alpha > 1) {
                alpha = 1;
            }
            m.vc += alpha * (vset - m.vc);
        } else if (sitl_cfg.battery.sink_resistance > 0) {
            // absorption above the set voltage, saturating at
            // sink_current_max: a bench PSU downprogrammer absorbs a
            // bounded current, so heavy regen pumps the bus well above
            // the small-signal plateau (measured 24V vs 20.4V on the
            // bench supply)
            double i_sink = (m.vc - vset) / sitl_cfg.battery.sink_resistance;
            if (sitl_cfg.battery.sink_current_max > 0 &&
                i_sink > sitl_cfg.battery.sink_current_max) {
                i_sink = sitl_cfg.battery.sink_current_max;
            }
            double dv = i_sink * dt / cap;
            if (dv > m.vc - vset) {
                dv = m.vc - vset;
            }
            m.vc -= dv;
            m.e_sink += (m.vc - vset) * i_sink * dt;
        }
        m.vc -= m.ibus * dt / cap;
        if (m.vc < 0) {
            m.vc = 0;
        }
        vbus = m.vc;
    } else {
        // legacy stiff bidirectional source
        vbus = sitl_cfg.battery.voltage - m.ibus * sitl_cfg.battery.resistance;
        m.vc = vbus;
    }
    if (vbus < 0) {
        vbus = 0;
    }
    m.vbus = vbus;

    // BEMF per phase
    // phase order such that the AM32 comStep sequence 1..6 advances the
    // field by +60 degrees electrical per step (verified by detent test)
    const double thetae = m.theta * pole_pairs;
    double e[3], shape[3];
    for (int p = 0; p < 3; p++) {
        shape[p] = trap_shape(thetae + p * (TWO_PI / 3.0));
        e[p] = m.ke * m.omega * shape[p];
    }

    // gate states from the phase mode and the emulated PWM timer. On a
    // complementary switched phase both fets are off for the configured
    // dead time after each PWM edge and the body diode conducts, as on
    // hardware
    const uint32_t dead_ns = sitl_tim1_dead_time_ns();
    bool hi[3], lo[3];
    for (int p = 0; p < 3; p++) {
        const bool pwm = sitl_tim1_pwm_out(p, now_ns);
        switch (sitl_phase_mode[p]) {
        case SITL_PHASE_PWM:
            if (pwm != m.pwm_last[p]) {
                m.pwm_last[p] = pwm;
                m.dead_until[p] = now_ns + dead_ns;
            }
            if (now_ns < m.dead_until[p]) {
                hi[p] = false;
                lo[p] = false;
                break;
            }
            hi[p] = pwm;
            lo[p] = !pwm;
            break;
        case SITL_PHASE_PWM_NOCOMP:
            hi[p] = pwm;
            lo[p] = false;
            break;
        case SITL_PHASE_LOW:
            hi[p] = false;
            lo[p] = true;
            break;
        case SITL_PHASE_BRAKE_PWM:
            hi[p] = false;
            lo[p] = !pwm;
            break;
        case SITL_PHASE_FLOAT:
        default:
            hi[p] = false;
            lo[p] = false;
            break;
        }
        if (sitl_phase_mode[p] != SITL_PHASE_PWM) {
            m.pwm_last[p] = pwm;
            m.dead_until[p] = 0;
        }
    }

    // commutation current transfer: on a firmware commutation the
    // incoming phase current is moved this fraction of the way to the
    // KCL continuation of the continuing phase, and the outgoing
    // freewheel current is scaled down to match. A real motor
    // completes the transfer within the commutation on these
    // timescales; integrating it through L instead loses small-signal
    // torque gain (measured as a low chirp bandwidth against real
    // hardware). 0 keeps the plain integration
    const double ct_k = sitl_cfg.esc.commutation_transfer;
    {
        bool driven[3];
        bool changed = false;
        for (int p = 0; p < 3; p++) {
            driven[p] = sitl_phase_mode[p] != SITL_PHASE_FLOAT;
            if (driven[p] != m.driven_last[p]) {
                changed = true;
            }
        }
        if (changed && ct_k > 0) {
            int cont = -1, inc = -1, out = -1;
            int ndriven = 0, nwas = 0;
            for (int p = 0; p < 3; p++) {
                ndriven += driven[p];
                nwas += m.driven_last[p];
                if (driven[p] && m.driven_last[p]) {
                    cont = p;
                } else if (driven[p] && !m.driven_last[p]) {
                    inc = p;
                } else if (!driven[p] && m.driven_last[p]) {
                    out = p;
                }
            }
            if (ndriven == 2 && nwas == 2 && cont >= 0 && inc >= 0 && out >= 0) {
                m.i[inc] += ct_k * (-m.i[cont] - m.i[inc]);
                m.i[out] *= 1.0 - ct_k;
            }
        }
        for (int p = 0; p < 3; p++) {
            m.driven_last[p] = driven[p];
        }
    }

    /*
      terminal voltages and phase currents with persistent diode
      states (paper eq 7-10). A driven phase is tied to its rail
      through the fet. A body diode, once conducting, stays on until
      its current reaches zero - the crossing instant is found by
      linear interpolation and the step is split there, so the diode
      turns off exactly at zero current instead of oscillating around
      a threshold. Diodes arm on a driven->open transition that
      leaves current in the phase, or when a floating terminal
      voltage crosses a rail (windmill rectification)
     */
    double v[3] = { 0, 0, 0 };
    double r_eff[3];
    bool conducting[3];
    // arm freewheel diodes on freshly opened phases carrying current
    for (int p = 0; p < 3; p++) {
        if (!hi[p] && !lo[p]) {
            if (m.diode_state[p] == 0 && fabs(m.i[p]) > 1e-9) {
                m.diode_state[p] = m.i[p] < 0 ? 1 : -1;
                m.diag.diode_on_i++;
            }
        } else {
            // a driving fet shorts out the diode path
            m.diode_state[p] = 0;
        }
    }

    double int_i[3] = { 0, 0, 0 }; // per-phase integral of i over dt
    double int_i2[3] = { 0, 0, 0 }; // integral of i^2 (copper/bridge)
    double int_iabs[3] = { 0, 0, 0 }; // integral of |i| (diode drop)
    double int_ibus = 0;
    double diode_time[3] = { 0, 0, 0 }; // conduction time via diode
    // phases barred from voltage re-arming for the rest of this step:
    // a diode that armed and immediately proved inconsistent (the star
    // point moved) or that turned off at zero current must not cycle
    // within the step - sequential relaxation with a tabu set stands
    // in for the full complementarity solve
    bool no_rearm[3] = { false, false, false };
    double t_rem = dt;
    int events = 0;
    while (t_rem > 0) {
        // conducting set for this sub-interval
        int n_cond = 0;
        for (int p = 0; p < 3; p++) {
            r_eff[p] = R;
            if (hi[p]) {
                v[p] = vbus;
                r_eff[p] += rds;
                conducting[p] = true;
            } else if (lo[p]) {
                v[p] = 0;
                r_eff[p] += rds;
                conducting[p] = true;
            } else if (m.diode_state[p] > 0) {
                v[p] = vbus + vf;
                conducting[p] = true;
            } else if (m.diode_state[p] < 0) {
                v[p] = -vf;
                conducting[p] = true;
            } else {
                conducting[p] = false;
            }
            if (conducting[p]) {
                n_cond++;
            }
        }

        // star point from the conducting phases including the
        // resistive drop (equal L per phase):
        //   v_star = sum(v - r_eff*i - e) / n
        // with nothing conducting the network floats; centre it on
        // the bus as a symmetric bridge does
        double v_star = 0;
        for (int p = 0; p < 3; p++) {
            if (conducting[p]) {
                v_star += v[p] - r_eff[p] * m.i[p] - e[p];
            }
        }
        v_star = n_cond > 0 ? v_star / n_cond : 0.5 * vbus;

        // rail-crossing turn-on of open phases (paper eq 7 second
        // condition / fig 2(e)); each new clamp moves the star point,
        // and a clamp the moved star point no longer supports is
        // dropped again and barred for the rest of the step
        for (int pass = 0; pass < 4; pass++) {
            bool changed = false;
            for (int p = 0; p < 3; p++) {
                if (conducting[p] || no_rearm[p]) {
                    continue;
                }
                const double vt = e[p] + v_star;
                if (vt > vbus + vf) {
                    m.diode_state[p] = 1;
                    v[p] = vbus + vf;
                } else if (vt < -vf) {
                    m.diode_state[p] = -1;
                    v[p] = -vf;
                } else {
                    continue;
                }
                m.diag.diode_on_v++;
                conducting[p] = true;
                n_cond++;
                changed = true;
            }
            // drop zero-current voltage-armed diodes whose current
            // would build in the blocking direction under the updated
            // star point (complementarity consistency)
            v_star = 0;
            for (int p = 0; p < 3; p++) {
                if (conducting[p]) {
                    v_star += v[p] - r_eff[p] * m.i[p] - e[p];
                }
            }
            if (n_cond > 0) {
                v_star /= n_cond;
            }
            for (int p = 0; p < 3; p++) {
                if (m.diode_state[p] == 0 || fabs(m.i[p]) > 1e-9 ||
                    !conducting[p] || hi[p] || lo[p]) {
                    continue;
                }
                const double ddi = v[p] - e[p] - v_star;
                if (ddi * (double)m.diode_state[p] > 0) {
                    // high diode needs di<0 from zero, low needs di>0
                    m.diode_state[p] = 0;
                    conducting[p] = false;
                    no_rearm[p] = true;
                    n_cond--;
                    changed = true;
                }
            }
            if (!changed) {
                break;
            }
            v_star = 0;
            for (int p = 0; p < 3; p++) {
                if (conducting[p]) {
                    v_star += v[p] - r_eff[p] * m.i[p] - e[p];
                }
            }
            v_star = n_cond > 0 ? v_star / n_cond : 0.5 * vbus;
        }

        // an open phase floats at e + v_star
        for (int p = 0; p < 3; p++) {
            if (!conducting[p]) {
                v[p] = e[p] + v_star;
            }
        }
        for (int p = 0; p < 3; p++) {
            m.v_term[p] = v[p];
        }

        if (n_cond < 2) {
            // no return path: no current can flow. Every terminal
            // floats at its BEMF above the network centre - the
            // comparator still reads these, so they must be set here
            // as well as on the normal path below
            for (int p = 0; p < 3; p++) {
                if (m.diode_state[p] != 0 && fabs(m.i[p]) > 1e-9) {
                    m.diag.sign_viol++;
                }
                m.i[p] = 0;
                m.diode_state[p] = 0;
                v[p] = e[p] + v_star;
                m.v_term[p] = v[p];
            }
            break;
        }

        // current derivatives, and the earliest diode zero crossing
        double di[3] = { 0, 0, 0 };
        double t_ev = t_rem;
        int ev_p = -1;
        for (int p = 0; p < 3; p++) {
            if (!conducting[p]) {
                continue;
            }
            di[p] = (v[p] - r_eff[p] * m.i[p] - e[p] - v_star) / L_eff;
            if (m.diode_state[p] == 0 || di[p] == 0) {
                continue;
            }
            // wrong-sign current on a conducting diode: turn off now
            if (m.i[p] * (double)m.diode_state[p] > 1e-9) {
                m.diag.sign_viol++;
                t_ev = 0;
                ev_p = p;
                break;
            }
            const double t0 = -m.i[p] / di[p];
            if (t0 >= 0 && t0 < t_ev) {
                t_ev = t0;
                ev_p = p;
            }
        }

        // integrate this sub-interval (currents are linear in t)
        for (int p = 0; p < 3; p++) {
            if (!conducting[p]) {
                continue;
            }
            const double i0 = m.i[p];
            const double i1 = i0 + di[p] * t_ev;
            int_i[p] += 0.5 * (i0 + i1) * t_ev;
            int_i2[p] += (i0 * i0 + i0 * i1 + i1 * i1) / 3.0 * t_ev;
            int_iabs[p] += 0.5 * (fabs(i0) + fabs(i1)) * t_ev;
            if (m.diode_state[p] != 0) {
                diode_time[p] += t_ev;
            }
            if (v[p] >= vbus - 1e-9) {
                int_ibus += 0.5 * (i0 + i1) * t_ev;
            }
            m.i[p] = i1;
        }
        // with exactly two conducting phases KCL forces them equal
        // and opposite; project out any numerical drift
        if (n_cond == 2) {
            int a = -1, b = -1;
            for (int p = 0; p < 3; p++) {
                if (conducting[p]) {
                    if (a < 0) {
                        a = p;
                    } else {
                        b = p;
                    }
                }
            }
            const double ic = 0.5 * (m.i[a] - m.i[b]);
            m.i[a] = ic;
            m.i[b] = -ic;
        }
        if (ev_p >= 0) {
            m.i[ev_p] = 0;
            m.diode_state[ev_p] = 0;
            no_rearm[ev_p] = true;
            m.diag.diode_off++;
        }
        t_rem -= t_ev;
        if (++events > 8) {
            // event budget exhausted: integrate the remainder plainly
            m.diag.ev_overflow++;
            if (t_rem > 0) {
                for (int p = 0; p < 3; p++) {
                    if (conducting[p] && p != ev_p) {
                        m.i[p] += di[p] * t_rem;
                    }
                }
            }
            break;
        }
    }

    // electromagnetic torque from the time-averaged currents of this
    // step: tau = ke * sum(shape_p * i_p) (paper eq 2)
    double tau = 0;
    for (int p = 0; p < 3; p++) {
        tau += m.ke * shape[p] * int_i[p];
    }
    tau /= dt;
    m.tau = tau;

    // energy ledger
    m.e_em += tau * m.omega * dt;
    for (int p = 0; p < 3; p++) {
        m.e_cu += int_i2[p] * R;
        const double f_diode = diode_time[p] / dt;
        m.e_br += int_iabs[p] * vf * f_diode
                  + int_i2[p] * rds * (1.0 - f_diode);
    }

    // load torques and motion (paper eq 3, plus k*omega^2 propeller
    // load and static friction). A stuck rotor (prop blocked by an
    // obstruction, e.g. a tree branch) adds Coulomb holding torque
    // through the same dead band; at 1.0 the rotor is held rigidly
    const double stuck = sitl_cfg.stuck;
    const double w = m.omega;
    double tau_load = sitl_cfg.motor.damping * w + sitl_cfg.motor.load_k_omega2 * w * fabs(w);
    double tau_net = tau - tau_load;
    const double sf = sitl_cfg.motor.static_friction
        + stuck * sitl_cfg.motor.stuck_torque;
    if (fabs(w) < 0.5) {
        // static friction dead band
        if (fabs(tau_net) <= sf) {
            tau_net = 0;
            m.omega = 0;
        } else {
            tau_net -= (tau_net > 0 ? sf : -sf);
        }
    } else {
        tau_net -= (w > 0 ? sf : -sf);
    }
    m.e_drag += (tau - tau_net) * m.omega * dt;
    if (stuck >= 1.0) {
        // completely stuck: the obstruction reacts all torque and
        // absorbs the remaining kinetic energy
        m.e_drag += 0.5 * sitl_cfg.motor.inertia * m.omega * m.omega;
        m.omega = 0;
        tau_net = 0;
    }
    m.omega += tau_net / sitl_cfg.motor.inertia * dt;
    // Coulomb friction may stop the rotor within a step but must not
    // reverse it: a large obstruction holding torque would otherwise
    // jump the dead band in one step and flip the spin direction every
    // step. Stopped at zero, the next step's dead band decides whether
    // it stays held
    if (w != 0 && m.omega * w < 0) {
        m.omega = 0;
    }
    m.theta += m.omega * dt;
    if (m.theta > TWO_PI || m.theta < -TWO_PI) {
        m.theta = fmod(m.theta, TWO_PI);
    }

    // battery current: everything sourced from the positive rail,
    // averaged over the step from the sub-interval integrals
    m.ibus = int_ibus / dt;
    m.e_bus += vbus * int_ibus;

    // comparator analog front end: on a real board each phase feeds
    // the comparator through a divider with an RC, and the virtual
    // neutral is a resistor star with its own RC. Model each node as a
    // first-order filter so commutation-step chatter is suppressed at
    // the source as on hardware. Zero time constants keep the legacy
    // raw comparator
    const double tau_p_ns = (double)sitl_cfg.sim.comparator_phase_rc_ns;
    const double tau_n_ns = (double)sitl_cfg.sim.comparator_neutral_rc_ns;
    const double v_star_raw = (v[0] + v[1] + v[2]) / 3.0;
    if (tau_p_ns > 0) {
        double a = (double)dt_ns / tau_p_ns;
        if (a > 1) {
            a = 1;
        }
        for (int p = 0; p < 3; p++) {
            m.v_cmp[p] += a * (v[p] - m.v_cmp[p]);
        }
    } else {
        for (int p = 0; p < 3; p++) {
            m.v_cmp[p] = v[p];
        }
    }
    if (tau_n_ns > 0) {
        double a = (double)dt_ns / tau_n_ns;
        if (a > 1) {
            a = 1;
        }
        m.vn_cmp += a * (v_star_raw - m.vn_cmp);
    } else {
        m.vn_cmp = v_star_raw;
    }

    // comparator: filtered virtual neutral against the filtered
    // floating phase terminal
    const double v_neutral = m.vn_cmp;
    const double v_float = m.v_cmp[sitl_comp_phase];
    double diff_mv = (v_neutral - v_float) * 1000.0;
    if (sitl_cfg.sim.comparator_noise_mv > 0) {
        const double r = (double)rand_r(&m.rand_seed) / RAND_MAX - 0.5;
        const double white = r * 2.0 * sitl_cfg.sim.comparator_noise_mv;
        if (tau_p_ns > 0) {
            // band-limit the noise with the front-end time constant:
            // wideband white noise on an RC-slowed signal would toggle
            // the comparator at every crossing, which no real board
            // shows
            double a = (double)dt_ns / tau_p_ns;
            if (a > 1) {
                a = 1;
            }
            m.noise_cmp += a * (white - m.noise_cmp);
            diff_mv += m.noise_cmp;
        } else {
            diff_mv += white;
        }
    }
    const double hyst = sitl_cfg.sim.comparator_hysteresis_mv * 0.5;
    uint8_t raw = sitl_comp_out;
    if (raw) {
        raw = diff_mv > -hyst;
    } else {
        raw = diff_mv > hyst;
    }
    /*
      inertial propagation: a real comparator's output commits only
      after its input has crossed the threshold for its response time;
      shorter excursions are absorbed. Unlike a post-transition
      refractory hold, the committed edge is a CONSTANT delay behind
      the true crossing, so it filters noise pulses without adding
      timing jitter to clean crossings. comparator_min_toggle_ns is
      the response time; 0 commits immediately
     */
    static uint64_t pend_since_ns;
    uint8_t out = sitl_comp_out;
    if (raw == sitl_comp_out) {
        pend_since_ns = 0;
    } else {
        if (pend_since_ns == 0) {
            pend_since_ns = now_ns;
        }
        if (now_ns - pend_since_ns >= sitl_cfg.sim.comparator_min_toggle_ns) {
            out = raw;
            pend_since_ns = 0;
        }
    }
    if (out != sitl_comp_out) {
        sitl_comp_out = out;
        m.diag.comp_edges++;
        const uint32_t line = current_EXTI_LINE;
        const bool rising_edge = out != 0;
        if ((rising_edge && (sitl_exti.RTSR & line)) || (!rising_edge && (sitl_exti.FTSR & line))) {
            sitl_exti.PR |= line;
            const bool unmasked = (sitl_exti.IMR & line) != 0;
            if (unmasked) {
                sitl_irq_pend(SITL_IRQ_COMP);
            }
            motor_log_event(MEV_EDGE, out, unmasked, 0);
        }
    }

    check_desync_dump();

    // optional raw physics log: esc_measure-compatible JSONL at 1kHz
    // of sim time, for comparing the true physics against what arrives
    // over DroneCAN telemetry. Appended across boots (re-exec restarts
    // sim time at zero, so consumers split segments where t decreases).
    // Lines are formatted here but written by a separate logger thread,
    // so filesystem write stalls can never perturb physics timing; on
    // ring overflow lines are dropped and counted rather than blocking
    if (sitl_cfg.physics_log) {
        static uint64_t next_log_ns;
        if (now_ns >= next_log_ns) {
            next_log_ns = now_ns + 1000000;
            extern volatile uint16_t duty_cycle;
            char line[PLOG_LINE_MAX];
            snprintf(line, sizeof(line),
                "{\"t\": %.6f, \"type\": \"status\", \"rpm\": %.1f, \"volt\": %.3f,"
                " \"curr\": %.3f, \"duty\": %u,"
                " \"e_em\": %.4f, \"e_cu\": %.4f, \"e_br\": %.4f,"
                " \"e_bus\": %.4f, \"e_sink\": %.4f, \"e_drag\": %.4f,"
                " \"e_mag\": %.5f, \"e_cap\": %.4f}\n",
                now_ns * 1e-9, m.omega * 60.0 / TWO_PI, m.vbus, m.ibus,
                (unsigned)duty_cycle,
                m.e_em, m.e_cu, m.e_br, m.e_bus, m.e_sink, m.e_drag,
                0.5 * L_eff * (m.i[0]*m.i[0] + m.i[1]*m.i[1] + m.i[2]*m.i[2]),
                sitl_cfg.battery.capacitance > 0 ?
                    0.5 * sitl_cfg.battery.capacitance * m.vc * m.vc : 0.0);
            plog_push(line);
        }
    }

    // sensor averaging, snapshot every 64 steps
    m.acc_v += m.vbus;
    m.acc_i += m.ibus > 0 ? m.ibus : 0;
    m.acc_n++;
    if (m.acc_n >= 64) {
        sitl_sensors_t s;
        s.bus_voltage = (float)(m.acc_v / m.acc_n);
        s.bus_current = (float)(m.acc_i / m.acc_n);
        s.temperature_c = sitl_cfg.esc.temperature_c;
        s.rpm = (float)(m.omega * 60.0 / TWO_PI);
        sitl_sensors_write(&s);
        m.acc_v = m.acc_i = 0;
        m.acc_n = 0;
    }
}

/*
  commutation debug ring: comStep calls in here; on a firmware desync the
  recent history is dumped so the failure can be analysed
 */
#define COMM_RING 96
static struct {
    uint64_t t_ns;
    float thetae_deg; // electrical angle
    float rpm;
    uint32_t ci;
    uint32_t a, b, c;
    uint8_t kind;
} comm_ring[COMM_RING];
static unsigned comm_ring_pos;

void motor_log_event(int kind, uint32_t a, uint32_t b, uint32_t c)
{
    extern volatile uint32_t commutation_interval;
    const int pole_pairs = sitl_cfg.motor.poles / 2;
    double deg = fmod(m.theta * pole_pairs * 180.0 / M_PI, 360.0);
    if (deg < 0) {
        deg += 360;
    }
    const unsigned idx = __atomic_fetch_add(&comm_ring_pos, 1, __ATOMIC_SEQ_CST) % COMM_RING;
    comm_ring[idx].t_ns = sitl_time_ns();
    comm_ring[idx].thetae_deg = (float)deg;
    comm_ring[idx].rpm = (float)(m.omega * 60.0 / TWO_PI);
    comm_ring[idx].ci = commutation_interval;
    comm_ring[idx].a = a;
    comm_ring[idx].b = b;
    comm_ring[idx].c = c;
    comm_ring[idx].kind = (uint8_t)kind;

}

void motor_log_mainloop(void)
{
    extern volatile uint32_t average_interval;
    extern uint32_t last_average_interval;
    static uint64_t last_ns;
    const uint64_t now = sitl_time_ns();
    const uint32_t gap_us = (uint32_t)((now - last_ns) / 1000ULL);
    last_ns = now;
    // only log iterations after a gap to avoid flooding the ring
    if (gap_us >= 50) {
        motor_log_event(MEV_MAINLOOP, average_interval, last_average_interval, gap_us);
    }
}

void motor_log_commutation(int step)
{
    extern volatile uint16_t duty_cycle;
    extern uint16_t duty_cycle_maximum;
    motor_log_event(MEV_COMMUTATE, (uint32_t)step, duty_cycle, duty_cycle_maximum);
}

static void check_desync_dump(void)
{
    extern uint32_t desync_happened;
    static uint32_t last_desync;
    static int dumps;
    if (desync_happened == last_desync) {
        return;
    }
    last_desync = desync_happened;
    if (dumps >= 3) {
        return;
    }
    dumps++;
    extern volatile uint32_t average_interval;
    extern uint32_t last_average_interval;
    extern volatile uint32_t zero_crosses;
    extern int e_com_time;
    fprintf(stderr,
        "SITL: desync %u at t=%.4fs avg=%u last_avg=%u zc=%u e_com_time=%d, recent events:\n",
        (unsigned)desync_happened, sitl_time_ns() * 1e-9,
        (unsigned)average_interval, (unsigned)last_average_interval,
        (unsigned)zero_crosses, e_com_time);
    dump_ring();
}

static void dump_ring(void)
{
    static const char* kinds[] = { "COMMUTATE", "EDGE", "BLANKED", "COMP_RUN", "ZC_ACCEPT", "MAINLOOP" };
    for (unsigned k = 0; k < COMM_RING; k++) {
        const unsigned idx = (comm_ring_pos + k) % COMM_RING;
        if (comm_ring[idx].t_ns == 0) {
            continue;
        }
        fprintf(stderr, "  t=%.6f %-9s thetae=%5.1f rpm=%6.0f ci=%u a=%u b=%u c=%u\n",
            comm_ring[idx].t_ns * 1e-9, kinds[comm_ring[idx].kind],
            (double)comm_ring[idx].thetae_deg, (double)comm_ring[idx].rpm,
            (unsigned)comm_ring[idx].ci,
            (unsigned)comm_ring[idx].a, (unsigned)comm_ring[idx].b,
            (unsigned)comm_ring[idx].c);
    }
}

void motor_get_state(double* theta, double* omega, double i[3])
{
    *theta = m.theta;
    *omega = m.omega;
    for (int p = 0; p < 3; p++) {
        i[p] = m.i[p];
    }
}

void motor_print_state(uint64_t now_ns, float time_ratio)
{
    // solver diagnostics: rates since the previous print
    {
        static typeof(m.diag) last;
        fprintf(stderr,
            "SITL diag: comp_edges=%u diode_on_i=%u on_v=%u off=%u sign_viol=%u ev_ovf=%u\n",
            m.diag.comp_edges - last.comp_edges,
            m.diag.diode_on_i - last.diode_on_i,
            m.diag.diode_on_v - last.diode_on_v,
            m.diag.diode_off - last.diode_off,
            m.diag.sign_viol - last.sign_viol,
            m.diag.ev_overflow - last.ev_overflow);
        last = m.diag;
    }
    // firmware state, for debug output only
    extern uint16_t input;
    extern volatile char armed;
    extern char step;
    extern volatile uint32_t zero_crosses;
    extern volatile uint32_t commutation_interval;
    extern volatile uint16_t duty_cycle;
    extern uint8_t bemf_timeout_happened;
    extern uint8_t running;
    extern char old_routine;
    extern volatile uint16_t newinput;
    extern uint16_t adjusted_input;
    extern EEprom_t eepromBuffer;
    static bool printed_settings;
    if (!printed_settings) {
        printed_settings = true;
        fprintf(stderr,
            "SITL settings: bidir=%u dir_rev=%u comp_pwm=%u poles=%u input_type=%u sine=%u brake_on_stop=%u\n",
            eepromBuffer.bi_direction, eepromBuffer.dir_reversed,
            eepromBuffer.comp_pwm, eepromBuffer.motor_poles,
            eepromBuffer.input_type, eepromBuffer.use_sine_start,
            eepromBuffer.brake_on_stop);
    }

    extern void sitl_can_stats(uint32_t stats[4]);
    uint32_t cs[4];
    sitl_can_stats(cs);

    fprintf(stderr,
        "SITL t=%.1fs x%.2f rpm=%.0f Vbus=%.2f Ibus=%.2f modes=%d%d%d in=%u newin=%u adj=%u armed=%d duty=%u step=%d zc=%u ci=%u run=%d old=%d bemf_to=%u cmd=%u\n",
        now_ns * 1e-9, (double)time_ratio, m.omega * 60.0 / TWO_PI,
        m.vbus, m.ibus,
        sitl_phase_mode[0], sitl_phase_mode[1], sitl_phase_mode[2],
        input, newinput, adjusted_input, armed, duty_cycle, step,
        (unsigned)zero_crosses, (unsigned)commutation_interval,
        running, old_routine, bemf_timeout_happened,
        (unsigned)cs[1]);
}
