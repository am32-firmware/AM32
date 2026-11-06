/*
  sitl_config.h - JSON file + command line configuration for AM32 SITL
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    struct {
        float kv; // rpm per volt
        int poles; // magnetic poles (not pole pairs)
        float resistance; // phase resistance, ohm
        float inductance; // phase self inductance, henry
        float mutual_inductance; // henry (negative or zero)
        float inertia; // rotor+prop inertia, kg m^2
        float damping; // Nm/(rad/s)
        float static_friction; // Nm
        float load_k_omega2; // propeller load: Nm/(rad/s)^2
        float stuck_torque; // holding torque of a full obstruction, Nm
    } motor;
    struct {
        float voltage; // open circuit volts
        float resistance; // internal resistance, ohm
        // bus/supply dynamics, for supplies that cannot absorb regen
        // (bench PSUs): 0 capacitance keeps the legacy stiff
        // bidirectional source
        float capacitance; // bus capacitance seen by the ESC, farad
        float sink_resistance; // ohm, reverse-current absorption above
                               // the set voltage (0 = cannot sink)
        float sink_current_max; // A, absorption saturates here (a PSU
                                // downprogrammer current limit;
                                // 0 = unlimited)
    } battery;
    struct {
        float rds_on; // fet on resistance, ohm
        float diode_vf; // body diode forward voltage
        float temperature_c; // reported temperature
        // fraction of the conduction current transferred to the
        // incoming phase instantly at commutation (0..1). Real motors
        // complete the transfer within the commutation; 0 integrates
        // it through L (legacy)
        float commutation_transfer;
    } esc;
    struct {
        uint32_t physics_dt_ns; // integration step
        uint32_t loop_time_ns; // firmware main loop pacing sleep
        uint32_t isr_read_ns; // cost of a register read in interrupt context
        float comparator_noise_mv;
        float comparator_hysteresis_mv;
        // analog front end: RC time constants of the per-phase BEMF
        // divider node and the virtual-neutral node feeding the
        // comparator. A real board's RC filters commutation chatter at
        // the source; 0 disables (legacy raw comparator)
        uint32_t comparator_phase_rc_ns;
        uint32_t comparator_neutral_rc_ns;
        // comparator response time: the input must stay across the
        // threshold this long before the output commits (inertial
        // propagation - constant delay, absorbs shorter pulses)
        uint32_t comparator_min_toggle_ns;
        // pulse injected into the comparator difference at every switching
        // edge of a driven leg, decaying with this time constant (0 = off)
        float comparator_pwm_glitch_mv;
        uint32_t comparator_pwm_glitch_ns;
        // damped ringing of the comparator input after every driven-leg PWM
        // edge: bursts of toggles around each edge near the crossing, a single
        // short dip per edge away from it (bench capture, TBS 12S L431 at 15 %)
        float comparator_ring_mv;
        uint32_t comparator_ring_hz;
        uint32_t comparator_ring_tau_ns;
        // comparator input offset: pins the output at rest, shifts the crossing slightly
        float comparator_offset_mv;
        // bench-measured unipolar excursions (TBS 12S L431, unloaded): the
        // comparator reads the floating phase below the neutral in a ramp
        // through the high-side on-time and in a lobe some microseconds
        // after the turn-off; the lobe shrinks with speed as (ref/rpm)^exp
        float comparator_on_ramp_mv_per_us;
        uint32_t comparator_on_ramp_delay_ns;
        float comparator_on_ramp_max_mv;
        float comparator_off_lobe_mv;
        uint32_t comparator_off_lobe_delay_ns;
        uint32_t comparator_off_lobe_width_ns;
        uint32_t comparator_off_lobe_ref_rpm;
        float comparator_off_lobe_rpm_exp;
        float comparator_off_notch_mv; // brief opposite swing just ahead of the lobe
        // board dead time when nonzero (the SITL target's BDTR otherwise)
        uint32_t dead_time_ns;
        // MCU profile knobs: interrupt entry latency, the F051/G071-style
        // comparator handler that keeps an in-window edge pending until the
        // blanking gate opens, and a 16-bit interval timer
        uint32_t irq_latency_ns;
        bool comparator_hold_pending;
        uint32_t interval_timer_bits;
        // demag guard policy limits of the profiled MCU
        uint32_t demag_max_advance_level;
        uint32_t demag_min_wait_ticks;
        uint32_t demag_deadline_margin_ticks;
        // mainline progress lease: simulated time may not run further
        // than this ahead of the last firmware-thread interception
        // while the mainline is runnable. Bounds sim-visible mainline
        // starvation under host load. 0 disables the gate
        uint32_t fw_lag_max_ns;
        bool watchdog_enabled;
    } sim;

    // runtime options
    float speedup; // 0 = free run
    // stuck rotor fraction 0..1 (prop blocked by an obstruction, e.g. a
    // tree branch): scales motor.stuck_torque, 1.0 locks the rotor.
    // Set at runtime over the state port
    float stuck;
    int input_port; // UDP port for PWM/DShot input, 0 disables
    int state_port; // UDP port for state streaming/model control, 0 disables
    bool bind_any; // bind input/state ports on all interfaces, not loopback
    const char* eeprom_path;
    const char* can_uri;
    const char* uid; // optional fixed unique ID string
    const char* bootloader_path; // bootloader elf: resets exec it (NULL = off)
    const char* physics_log; // JSONL raw physics log path (NULL = off)
    int node_id; // -1 = leave to eeprom/DNA
    int input_type; // eeprom INPUT_SIGNAL_TYPE override, -1 = leave
    bool verbose;
    const char* log_file; // redirect diagnostics to a file instead of stderr
    bool wait_for_input; // hold startup until the PWM/DShot sender is ready
    bool exit_on_reset; // end the run instead of re-exec (for host debuggers)
    bool nosleep; // busy wait instead of sleeping, for timing accuracy
    bool realtime; // SCHED_FIFO for both threads
} sitl_config_t;

extern sitl_config_t sitl_cfg;

// parse CLI and optional JSON config, exits on error
void sitl_config_init(int argc, char** argv);

// runtime reload of the motor/battery/esc sections from a JSON file
// (sim section ignored). Returns false on error, never exits
bool sitl_config_reload(const char* path);
