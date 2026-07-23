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
    bool nosleep; // busy wait instead of sleeping, for timing accuracy
    bool realtime; // SCHED_FIFO for both threads
} sitl_config_t;

extern sitl_config_t sitl_cfg;

// parse CLI and optional JSON config, exits on error
void sitl_config_init(int argc, char** argv);

// runtime reload of the motor/battery/esc sections from a JSON file
// (sim section ignored). Returns false on error, never exits
bool sitl_config_reload(const char* path);
