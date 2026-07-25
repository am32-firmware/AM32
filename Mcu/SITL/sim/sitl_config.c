/*
  sitl_config.c - JSON config file and command line handling for AM32 SITL
 */

#include "sitl_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <math.h>

#include "jsmn.h"

sitl_config_t sitl_cfg = {
    /*
      defaults are the calibrated VimDrones Nano + T-Motor AIR 2216 II
      920Kv with a 10x4.5 prop on 3S (an Edu450 drivetrain), fitted
      against the bench captures in data/VIMDRONES_NANO_2216 - see
      models/vimdrones_nano_2216.json. A prop-loaded model is the right
      default: it matches how ESCs are actually flown, and the real
      combination never desyncs, which the model reproduces
     */
    .motor = {
        .kv = 944,
        .poles = 14,
        .resistance = 0.09f,
        .inductance = 1.5e-5f,
        .mutual_inductance = 0.0f,
        .inertia = 3.35e-5f,
        .damping = 1.0e-6f,
        .static_friction = 0.002f,
        .load_k_omega2 = 1.9e-7f,
        .stuck_torque = 1.0f,
    },
    .battery = {
        .voltage = 12.4f,
        .resistance = 0.09f,
        .capacitance = 0.002f,
        .sink_resistance = 4.0f,
        .sink_current_max = 0.8f,
    },
    .esc = {
        .rds_on = 0.006f,
        .diode_vf = 0.7f,
        .temperature_c = 38.0f,
        .commutation_transfer = 1.0f,
    },
    .sim = {
        .physics_dt_ns = 500,
        .loop_time_ns = 2000,
        .isr_read_ns = 100,
        .comparator_noise_mv = 5.0f,
        // the real boards configure no comparator hysteresis; the
        // divider RC band-limits the input instead
        .comparator_hysteresis_mv = 0.0f,
        .comparator_phase_rc_ns = 800,
        .comparator_neutral_rc_ns = 800,
        .comparator_min_toggle_ns = 2000,
        .fw_lag_max_ns = 20000,
        .watchdog_enabled = true,
    },
    .speedup = 1.0f,
    .stuck = 0.0f,
    .input_port = 57733,
    .state_port = 57734,
    .bind_any = false,
    .eeprom_path = "am32_eeprom.bin",
    .can_uri = "mcast:0",
    .uid = NULL,
    .bootloader_path = NULL,
    .physics_log = NULL,
    .node_id = -1,
    .input_type = -1,
    .verbose = false,
    .nosleep = false,
    .realtime = false,
};

struct cfg_entry {
    const char* section;
    const char* key;
    enum { CFG_FLOAT,
        CFG_INT,
        CFG_U32,
        CFG_BOOL } type;
    void* ptr;
};

static const struct cfg_entry cfg_table[] = {
    { "motor", "kv", CFG_FLOAT, &sitl_cfg.motor.kv },
    { "motor", "poles", CFG_INT, &sitl_cfg.motor.poles },
    { "motor", "resistance", CFG_FLOAT, &sitl_cfg.motor.resistance },
    { "motor", "inductance", CFG_FLOAT, &sitl_cfg.motor.inductance },
    { "motor", "mutual_inductance", CFG_FLOAT, &sitl_cfg.motor.mutual_inductance },
    { "motor", "inertia", CFG_FLOAT, &sitl_cfg.motor.inertia },
    { "motor", "damping", CFG_FLOAT, &sitl_cfg.motor.damping },
    { "motor", "static_friction", CFG_FLOAT, &sitl_cfg.motor.static_friction },
    { "motor", "load_k_omega2", CFG_FLOAT, &sitl_cfg.motor.load_k_omega2 },
    { "motor", "stuck_torque", CFG_FLOAT, &sitl_cfg.motor.stuck_torque },
    { "battery", "voltage", CFG_FLOAT, &sitl_cfg.battery.voltage },
    { "battery", "resistance", CFG_FLOAT, &sitl_cfg.battery.resistance },
    { "battery", "capacitance", CFG_FLOAT, &sitl_cfg.battery.capacitance },
    { "battery", "sink_resistance", CFG_FLOAT, &sitl_cfg.battery.sink_resistance },
    { "battery", "sink_current_max", CFG_FLOAT, &sitl_cfg.battery.sink_current_max },
    { "esc", "rds_on", CFG_FLOAT, &sitl_cfg.esc.rds_on },
    { "esc", "diode_vf", CFG_FLOAT, &sitl_cfg.esc.diode_vf },
    { "esc", "temperature_c", CFG_FLOAT, &sitl_cfg.esc.temperature_c },
    { "esc", "commutation_transfer", CFG_FLOAT, &sitl_cfg.esc.commutation_transfer },
    { "sim", "physics_dt_ns", CFG_U32, &sitl_cfg.sim.physics_dt_ns },
    { "sim", "loop_time_ns", CFG_U32, &sitl_cfg.sim.loop_time_ns },
    { "sim", "isr_read_ns", CFG_U32, &sitl_cfg.sim.isr_read_ns },
    { "sim", "comparator_noise_mv", CFG_FLOAT, &sitl_cfg.sim.comparator_noise_mv },
    { "sim", "comparator_hysteresis_mv", CFG_FLOAT, &sitl_cfg.sim.comparator_hysteresis_mv },
    { "sim", "comparator_phase_rc_ns", CFG_U32, &sitl_cfg.sim.comparator_phase_rc_ns },
    { "sim", "comparator_neutral_rc_ns", CFG_U32, &sitl_cfg.sim.comparator_neutral_rc_ns },
    { "sim", "comparator_min_toggle_ns", CFG_U32, &sitl_cfg.sim.comparator_min_toggle_ns },
    { "sim", "fw_lag_max_ns", CFG_U32, &sitl_cfg.sim.fw_lag_max_ns },
    { "sim", "watchdog_enabled", CFG_BOOL, &sitl_cfg.sim.watchdog_enabled },
};

static bool set_value(const char* section, const char* js, const jsmntok_t* key, const jsmntok_t* val, const char* path)
{
    char keystr[64], valstr[64];
    snprintf(keystr, sizeof(keystr), "%.*s", key->end - key->start, js + key->start);
    snprintf(valstr, sizeof(valstr), "%.*s", val->end - val->start, js + val->start);
    for (unsigned i = 0; i < sizeof(cfg_table) / sizeof(cfg_table[0]); i++) {
        const struct cfg_entry* e = &cfg_table[i];
        if (strcmp(e->section, section) != 0 || strcmp(e->key, keystr) != 0) {
            continue;
        }
        switch (e->type) {
        case CFG_FLOAT:
            *(float*)e->ptr = strtof(valstr, NULL);
            break;
        case CFG_INT:
            *(int*)e->ptr = atoi(valstr);
            break;
        case CFG_U32:
            *(uint32_t*)e->ptr = strtoul(valstr, NULL, 0);
            break;
        case CFG_BOOL:
            *(bool*)e->ptr = (strcmp(valstr, "true") == 0 || strcmp(valstr, "1") == 0);
            break;
        }
        return true;
    }
    fprintf(stderr, "SITL: %s: unknown config key %s.%s\n", path, section, keystr);
    return false;
}

// count the tokens making up one JSON value, for skipping
static int value_size(const jsmntok_t* t)
{
    int count = 1;
    if (t->type == JSMN_OBJECT) {
        const jsmntok_t* p = t + 1;
        for (int i = 0; i < t->size; i++) {
            count++; // key
            const int vs = value_size(p + 1);
            count += vs;
            p += 1 + vs;
        }
    } else if (t->type == JSMN_ARRAY) {
        const jsmntok_t* p = t + 1;
        for (int i = 0; i < t->size; i++) {
            const int vs = value_size(p);
            count += vs;
            p += vs;
        }
    }
    return count;
}

/*
  load a JSON config file. When runtime is true only the motor, battery
  and esc sections are applied (the sim section cannot change while
  running) and errors are reported instead of exiting
 */
static bool load_json_ex(const char* path, bool runtime)
{
    FILE* f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "SITL: failed to open config %s\n", path);
        return false;
    }
    static char js[16384];
    const size_t n = fread(js, 1, sizeof(js) - 1, f);
    fclose(f);
    js[n] = 0;

    jsmn_parser parser;
    static jsmntok_t tokens[512];
    jsmn_init(&parser);
    const int ntok = jsmn_parse(&parser, js, n, tokens, 512);
    if (ntok < 1 || tokens[0].type != JSMN_OBJECT) {
        fprintf(stderr, "SITL: invalid JSON in %s (err %d)\n", path, ntok);
        return false;
    }

    bool ok = true;
    const jsmntok_t* t = &tokens[1];
    for (int i = 0; i < tokens[0].size; i++) {
        const jsmntok_t* section = t;
        const jsmntok_t* sec_obj = t + 1;
        char secstr[64];
        snprintf(secstr, sizeof(secstr), "%.*s", section->end - section->start, js + section->start);
        if (sec_obj->type != JSMN_OBJECT) {
            fprintf(stderr, "SITL: %s: section %s is not an object\n", path, secstr);
            return false;
        }
        const bool skip = runtime && strcmp(secstr, "motor") != 0 &&
            strcmp(secstr, "battery") != 0 && strcmp(secstr, "esc") != 0;
        const jsmntok_t* kt = sec_obj + 1;
        for (int k = 0; k < sec_obj->size; k++) {
            const jsmntok_t* val = kt + 1;
            if (!skip && !set_value(secstr, js, kt, val, path)) {
                ok = false;
            }
            kt += 1 + value_size(val);
        }
        t += 1 + value_size(sec_obj);
    }
    return ok;
}

static void load_json(const char* path)
{
    if (!load_json_ex(path, false)) {
        exit(1);
    }
}

/*
  reject config values that would break the physics: motor.c divides by
  kv, inertia and L - M, and any NaN poisons the whole state. Clamp with
  a warning rather than exit so a bad runtime LOAD_MODEL cannot wedge a
  running simulation
 */
static void clampf(float* v, float lo, float hi, const char* name)
{
    float nv = *v;
    if (!isfinite(nv) || nv < lo) {
        nv = lo;
    } else if (nv > hi) {
        nv = hi;
    }
    if (nv != *v || !isfinite(*v)) {
        fprintf(stderr, "SITL: config %s=%g clamped to %g\n", name, (double)*v, (double)nv);
        *v = nv;
    }
}

static void config_sanitise(void)
{
    clampf(&sitl_cfg.motor.kv, 1.0f, 1e6f, "motor.kv");
    if (sitl_cfg.motor.poles < 2 || (sitl_cfg.motor.poles & 1)) {
        fprintf(stderr, "SITL: config motor.poles=%d invalid, using 2\n", sitl_cfg.motor.poles);
        sitl_cfg.motor.poles = sitl_cfg.motor.poles > 2 ? sitl_cfg.motor.poles & ~1 : 2;
    }
    clampf(&sitl_cfg.motor.resistance, 1e-4f, 100.0f, "motor.resistance");
    clampf(&sitl_cfg.motor.inductance, 1e-8f, 1.0f, "motor.inductance");
    // keep L - M positive
    clampf(&sitl_cfg.motor.mutual_inductance, -1.0f,
        sitl_cfg.motor.inductance * 0.9f, "motor.mutual_inductance");
    clampf(&sitl_cfg.motor.inertia, 1e-9f, 100.0f, "motor.inertia");
    clampf(&sitl_cfg.motor.damping, 0.0f, 1.0f, "motor.damping");
    clampf(&sitl_cfg.motor.static_friction, 0.0f, 10.0f, "motor.static_friction");
    clampf(&sitl_cfg.motor.load_k_omega2, 0.0f, 1.0f, "motor.load_k_omega2");
    clampf(&sitl_cfg.motor.stuck_torque, 0.0f, 100.0f, "motor.stuck_torque");
    clampf(&sitl_cfg.stuck, 0.0f, 1.0f, "stuck");
    clampf(&sitl_cfg.battery.voltage, 1.0f, 200.0f, "battery.voltage");
    clampf(&sitl_cfg.battery.resistance, 0.0f, 10.0f, "battery.resistance");
    clampf(&sitl_cfg.battery.capacitance, 0.0f, 1.0f, "battery.capacitance");
    clampf(&sitl_cfg.battery.sink_resistance, 0.0f, 1000.0f, "battery.sink_resistance");
    clampf(&sitl_cfg.battery.sink_current_max, 0.0f, 100.0f, "battery.sink_current_max");
    if (sitl_cfg.sim.fw_lag_max_ns != 0 &&
        sitl_cfg.sim.fw_lag_max_ns < 4 * sitl_cfg.sim.loop_time_ns) {
        fprintf(stderr, "SITL: sim.fw_lag_max_ns raised to %u (4x loop_time_ns)\n",
            4 * sitl_cfg.sim.loop_time_ns);
        sitl_cfg.sim.fw_lag_max_ns = 4 * sitl_cfg.sim.loop_time_ns;
    }
    clampf(&sitl_cfg.esc.rds_on, 0.0f, 1.0f, "esc.rds_on");
    clampf(&sitl_cfg.esc.diode_vf, 0.0f, 5.0f, "esc.diode_vf");
    clampf(&sitl_cfg.esc.commutation_transfer, 0.0f, 1.0f, "esc.commutation_transfer");
}

bool sitl_config_reload(const char* path)
{
    const bool ok = load_json_ex(path, true);
    config_sanitise();
    return ok;
}

static void usage(const char* prog)
{
    printf("Usage: %s [options]\n"
           "  --config FILE    JSON config file for motor/battery/esc/sim\n"
           "  --eeprom FILE    eeprom backing file (default am32_eeprom.bin)\n"
           "  --can-uri URI    CAN interface (default mcast:0)\n"
           "  --input-port N   UDP port for PWM/DShot input, 0 to disable\n"
           "                   (default 57733)\n"
           "  --state-port N   UDP port for simulation state streaming and\n"
           "                   runtime model control, 0 to disable\n"
           "                   (default 57734)\n"
           "  --speedup X      simulation speed, 0 for free running (default 1.0)\n"
           "  --bind-any       bind the input/state UDP ports on all interfaces\n"
           "                   instead of loopback only, for a GUI on another\n"
           "                   host\n"
           "  --node-id N      force DroneCAN node ID\n"
           "  --input-type N   force eeprom INPUT_SIGNAL_TYPE (0=auto 1=dshot\n"
           "                   2=servo 5=dronecan)\n"
           "  --uid STR        string used to derive the 16 byte unique ID\n"
           "  --physics-log F  write raw physics state (rpm/volt/curr/duty)\n"
           "                   as JSONL at 1kHz of sim time, appended\n"
           "                   across boots\n"
           "  --bootloader ELF chain with the bootloader SITL (am32-bootloader\n"
           "                   repo): boot starts in the bootloader and resets\n"
           "                   return to it, as on hardware. Default off\n"
           "  --verbose        1Hz state output on stderr\n"
           "  --nosleep        busy wait instead of sleeping (uses two full\n"
           "                   CPU cores but gives the most accurate timing)\n"
           "  --realtime       SCHED_FIFO scheduling for both threads (needs\n"
           "                   root or an rtprio rlimit; with --nosleep also\n"
           "                   set kernel.sched_rt_runtime_us=-1)\n",
        prog);
}

void sitl_config_init(int argc, char** argv)
{
    static const struct option opts[] = {
        { "config", required_argument, NULL, 'c' },
        { "eeprom", required_argument, NULL, 'e' },
        { "can-uri", required_argument, NULL, 'u' },
        { "input-port", required_argument, NULL, 'p' },
        { "state-port", required_argument, NULL, 'P' },
        { "speedup", required_argument, NULL, 's' },
        { "bind-any", no_argument, NULL, 'A' },
        { "node-id", required_argument, NULL, 'n' },
        { "input-type", required_argument, NULL, 'I' },
        { "uid", required_argument, NULL, 'U' },
        { "bootloader", required_argument, NULL, 'B' },
        { "physics-log", required_argument, NULL, 'L' },
        { "verbose", no_argument, NULL, 'v' },
        { "nosleep", no_argument, NULL, 'N' },
        { "realtime", no_argument, NULL, 'R' },
        { "help", no_argument, NULL, 'h' },
        { NULL, 0, NULL, 0 },
    };
    int c;
    while ((c = getopt_long(argc, argv, "c:e:u:p:P:s:An:I:U:B:L:vNRh", opts, NULL)) != -1) {
        switch (c) {
        case 'c':
            load_json(optarg);
            break;
        case 'e':
            sitl_cfg.eeprom_path = optarg;
            break;
        case 'u':
            sitl_cfg.can_uri = optarg;
            break;
        case 'p':
            sitl_cfg.input_port = atoi(optarg);
            break;
        case 'P':
            sitl_cfg.state_port = atoi(optarg);
            break;
        case 's':
            sitl_cfg.speedup = strtof(optarg, NULL);
            break;
        case 'A':
            sitl_cfg.bind_any = true;
            break;
        case 'n':
            sitl_cfg.node_id = atoi(optarg);
            break;
        case 'I':
            sitl_cfg.input_type = atoi(optarg);
            break;
        case 'U':
            sitl_cfg.uid = optarg;
            break;
        case 'B':
            sitl_cfg.bootloader_path = optarg;
            break;
        case 'L':
            sitl_cfg.physics_log = optarg;
            break;
        case 'v':
            sitl_cfg.verbose = true;
            break;
        case 'N':
            sitl_cfg.nosleep = true;
            break;
        case 'R':
            sitl_cfg.realtime = true;
            break;
        case 'h':
        default:
            usage(argv[0]);
            exit(c == 'h' ? 0 : 1);
        }
    }
    // a runtime SET_SPEEDUP is exported to the environment, which
    // survives the execv of an emulated reset; the reset exec vector is
    // the original argv, so without this the pace would jump back to
    // the command line value whenever the firmware reboots
    const char* env_speedup = getenv("AM32_SITL_SPEEDUP");
    if (env_speedup != NULL) {
        const float s = strtof(env_speedup, NULL);
        if (s >= 0 && s <= 100) {
            sitl_cfg.speedup = s;
        }
    }
    // a runtime stuck rotor setting survives resets the same way: the
    // obstruction is still in the prop when the firmware reboots
    const char* env_stuck = getenv("AM32_SITL_STUCK");
    if (env_stuck != NULL) {
        const float s = strtof(env_stuck, NULL);
        if (s >= 0 && s <= 1) {
            sitl_cfg.stuck = s;
        }
    }
    config_sanitise();
}
