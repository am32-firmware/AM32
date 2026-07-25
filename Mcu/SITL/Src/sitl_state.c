/*
  sitl_state.c - high rate simulation state streaming and runtime model
  control over UDP, for GUI graphs and animations

  A client subscribes by sending a SUBSCRIBE packet with the desired
  sample period in simulated nanoseconds; the simulation thread then
  streams batched state samples (rotor angle/speed, phase currents, bus
  voltage/current, bridge modes, comparator) to the subscriber. The
  subscription expires two seconds after the last refresh.

  A LOAD_MODEL packet carries the path of a motor/battery/esc JSON file
  (same format as --config, sim section ignored) which is applied to the
  running simulation.

  client -> SITL (little endian):
    u16 magic 0x5353, u8 cmd, u8 pad, payload
      cmd 0 SUBSCRIBE: u32 period_ns; flags byte bit0 = averaged
        sampling (currents/voltages are the mean over each sample
        period instead of instantaneous, avoiding PWM aliasing at
        coarse periods)
      cmd 1 LOAD_MODEL: JSON file path (rest of packet)
      cmd 2 SET_SPEEDUP: float speedup (0 = free run)
      cmd 3 SUBSCRIBE_TONES: no payload. Streams tone events: the
        firmware beeps by driving the motor PWM at an audible
        frequency (sounds.c), which shows up here as TIM1 PSC > 0.
        An event is sent on every tone change plus a 20Hz wall-clock
        keepalive so a lost "off" event cannot leave a stuck tone.
      cmd 4 SUBSCRIBE_AUDIO: no payload. Streams physics audio: what
        the motor radiates acoustically, derived from torque ripple
        plus phase current magnitude, high-pass filtered and sampled
        at 48kHz of SIMULATED time (pitch scales with speedup, as
        slow motion should sound)
      cmd 7 SET_STUCK: float 0..1, stuck rotor fraction (prop blocked
        by an obstruction, e.g. a tree branch): scales the model's
        holding torque, 1.0 locks the rotor rigidly
  SITL -> client:
    u16 magic 0x5354, u8 version=1, u8 count, count * sample
    u16 magic 0x5355, u8 ok, u8 pad, message   (LOAD_MODEL reply)

  eeprom access (cmd 5/6) lets a local tool read and edit the ESC
  settings directly, without the 4-way or DroneCAN parameter paths:
    cmd 5: fetch, replies 0x5356-tagged image (u16 magic, u8 cmd=5,
           u8 len_hi... see eeprom_reply below)
    cmd 6: set, u16 offset, u16 length, bytes[]; writes the live
           eepromBuffer and the backing file
    u16 magic 0x5356, u8 version=1, u8 source, u64 t_ns (simulated),
        float freq_hz, float amplitude   (tone event; amplitude is
        the raw PWM duty fraction, 0 = silence; source 0 = TIM1
        synthesized, source 1 = physics audio stream active)
    u16 magic 0x5357, u8 version=1, u8 count, u64 t0_ns (simulated,
        first sample), u32 sample_period_ns, count * float
        (physics audio samples, arbitrary linear units)
*/

#include "eeprom.h"
#include "targets.h"
#include "sitl.h"
#include "sitl_config.h"
#include "motor.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define STATE_MAGIC_CMD 0x5353
#define STATE_MAGIC_DATA 0x5354
#define STATE_MAGIC_REPLY 0x5355
#define STATE_MAGIC_TONE 0x5356
#define STATE_MAGIC_AUDIO 0x5357

struct __attribute__((packed)) state_sample {
    uint64_t t_ns;
    float omega; // mechanical rad/s
    float theta; // mechanical angle, rad [0,2pi)
    float theta_e; // electrical angle, rad [0,2pi)
    float iu, iv, iw; // phase currents, A
    float vu, vv, vw; // phase terminal voltages, V
    float vbus, ibus;
    uint8_t modes[3]; // sitl_phase_mode per phase
    uint8_t comp_phase; // floating phase
    uint8_t comp_out;
    uint8_t pad[3];
};

#define STATE_BATCH 16

static int fd = -1;
static struct sockaddr_in sub_addr;
static bool have_sub;
static time_t sub_expire;
static uint32_t period_req_ns = 50000; // requested by the subscriber
static uint32_t period_ns = 50000; // effective, wall rate limited
static bool averaged; // mean over the period instead of point samples
static double sig_acc[8];
static uint32_t sig_n;
static uint64_t next_sample_ns;
static uint64_t last_flush_ns;

static struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t count;
    struct state_sample s[STATE_BATCH];
} batch = { .magic = STATE_MAGIC_DATA, .version = 2 };

// tone event stream (cmd 3)
static struct sockaddr_in tone_addr;
static bool tone_have_sub;
static time_t tone_expire;
static uint32_t tone_raw[5]; // last seen psc, arr, ccr[3]
static uint8_t tone_raw_modes[3];
static float tone_freq, tone_amp; // last sent
static uint64_t tone_last_tx_wall_ns;

#define TONE_KEEPALIVE_NS 50000000ULL

/*
  physics audio stream (cmd 4): 48kHz of simulated time, batched.
  Each sample is the mean over its period of the torque plus a
  weighted phase current magnitude, both high-pass filtered to strip
  the DC operating point and keep only the audible ripple
 */
#define AUDIO_RATE_HZ 48000
#define AUDIO_PERIOD_NS (1000000000ULL / AUDIO_RATE_HZ)
#define AUDIO_BATCH 64
// N*m per A weighting of the current magnitude term: keeps beeps
// audible when the rotor happens to sit at a zero-torque angle
#define AUDIO_CURRENT_WEIGHT 0.02
// one pole high pass at ~40Hz: a = 1 - 2*pi*fc/fs
#define AUDIO_HPF_A 0.9948

static struct sockaddr_in audio_addr;
static bool audio_have_sub;
static time_t audio_expire;
static double audio_acc[2];
static uint32_t audio_acc_n;
static uint64_t audio_next_ns;
static double audio_hp_y[2], audio_hp_x[2]; // HPF state per signal

static struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t count;
    uint64_t t0_ns;
    uint32_t period_ns;
    float s[AUDIO_BATCH];
} audio_batch = { .magic = STATE_MAGIC_AUDIO, .version = 1,
                  .period_ns = (uint32_t)AUDIO_PERIOD_NS };

static void audio_step(uint64_t now_ns)
{
    if (!audio_have_sub) {
        return;
    }
    motor_add_audio(audio_acc);
    audio_acc_n++;
    if (now_ns < audio_next_ns) {
        return;
    }
    audio_next_ns = now_ns + AUDIO_PERIOD_NS;

    double s = 0;
    for (int k = 0; k < 2; k++) {
        const double x = audio_acc[k] / audio_acc_n;
        audio_hp_y[k] = AUDIO_HPF_A * (audio_hp_y[k] + x - audio_hp_x[k]);
        audio_hp_x[k] = x;
        s += k == 0 ? audio_hp_y[k] : AUDIO_CURRENT_WEIGHT * audio_hp_y[k];
    }
    audio_acc[0] = audio_acc[1] = 0;
    audio_acc_n = 0;

    if (audio_batch.count == 0) {
        audio_batch.t0_ns = now_ns;
    }
    audio_batch.s[audio_batch.count++] = (float)s;
    if (audio_batch.count >= AUDIO_BATCH) {
        sendto(fd, &audio_batch, sizeof(audio_batch), 0,
               (struct sockaddr*)&audio_addr, sizeof(audio_addr));
        audio_batch.count = 0;
    }
}

/*
  tone/audio subscribers are preserved across an emulated reset (which
  re-execs the process) via the environment, so the boot tune after a
  reset is delivered from its first note. The expiry still applies if
  the subscriber is gone
 */
#define TONE_SUB_ENV "AM32_SITL_TONE_SUB"
#define AUDIO_SUB_ENV "AM32_SITL_AUDIO_SUB"

static void sub_save_env(const char* env, const struct sockaddr_in* a)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "%s:%u", inet_ntoa(a->sin_addr),
             ntohs(a->sin_port));
    setenv(env, buf, 1);
}

static bool sub_restore_env(const char* env, struct sockaddr_in* a)
{
    const char* val = getenv(env);
    char ip[24];
    unsigned port;
    if (val == NULL || sscanf(val, "%23[0-9.]:%u", ip, &port) != 2) {
        return false;
    }
    memset(a, 0, sizeof(*a));
    a->sin_family = AF_INET;
    a->sin_addr.s_addr = inet_addr(ip);
    a->sin_port = htons((uint16_t)port);
    return true;
}

static void tone_send(uint64_t now_ns)
{
    struct __attribute__((packed)) {
        uint16_t magic;
        uint8_t version;
        uint8_t source;
        uint64_t t_ns;
        float freq_hz;
        float amplitude;
    } ev = { STATE_MAGIC_TONE, 1, 0, now_ns, tone_freq, tone_amp };
    sendto(fd, &ev, sizeof(ev), 0, (struct sockaddr*)&tone_addr, sizeof(tone_addr));
    tone_last_tx_wall_ns = sitl_wallclock_ns();
}

/*
  detect an audible tone from the latched TIM1 state. Only sounds.c
  ever sets a non-zero PWM prescaler, so PSC > 0 with a driven phase
  is a beep; running motor code changes ARR/CCR only
 */
static void tone_step(uint64_t now_ns)
{
    if (!tone_have_sub) {
        return;
    }
    uint32_t raw[5];
    sitl_tim1_get_active(&raw[0], &raw[1], &raw[2]);
    if (memcmp(raw, tone_raw, sizeof(raw)) == 0
        && memcmp((const void*)sitl_phase_mode, tone_raw_modes, 3) == 0) {
        return;
    }
    memcpy(tone_raw, raw, sizeof(raw));
    memcpy(tone_raw_modes, (const void*)sitl_phase_mode, 3);

    const uint32_t psc = raw[0], arr = raw[1];
    uint32_t ccr = 0;
    for (int p = 0; p < 3; p++) {
        const uint8_t mode = tone_raw_modes[p];
        if ((mode == SITL_PHASE_PWM || mode == SITL_PHASE_PWM_NOCOMP) && raw[2 + p] > ccr) {
            ccr = raw[2 + p];
        }
    }
    const float freq = 160e6f / (float)((psc + 1) * (arr + 1));
    float new_freq = 0, new_amp = 0;
    if (psc > 0 && ccr > 0 && freq < 20000) {
        new_freq = freq;
        new_amp = (float)ccr / (float)(arr + 1);
    }
    if (new_freq != tone_freq || new_amp != tone_amp) {
        tone_freq = new_freq;
        tone_amp = new_amp;
        tone_send(now_ns);
    }
}

void sitl_state_init(void)
{
    if (sitl_cfg.state_port <= 0) {
        return;
    }
    fd = sitl_udp_socket();
    if (fd < 0) {
        perror("SITL: state socket");
        return;
    }
    // no SO_REUSEADDR: a second instance on the same port must fail
    // loudly instead of silently stealing datagrams
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t)sitl_cfg.state_port);
    addr.sin_addr.s_addr = htonl(sitl_cfg.bind_any ? INADDR_ANY : INADDR_LOOPBACK);
    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
        perror("SITL: state bind");
        close(fd);
        fd = -1;
        return;
    }
    fprintf(stderr, "SITL: state/model port udp %d\n", sitl_cfg.state_port);
    if (sub_restore_env(TONE_SUB_ENV, &tone_addr)) {
        tone_have_sub = true;
        tone_expire = time(NULL) + 2;
    }
    if (sub_restore_env(AUDIO_SUB_ENV, &audio_addr)) {
        audio_have_sub = true;
        audio_expire = time(NULL) + 2;
    }
}

static void load_model(const char* path, struct sockaddr_in* src)
{
    char msg[256];
    const bool ok = sitl_config_reload(path);
    if (ok) {
        motor_config_changed();
        snprintf(msg, sizeof(msg), "loaded %.200s", path);
        fprintf(stderr, "SITL: %s\n", msg);
    } else {
        snprintf(msg, sizeof(msg), "failed to load %.200s", path);
        fprintf(stderr, "SITL: %s\n", msg);
    }
    struct __attribute__((packed)) {
        uint16_t magic;
        uint8_t ok;
        uint8_t pad;
        char msg[256];
    } reply = { .magic = STATE_MAGIC_REPLY, .ok = ok, .pad = 0 };
    strncpy(reply.msg, msg, sizeof(reply.msg) - 1);
    sendto(fd, &reply, 4 + strlen(reply.msg) + 1, 0, (struct sockaddr*)src, sizeof(*src));
}

/*
  eeprom access for a local editor (cmd 5 fetch / cmd 6 set). This
  deliberately bypasses the 4-way and DroneCAN parameter paths: it
  edits the simulated ESC's settings the way a bench technician would
  reflash them, which is what the GUI parameter editor wants
 */
#define STATE_MAGIC_EEPROM 0x5358

static void eeprom_fetch(struct sockaddr_in* src)
{
    // the simulated motor travels with the image so an editor can flag
    // settings that disagree with what is being simulated
    struct __attribute__((packed)) {
        uint16_t magic;
        uint8_t cmd;
        uint8_t pad;
        uint16_t length;
        uint16_t pad2;
        float model_kv;
        uint8_t model_poles;
        uint8_t pad3[3];
        uint8_t data[sizeof(eepromBuffer.buffer)];
    } reply = { .magic = STATE_MAGIC_EEPROM, .cmd = 5, .pad = 0,
                .length = (uint16_t)sizeof(eepromBuffer.buffer), .pad2 = 0,
                .model_kv = sitl_cfg.motor.kv,
                .model_poles = (uint8_t)sitl_cfg.motor.poles };
    memcpy(reply.data, eepromBuffer.buffer, sizeof(reply.data));
    sendto(fd, &reply, 16 + sizeof(reply.data), 0,
           (struct sockaddr*)src, sizeof(*src));
}

static void eeprom_set(uint16_t off, uint16_t len, const uint8_t* data,
                       int avail, struct sockaddr_in* src)
{
    char msg[128];
    bool ok = false;
    if (len > (uint16_t)avail) {
        snprintf(msg, sizeof(msg), "truncated: %u bytes for length %u",
                 (unsigned)avail, (unsigned)len);
    } else if ((int)off + (int)len > (int)sizeof(eepromBuffer.buffer)) {
        snprintf(msg, sizeof(msg), "range %u+%u past the eeprom (%u bytes)",
                 (unsigned)off, (unsigned)len,
                 (unsigned)sizeof(eepromBuffer.buffer));
    } else {
        memcpy(eepromBuffer.buffer + off, data, len);
        // persist the whole image so the file always matches the live
        // settings, then let the firmware re-read them
        save_flash_nolib(eepromBuffer.buffer, sizeof(eepromBuffer.buffer),
                         EEPROM_START_ADD);
        extern void loadEEpromSettings(void);
        loadEEpromSettings();
        ok = true;
        snprintf(msg, sizeof(msg), "wrote %u bytes at %u",
                 (unsigned)len, (unsigned)off);
        fprintf(stderr, "SITL: eeprom %s\n", msg);
    }
    struct __attribute__((packed)) {
        uint16_t magic;
        uint8_t ok;
        uint8_t pad;
        char msg[128];
    } reply = { .magic = STATE_MAGIC_REPLY, .ok = ok, .pad = 0 };
    strncpy(reply.msg, msg, sizeof(reply.msg) - 1);
    sendto(fd, &reply, 4 + strlen(reply.msg) + 1, 0,
           (struct sockaddr*)src, sizeof(*src));
}

/*
  effective sample period: the subscriber's request, floored to the
  physics step and rate limited to about 200k samples per second of
  wall clock so fine sampling does not overload the sim thread at high
  speedups. Slowing the simulation down automatically allows finer
  sampling
 */
static void apply_period(void)
{
    uint32_t period = period_req_ns;
    if (period < sitl_cfg.sim.physics_dt_ns) {
        period = sitl_cfg.sim.physics_dt_ns;
    }
    const uint32_t wall_floor = (uint32_t)(5000.0f * sitl_cfg.speedup);
    if (sitl_cfg.speedup > 0 && period < wall_floor) {
        period = wall_floor;
    }
    period_ns = period;
}

// called from the sim thread every 100us
void sitl_state_poll(void)
{
    if (fd < 0) {
        return;
    }
    if (have_sub && time(NULL) > sub_expire) {
        have_sub = false;
    }
    if (tone_have_sub) {
        if (time(NULL) > tone_expire) {
            tone_have_sub = false;
            unsetenv(TONE_SUB_ENV);
        } else if (sitl_wallclock_ns() - tone_last_tx_wall_ns > TONE_KEEPALIVE_NS) {
            tone_send(sitl_time_ns());
        }
    }
    if (audio_have_sub && time(NULL) > audio_expire) {
        audio_have_sub = false;
        unsetenv(AUDIO_SUB_ENV);
    }
    uint8_t pkt[512];
    struct sockaddr_in src;
    socklen_t srclen = sizeof(src);
    const ssize_t ret = recvfrom(fd, pkt, sizeof(pkt) - 1, MSG_DONTWAIT, (struct sockaddr*)&src, &srclen);
    if (ret < 4) {
        return;
    }
    uint16_t magic;
    memcpy(&magic, pkt, 2);
    if (magic != STATE_MAGIC_CMD) {
        return;
    }
    const uint8_t cmd = pkt[2];
    if (cmd == 0 && ret >= 8) {
        memcpy(&period_req_ns, pkt + 4, 4);
        averaged = (pkt[3] & 1) != 0;
        apply_period();
        // a new subscriber must not receive samples batched for the
        // previous one
        if (!have_sub || src.sin_addr.s_addr != sub_addr.sin_addr.s_addr
            || src.sin_port != sub_addr.sin_port) {
            batch.count = 0;
            memset(sig_acc, 0, sizeof(sig_acc));
            sig_n = 0;
            next_sample_ns = 0;
        }
        sub_addr = src;
        have_sub = true;
        sub_expire = time(NULL) + 2;
    } else if (cmd == 1) {
        pkt[ret] = 0;
        load_model((const char*)(pkt + 4), &src);
    } else if (cmd == 2 && ret >= 8) {
        float speedup;
        memcpy(&speedup, pkt + 4, 4);
        if (speedup >= 0 && speedup <= 100) {
            // the pacing loop rebases its references on change
            sitl_cfg.speedup = speedup;
            apply_period();
            // survive the re-exec of an emulated reset, like the
            // tone/audio subscribers (sitl_config.c reads it back)
            char buf[24];
            snprintf(buf, sizeof(buf), "%g", (double)speedup);
            setenv("AM32_SITL_SPEEDUP", buf, 1);
            fprintf(stderr, "SITL: speedup %.3f\n", (double)speedup);
        }
    } else if (cmd == 3) {
        tone_addr = src;
        tone_have_sub = true;
        tone_expire = time(NULL) + 2;
        sub_save_env(TONE_SUB_ENV, &tone_addr);
        memset(tone_raw, 0xff, sizeof(tone_raw)); // force recompute
        tone_step(sitl_time_ns());
        tone_send(sitl_time_ns()); // snapshot for the (re)subscriber
    } else if (cmd == 7 && ret >= 8) {
        float stuck;
        memcpy(&stuck, pkt + 4, 4);
        if (stuck >= 0 && stuck <= 1) {
            sitl_cfg.stuck = stuck;
            // survive the re-exec of an emulated reset: the obstruction
            // is still in the prop when the firmware reboots
            char buf[24];
            // full precision: %g would round 0.99899 up across the
            // 1.0 rigid-lock threshold on the way through the reset
            snprintf(buf, sizeof(buf), "%.9g", (double)stuck);
            setenv("AM32_SITL_STUCK", buf, 1);
            fprintf(stderr, "SITL: stuck rotor %.2f\n", (double)stuck);
        }
    } else if (cmd == 5) {
        eeprom_fetch(&src);
    } else if (cmd == 6 && ret >= 8) {
        uint16_t off, len;
        memcpy(&off, pkt + 4, 2);
        memcpy(&len, pkt + 6, 2);
        eeprom_set(off, len, pkt + 8, (int)(ret - 8), &src);
    } else if (cmd == 4) {
        if (!audio_have_sub || src.sin_addr.s_addr != audio_addr.sin_addr.s_addr
            || src.sin_port != audio_addr.sin_port) {
            audio_batch.count = 0;
            audio_acc[0] = audio_acc[1] = 0;
            audio_acc_n = 0;
            audio_next_ns = 0;
        }
        audio_addr = src;
        audio_have_sub = true;
        audio_expire = time(NULL) + 2;
        sub_save_env(AUDIO_SUB_ENV, &audio_addr);
    }
}

// called from the sim thread on every physics step
void sitl_state_step(uint64_t now_ns)
{
    tone_step(now_ns);
    audio_step(now_ns);
    if (!have_sub) {
        return;
    }
    if (averaged) {
        motor_add_signals(sig_acc);
        sig_n++;
    }
    if (now_ns < next_sample_ns) {
        return;
    }
    next_sample_ns = now_ns + period_ns;

    struct state_sample* s = &batch.s[batch.count];
    memset(s, 0, sizeof(*s));
    s->t_ns = now_ns;
    float omega, theta, theta_e, i[3], v[3], vbus, ibus;
    motor_get_live_state(&omega, &theta, &theta_e, i, v, &vbus, &ibus);
    s->omega = omega;
    s->theta = theta;
    s->theta_e = theta_e;
    if (averaged && sig_n > 0) {
        s->iu = (float)(sig_acc[0] / sig_n);
        s->iv = (float)(sig_acc[1] / sig_n);
        s->iw = (float)(sig_acc[2] / sig_n);
        s->vu = (float)(sig_acc[3] / sig_n);
        s->vv = (float)(sig_acc[4] / sig_n);
        s->vw = (float)(sig_acc[5] / sig_n);
        s->vbus = (float)(sig_acc[6] / sig_n);
        s->ibus = (float)(sig_acc[7] / sig_n);
        memset(sig_acc, 0, sizeof(sig_acc));
        sig_n = 0;
    } else {
        s->iu = i[0];
        s->iv = i[1];
        s->iw = i[2];
        s->vu = v[0];
        s->vv = v[1];
        s->vw = v[2];
        s->vbus = vbus;
        s->ibus = ibus;
    }
    for (int p = 0; p < 3; p++) {
        s->modes[p] = sitl_phase_mode[p];
    }
    s->comp_phase = sitl_comp_phase;
    s->comp_out = sitl_comp_out;
    batch.count++;

    if (batch.count >= STATE_BATCH || now_ns - last_flush_ns > 5000000ULL) {
        sendto(fd, &batch, 4 + batch.count * sizeof(struct state_sample), 0,
               (struct sockaddr*)&sub_addr, sizeof(sub_addr));
        batch.count = 0;
        last_flush_ns = now_ns;
    }
}
