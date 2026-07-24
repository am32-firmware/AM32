/*
  sitl_input.c - PWM/DShot signal input over UDP for the AM32 SITL

  Emulates the g431 input capture timer + DMA: each UDP packet carries one
  frame on the virtual signal wire (a servo pulse or a complete 16 bit
  DShot frame). Edge timestamps are synthesized in ticks of the emulated
  capture timer and fed through the firmware's unmodified detection and
  decode logic in Src/signal.c and Src/dshot.c, including input type
  auto-detection, CRC checking and bidirectional DShot auto-detect.

  Bidirectional DShot replies (eRPM and extended telemetry) are decoded
  from the firmware's GCR output buffer and sent back to the most recent
  sender in the same packet format.

  packet format (little endian):
    u16 magic 0x4453
    u8  type: 0=PWM, 1=DSHOT150, 2=DSHOT300, 3=DSHOT600
    u8  len: payload bytes after the 6 byte header (4)
    u16 flags: bit0 = line idle level (1 = idle high, ie. inverted
               bidirectional DShot)
    u16 data: PWM pulse width in microseconds, or the full 16 bit DShot
              frame (11 bit value, telemetry bit, 4 bit CRC)
 */

#include "targets.h"
#include "common.h"
#include "dshot.h"
#include "IO.h"
#include "sitl.h"
#include "sitl_config.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define SITL_INPUT_MAGIC 0x4453

enum sitl_input_type {
    SITL_INPUT_PWM = 0,
    SITL_INPUT_DSHOT150 = 1,
    SITL_INPUT_DSHOT300 = 2,
    SITL_INPUT_DSHOT600 = 3,
    SITL_INPUT_SERIAL19200 = 4, // len = N raw serial bytes (bootloader only)
    SITL_INPUT_LINE_LEVEL = 5, // constant line state, no data
};

#define SITL_INPUT_FLAG_IDLE_HIGH 0x0001
#define SITL_INPUT_FLAG_FLOATING 0x0002

struct __attribute__((packed)) input_pkt {
    uint16_t magic;
    uint8_t type;
    uint8_t len;
    uint16_t flags;
    uint16_t data;
};

extern void transfercomplete(void);
extern const char gcr_encode_table[16];
extern volatile char out_put;

static int fd = -1;
static struct sockaddr_in last_sender;
static bool have_sender;

/*
  virtual input capture peripheral, mirroring TIM15 + DMA on the g431:
  reset-on-arm timer at 160MHz/(prescaler+1), capture values are the
  timer count mod 65536 at each signal edge
 */
static volatile bool cap_armed;
static uint8_t cap_psc; // prescaler sampled at arm
static uint32_t cap_count; // DMA CNDTR equivalent
static uint32_t cap_index;
static uint64_t cap_base_ns; // sim time of CNT=0

static volatile uint8_t pin_idle_level; // from the last packet flags
static volatile bool pin_floating; // line explicitly floating (LINE_LEVEL flag)
static uint8_t last_type = SITL_INPUT_DSHOT300;
static uint64_t tx_done_ns; // sim time the BDShot reply transmit completes

static struct {
    uint32_t frames;
    uint32_t dropped;
    uint32_t replies;
    uint32_t bad_gcr;
    uint32_t serial_ignored;
} stats;

void sitl_input_init(void)
{
    if (sitl_cfg.input_port <= 0) {
        return;
    }
    fd = sitl_udp_socket();
    if (fd < 0) {
        perror("SITL: input socket");
        return;
    }
    // no SO_REUSEADDR: a second instance on the same port must fail
    // loudly instead of silently stealing datagrams
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t)sitl_cfg.input_port);
    addr.sin_addr.s_addr = htonl(sitl_cfg.bind_any ? INADDR_ANY : INADDR_LOOPBACK);
    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
        perror("SITL: input bind");
        close(fd);
        fd = -1;
        return;
    }
    fprintf(stderr, "SITL: PWM/DShot input on udp port %d\n", sitl_cfg.input_port);
}

// capture timer count at a simulated time, in ticks of 6.25ns*(psc+1)
static uint32_t cap_cnt_at(uint64_t t_ns)
{
    const uint64_t elapsed = t_ns - cap_base_ns;
    return (uint32_t)((elapsed * 4U) / (25ULL * (cap_psc + 1U))) & 0xffffU;
}

void sitl_input_arm(void)
{
    cap_armed = false;
    cap_psc = (uint8_t)ic_timer_prescaler;
    cap_count = buffersize;
    cap_index = 0;
    cap_base_ns = sitl_time_ns();
    cap_armed = true;
}

void sitl_input_timer_reset(void)
{
    // resetInputCaptureTimer(): PSC=0, CNT=0, capture DMA untouched
    cap_psc = 0;
    cap_base_ns = sitl_time_ns();
}

uint8_t sitl_input_pin_state(void)
{
    return pin_idle_level;
}

static void add_edge(uint64_t t_ns)
{
    if (cap_index < cap_count && cap_index < 64) {
        dma_buffer[cap_index++] = cap_cnt_at(t_ns);
    }
}

static void synth_frame(uint8_t type, uint16_t data)
{
    const uint64_t now = sitl_time_ns();
    if (type == SITL_INPUT_PWM) {
        // one pulse: rising then falling edge
        add_edge(now);
        add_edge(now + data * 1000ULL);
        return;
    }
    uint32_t bit_ns;
    switch (type) {
    case SITL_INPUT_DSHOT150:
        bit_ns = 6667;
        break;
    case SITL_INPUT_DSHOT300:
        bit_ns = 3333;
        break;
    case SITL_INPUT_DSHOT600:
    default:
        bit_ns = 1667;
        break;
    }
    // 16 bits MSB first, T1H = 0.75T, T0H = 0.375T. Polarity does not
    // change the captured values with both-edge capture
    for (int i = 0; i < 16; i++) {
        const uint64_t start = now + (uint64_t)i * bit_ns;
        const uint32_t high_ns = (data & (0x8000U >> i)) ? (bit_ns * 3U) / 4U : (bit_ns * 3U) / 8U;
        add_edge(start);
        add_edge(start + high_ns);
    }
}

/*
  decode the firmware's GCR output buffer back to the 16 bit BDShot reply
  frame. gcr[bp] is the pre-transition idle level; the 21 bit line code is
  gcr[bp+1..bp+21] (each entry 0 or 128, one output bit period each) and
  the 20 GCR bits are the transitions between adjacent periods
 */
static bool decode_gcr(uint16_t* frame_out)
{
    uint32_t gcrnum = 0;
    for (int j = 2; j <= 21; j++) {
        const uint8_t bit = (gcr[buffer_padding + j] != 0) != (gcr[buffer_padding + j - 1] != 0);
        gcrnum = (gcrnum << 1) | bit;
    }
    uint16_t frame = 0;
    for (int q = 3; q >= 0; q--) {
        const uint8_t code = (gcrnum >> (q * 5)) & 0x1f;
        int nibble = -1;
        for (int i = 0; i < 16; i++) {
            if ((uint8_t)gcr_encode_table[i] == code) {
                nibble = i;
                break;
            }
        }
        if (nibble < 0) {
            return false;
        }
        frame = (frame << 4) | (uint16_t)nibble;
    }
    *frame_out = frame;
    return true;
}

void sitl_input_send_reply(void)
{
    uint16_t frame;
    if (!decode_gcr(&frame)) {
        stats.bad_gcr++;
        frame = 0;
    } else if (fd >= 0 && have_sender) {
        struct input_pkt pkt = {
            .magic = SITL_INPUT_MAGIC,
            .type = last_type,
            .len = 4,
            // the BDShot reply line idles high, pulses are active low
            .flags = SITL_INPUT_FLAG_IDLE_HIGH,
            .data = frame,
        };
        sendto(fd, &pkt, sizeof(pkt), 0, (struct sockaddr*)&last_sender, sizeof(last_sender));
        stats.replies++;
    }
    // the reply DMA completes after (23+buffer_padding) bit periods of
    // 109 ticks at 160MHz/(output_timer_prescaler+1)
    const uint64_t bit_ns = (109ULL * 25ULL * (uint64_t)(output_timer_prescaler + 1)) / 4ULL;
    tx_done_ns = sitl_time_ns() + (23ULL + buffer_padding) * bit_ns;
}

/*
  DMA transfer complete interrupt, mirroring the g431
  DMA1_Channel1_IRQHandler. The servo half-transfer polarity flip is
  deliberately collapsed: both edges of a pulse are captured at once
 */
void sitl_input_dma_irq(void)
{
    if (armed && dshot_telemetry) {
        if (out_put) {
            receiveDshotDma();
            compute_dshot_flag = 2;
        } else {
            sendDshotDma();
            compute_dshot_flag = 1;
        }
        sitl_irq_pend(SITL_IRQ_EXTI15);
        return;
    }
    transfercomplete();
    sitl_irq_pend(SITL_IRQ_EXTI15);
}

// called from the sim thread every 100us
void sitl_input_poll(void)
{
    if (fd < 0) {
        return;
    }
    // complete a pending BDShot reply transmit
    if (out_put && tx_done_ns != 0 && sitl_time_ns() >= tx_done_ns) {
        tx_done_ns = 0;
        sitl_irq_pend(SITL_IRQ_DMA);
        return;
    }
    // header (6 bytes) plus up to 200 payload bytes (type 4)
    uint8_t buf[6 + 200];
    struct input_pkt pkt;
    struct sockaddr_in src;
    socklen_t srclen = sizeof(src);
    const ssize_t ret = recvfrom(fd, buf, sizeof(buf), MSG_DONTWAIT, (struct sockaddr*)&src, &srclen);
    if (ret < (ssize_t)sizeof(pkt)) {
        return;
    }
    memcpy(&pkt, buf, sizeof(pkt));
    if (pkt.magic != SITL_INPUT_MAGIC || pkt.type > SITL_INPUT_LINE_LEVEL) {
        return;
    }
    if (pkt.type == SITL_INPUT_SERIAL19200) {
        if (pkt.len < 1 || pkt.len > 200 || ret != 6 + pkt.len) {
            return;
        }
        // 19200 serial is for the bootloader; the main firmware has no
        // uart on the signal pin so the bytes are ignored
        stats.serial_ignored++;
        return;
    }
    if (pkt.len != 4) {
        return;
    }
    if (pkt.type == SITL_INPUT_LINE_LEVEL) {
        // constant line state, e.g. an FC holding the wire. A floating
        // line reads low here (no pull on the fw input by default)
        pin_floating = (pkt.flags & SITL_INPUT_FLAG_FLOATING) != 0;
        pin_idle_level = (!pin_floating && (pkt.flags & SITL_INPUT_FLAG_IDLE_HIGH)) ? 1 : 0;
        return;
    }
    last_sender = src;
    have_sender = true;
    last_type = pkt.type;
    pin_floating = false;
    pin_idle_level = (pkt.flags & SITL_INPUT_FLAG_IDLE_HIGH) ? 1 : 0;
    stats.frames++;
    if (!cap_armed || out_put) {
        // capture not armed (or the wire is driven by the reply): the
        // frame is lost, as on the real signal wire
        stats.dropped++;
        return;
    }
    synth_frame(pkt.type, pkt.data);
    if (cap_index >= cap_count) {
        cap_armed = false;
        sitl_irq_pend(SITL_IRQ_DMA);
    }
}

void sitl_input_stats(uint32_t out[4])
{
    out[0] = stats.frames;
    out[1] = stats.dropped;
    out[2] = stats.replies;
    out[3] = stats.bad_gcr;
}
