/*
 * Host test for ultraPacketStart() (Inc/ultra.h), the packet boundary
 * check every MCU's runDshotCheck() relies on.
 *
 *   gcc -Wall -Wextra -Werror -DULTRA_DEDICATED -IInc \
 *       tests/ultra_packet_start_test.c -o obj/ultra_packet_start_test \
 *       && obj/ultra_packet_start_test
 */
#include "ultra.h"
#include <stdio.h>

#define BIT_TICKS 40 // one DShot bit, in input timer ticks
#define GAP (BIT_TICKS * 2) // valid_packet_high: frametime / 16
#define IDLE 3000 // line idle between frames

static uint32_t edges[64];
static int failures = 0;

// writes `count` edges starting at index `at`, first edge at time `t`,
// alternating 3/4 and 1/4 bit spacing like a frame of zero bits
static uint32_t put_edges(int at, int count, uint32_t t)
{
    for (int i = 0; i < count; i++) {
        edges[at + i] = t;
        t += (i & 1) ? (BIT_TICKS / 4) : (BIT_TICKS * 3 / 4);
    }
    return t;
}

static void check(const char* name, uint16_t captured, uint8_t want_ok, uint16_t want_start)
{
    uint16_t start = 0xFFFF;
    uint8_t ok = ultraPacketStart(edges, 64 - captured, GAP, &start);
    // a rejected capture must leave *start alone: runDshotCheck() passes
    // DMA_start_bit in directly, and that has to stay at the last accepted
    // packet's value
    if (ok != want_ok || (want_ok && start != want_start) || (!want_ok && start != 0xFFFF)) {
        printf("FAIL %s: ok=%u start=%u, want ok=%u start=%u\n", name, ok, start, want_ok, want_start);
        failures++;
    } else {
        printf("ok   %s\n", name);
    }
}

int main(void)
{
    // a clean frame: exactly 32 edges, decode from index 0
    put_edges(0, 32, 100);
    check("clean frame", 32, 1, 0);

    // truncated capture: fewer than 32 edges is never a frame
    put_edges(0, 31, 100);
    check("31 edges", 31, 0, 0);

    // two noise edges, idle gap, then the frame: skip the noise
    uint32_t t = put_edges(0, 2, 100);
    put_edges(2, 32, t + IDLE);
    check("noise ahead of the frame", 34, 1, 2);

    // the frame followed directly by one noise edge: 33 edges, and the
    // last 32 are the frame shifted by one edge. Must be dropped - decoding
    // it inverts every bit
    t = put_edges(0, 32, 100);
    edges[32] = t + BIT_TICKS / 4;
    check("noise edge trailing the frame", 33, 0, 0);

    // a gap of exactly `gap` ticks is not an idle gap
    t = put_edges(0, 1, 100);
    put_edges(1, 32, t - BIT_TICKS * 3 / 4 + GAP);
    check("gap == threshold", 33, 0, 0);
    t = put_edges(0, 1, 100);
    put_edges(1, 32, t - BIT_TICKS * 3 / 4 + GAP + 1);
    check("gap == threshold + 1", 33, 1, 1);

    // the 16 bit input timer wraps between the noise and the frame
    edges[0] = 0xFFF0;
    put_edges(1, 32, 0x10000 + IDLE);
    for (int i = 1; i < 33; i++) {
        edges[i] &= 0xFFFF;
    }
    check("timer wrap inside the gap", 33, 1, 1);

    // buffer full of edges: the frame is the last 32
    t = put_edges(0, 32, 100);
    put_edges(32, 32, t + IDLE);
    check("64 edges", 64, 1, 32);

    if (failures) {
        printf("%d failure(s)\n", failures);
        return 1;
    }
    printf("all passed\n");
    return 0;
}
