/*
 * ultra.h
 *
 * Dedicated firmware builds for the KISS Ultra flight controller
 * ("AM32 Ultra", successor of the 100.x release line from
 * AlkaMotors/AM32).
 *
 * Ultra support is strictly compile-time: every non-CAN hardware target
 * of the capable MCUs (F421, F415, G071, G431, L431, E230) is additionally
 * built as a <TARGET>_ULTRA artifact with ULTRA_DEDICATED defined (see the
 * <mcu>makefile.mk files and the CFLAGS mapping in the main Makefile).
 * Stock firmware contains no ultra code at all. Ultra artifacts are
 * versioned n00.x (see Inc/version.h).
 *
 * Feature set, ported from the AM32 Ultra 100.20 fork with fixes from
 * bench sessions on a KISS Ultra FC:
 *   - KISS telemetry detection marker (eRPM 0xFFFC, first 20 packets)
 *   - DShot cmds 30/31 -> telemetry baud 115200 / 2M, commands accepted
 *     while disarmed
 *   - per-request fast telemetry with in-flight TX guard
 *   - polled dshot input (runDshotCheck)
 *   - non-blocking tone engine, DShot-standard 0.1s beacons
 *   - telemetry keeps flowing during buzzer beacons (the FC takes a
 *     telemetry gap as an ESC reboot and falls back to 115200)
 *   - compile-outs (servo/PWM input, bidirectional dshot, BlueJay)
 */

#ifndef ULTRA_H_
#define ULTRA_H_

#include <stdint.h>

#ifdef ULTRA_DEDICATED

// number of noise edges preceding the packet in dma_buffer, determined by
// runDshotCheck()
extern uint16_t DMA_start_bit;

// shortest edge-to-edge gap treated as packet separation (frametime / 16)
extern uint16_t valid_packet_high;

// counts DMA captures runDshotCheck() discarded: too few edges for a full
// packet, or extra edges not separated from it by an idle gap
extern uint16_t packet_length_badcounts;

// Packet boundary check shared by every MCU's runDshotCheck(). The input
// capture DMA is 64 deep and `remaining` is its transfer counter once the
// line has gone idle: 64 - remaining edges were captured and a DShot frame
// is the last 32 of them. Returns 1 and the index of the frame's first
// edge in *start when the capture may be decoded, 0 when it must be dropped.
// *start is only written when the capture is accepted. Edge times are
// captures of the 16 bit input timer (0xFFFF reload on every MCU), hence
// the uint16_t difference.
//
// Edges ahead of the frame are only skipped as noise when an idle gap
// (more than `gap` timer ticks) separates them from it: a noise edge after
// the frame would shift the decode by one edge and invert every bit, and
// an inverted zero-throttle frame is full throttle with a valid CRC.
static inline uint8_t ultraPacketStart(const uint32_t* edges, uint16_t remaining, uint16_t gap, uint16_t* start)
{
    if (remaining > 32) {
        return 0; // fewer than 32 edges: not a full frame
    }
    const uint16_t first = 32 - remaining;
    if (first != 0 && (uint16_t)(edges[first] - edges[first - 1]) <= gap) {
        return 0;
    }
    *start = first;
    return 1;
}

#endif // ULTRA_DEDICATED

#endif // ULTRA_H_
