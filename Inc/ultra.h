/*
 * ultra.h
 *
 * Dedicated firmware builds for the KISS Ultra flight controller
 * ("AM32 Ultra", successor of the 100.x release line from
 * AlkaMotors/AM32).
 *
 * Ultra support is strictly compile-time: every non-CAN F421 hardware
 * target is additionally built as a <TARGET>_ULTRA artifact with
 * ULTRA_DEDICATED defined (see f421makefile.mk and the CFLAGS mapping in
 * the main Makefile). Stock firmware contains no ultra code at all.
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

// counts DMA captures discarded because the edge count never reached a
// full packet
extern uint16_t packet_length_badcounts;

#endif // ULTRA_DEDICATED

#endif // ULTRA_H_
