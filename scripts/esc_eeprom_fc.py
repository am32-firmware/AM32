#!/usr/bin/env python3
'''
read an AM32 ESC's settings through a Betaflight flight controller

Uses the FC's BLHeli 4-way passthrough (the link the configurators use)
to talk to the ESC's bootloader and dump its eeprom, so a calibration
data set can record the settings the capture was taken with. The
result is the 192 byte image the SITL loads with --eeprom.

Entering passthrough stops the motor outputs and needs the ESC in its
bootloader, so do this before or after a capture session, never in the
middle of one.

Getting the ESC into its bootloader is the awkward part, and it is why
this tool probes in a loop rather than trying once:

  - the application only reboots into the bootloader after its signal
    timeout, but every tone it plays clears that timeout (see
    signaltimeout in Src/sounds.c), so an ESC beeping about the lost
    signal can keep itself in the application indefinitely
  - on a DroneCAN capable build the bootloader jumps to the
    application NONCAN_FALLBACK_MS (250ms) after boot unless it has
    seen 4-way traffic, so a client that probes once, or slowly, will
    miss the window every time

So: reset the ESC (power cycle it, or reset it over SWD) while this is
running. It probes continuously and only needs one to land inside that
250ms window - the bootloader then stays put for the session. If it
still will not connect, dump the settings with the AM32 configurator
instead and include that with the capture.

usage:
  esc_eeprom_fc.py --port /dev/ttyACM0 --out sitl_eeprom.bin
'''

import argparse
import struct
import sys
import time

import serial

import msp

# 4-way interface (serial_4way.h)
LOCAL_ESCAPE = 0x2F
REMOTE_ESCAPE = 0x2E
CMD_INTERFACE_TEST_ALIVE = 0x30
CMD_PROTOCOL_GET_VERSION = 0x31
CMD_INTERFACE_GET_NAME = 0x32
CMD_INTERFACE_GET_VERSION = 0x33
CMD_INTERFACE_EXIT = 0x34
CMD_DEVICE_RESET = 0x35
CMD_DEVICE_INIT_FLASH = 0x37
CMD_DEVICE_READ = 0x3A
ACK_OK = 0x00

# AM32 bootloader magic addresses (bootloader/main.c). The bootloader
# maps these to the real flash locations, so no per-MCU address table
# is needed - but only from protocol version 2 (eeprom) and 3 (devinfo)
ADDR_MAGIC_EEPROM = 0x0020
ADDR_MAGIC_DEVINFO = 0x0023
EEPROM_SIZE = 192


def crc_xmodem(data, crc=0):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class FourWay(object):
    '''BLHeli 4-way client over an FC serial port already switched into
    passthrough mode'''

    def __init__(self, ser):
        self.ser = ser

    def send(self, cmd, addr=0, param=b''):
        # the length byte is read into a do-while, so 0 means 256 bytes:
        # commands without a parameter must still send one dummy byte
        if not param:
            param = b'\x00'
        body = bytes([LOCAL_ESCAPE, cmd, (addr >> 8) & 0xFF, addr & 0xFF,
                      len(param) & 0xFF]) + param
        self.ser.write(body + struct.pack('>H', crc_xmodem(body)))

    def resync(self):
        '''finish any oversized read a previous client left the FC in,
        then drain. Without this a half-parsed command wedges the link'''
        self.ser.write(b'\x00' * 300)
        time.sleep(0.5)
        self.ser.reset_input_buffer()

    def read_reply(self, timeout=3.0):
        '''returns (cmd, payload, ack) or None'''
        deadline = time.time() + timeout
        buf = b''
        while time.time() < deadline:
            buf += self.ser.read(256)
            start = buf.find(bytes([REMOTE_ESCAPE]))
            if start < 0:
                continue
            frame = buf[start:]
            if len(frame) < 5:
                continue
            plen = frame[4] or 256
            need = 5 + plen + 1 + 2
            if len(frame) < need:
                continue
            payload = frame[5:5 + plen]
            ack = frame[5 + plen]
            got = struct.unpack('>H', frame[need - 2:need])[0]
            if got != crc_xmodem(frame[:need - 2]):
                raise IOError('4way CRC error')
            return frame[1], payload, ack
        return None

    def cmd(self, cmd, addr=0, param=b'', timeout=3.0):
        self.send(cmd, addr, param)
        r = self.read_reply(timeout)
        if r is None:
            raise IOError('no 4way reply to command 0x%02x' % cmd)
        _, payload, ack = r
        if ack != ACK_OK:
            raise IOError('4way command 0x%02x rejected (ack 0x%02x)' % (cmd, ack))
        return payload


def enter_passthrough(port):
    '''switch the FC's MSP port into 4-way mode, returns the ESC count'''
    p = msp.MspPort(port)
    try:
        data = p.request(msp.MSP_SET_PASSTHROUGH, b'', timeout=5.0)
        count = data[0] if data else 0
    finally:
        # hand the raw port over: MSP is no longer spoken on it
        p.running = False
        p.thread.join(timeout=1.0)
        ser = p.ser
    return ser, count


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', required=True, help='FC serial port')
    ap.add_argument('--motor', type=int, default=1,
                    help='motor number the ESC is on (1-based)')
    ap.add_argument('--out', default='sitl_eeprom.bin',
                    help='where to write the eeprom image')
    ap.add_argument('--connect-timeout', type=float, default=60.0,
                    help='how long to keep probing for the ESC bootloader')
    args = ap.parse_args()

    ser, count = enter_passthrough(args.port)
    print('4-way passthrough: %u ESC(s) detected' % count)
    if count == 0:
        raise SystemExit('no ESCs found - check the signal wiring')
    if args.motor > count:
        raise SystemExit('--motor %u but only %u ESC(s) found'
                         % (args.motor, count))
    fw = FourWay(ser)
    try:
        time.sleep(0.3)          # the FC starts its 4-way loop after the reply
        try:
            fw.cmd(CMD_INTERFACE_TEST_ALIVE)
        except IOError:
            fw.resync()
            fw.cmd(CMD_INTERFACE_TEST_ALIVE)
        ver = fw.cmd(CMD_INTERFACE_GET_VERSION)
        name = fw.cmd(CMD_INTERFACE_GET_NAME)
        print('interface: %s v%s' % (name.decode('ascii', 'replace'),
                                     '.'.join(str(b) for b in ver)))
        print('probing for the ESC bootloader (reset or power cycle the ESC '
              'now if it does not connect)')
        info = None
        t0 = time.time()
        probes = 0
        while time.time() - t0 < args.connect_timeout:
            probes += 1
            fw.send(CMD_DEVICE_INIT_FLASH, param=bytes([args.motor - 1]))
            # short timeout keeps the probe rate high enough to land
            # inside the bootloader's 250ms fallback window
            r = fw.read_reply(timeout=0.12)
            if r and r[2] == ACK_OK:
                info = r[1]
                break
        if info is None:
            raise SystemExit(
                'ESC bootloader did not respond in %.0fs (%u probes). Reset '
                'the ESC while this runs: power cycle it, or reset it over '
                'SWD.' % (args.connect_timeout, probes))
        print('connected after %.1fs (%u probes), device info: %s'
              % (time.time() - t0, probes, info.hex()))
        # byte 2 is the AM32 bootloader revision: the magic addresses
        # arrived in protocol 2 (eeprom) and 3 (devinfo)
        boot_ver = info[2] if len(info) > 2 else 0
        print('bootloader protocol version: %u' % boot_ver)
        if boot_ver < 2:
            raise SystemExit(
                'this bootloader (version %u) has no eeprom magic address, so '
                'the settings cannot be read this way. Dump them with the AM32 '
                'configurator instead and include that with the capture.'
                % boot_ver)
        if boot_ver >= 3:
            d = fw.cmd(CMD_DEVICE_READ, addr=ADDR_MAGIC_DEVINFO,
                       param=bytes([27]))
            if len(d) >= 27:
                print('devinfo: eeprom_start=0x%04x firmware_start=0x%04x '
                      'address_shift=%u'
                      % (struct.unpack('<H', d[23:25])[0],
                         struct.unpack('<H', d[19:21])[0], d[18]))
        eeprom = fw.cmd(CMD_DEVICE_READ, addr=ADDR_MAGIC_EEPROM,
                        param=bytes([EEPROM_SIZE]), timeout=5.0)
        if len(eeprom) != EEPROM_SIZE:
            raise SystemExit('short eeprom read: %u bytes' % len(eeprom))
        with open(args.out, 'wb') as f:
            f.write(eeprom)
        print('wrote %s (%u bytes)' % (args.out, len(eeprom)))
        describe(eeprom)
    finally:
        try:
            fw.send(CMD_DEVICE_RESET, param=bytes([args.motor - 1]))
            time.sleep(0.2)
            fw.send(CMD_INTERFACE_EXIT)
            time.sleep(0.2)
        except Exception:
            pass
        ser.close()
    print('\nthe ESC has been reset out of its bootloader; power cycle it '
          'before capturing again')


# EEprom_t (Inc/eeprom.h): the settings a calibration data set must mirror
SETTINGS = (
    ('version', 3, 4), ('max_ramp', 5), ('minimum_duty_cycle', 6),
    ('absolute_voltage_cutoff', 8), ('current_P', 9), ('current_I', 10),
    ('current_D', 11), ('active_brake_power', 12), ('dir_reversed', 17),
    ('bi_direction', 18), ('use_sine_start', 19), ('comp_pwm', 20),
    ('variable_pwm', 21), ('stuck_rotor_protection', 22), ('advance_level', 23),
    ('pwm_frequency', 24), ('startup_power', 25), ('motor_kv', 26),
    ('motor_poles', 27), ('brake_on_stop', 28), ('stall_protection', 29),
    ('beep_volume', 30), ('telemetry_on_interval', 31), ('low_voltage_cut_off', 36),
    ('low_cell_volt_cutoff', 37), ('rc_car_reverse', 38), ('use_hall_sensors', 39),
    ('sine_mode_changeover', 40), ('drag_brake_strength', 41),
    ('driving_brake_strength', 42), ('temperature_limit', 43),
    ('current_limit', 44), ('sine_mode_power', 45), ('input_type', 46),
    ('auto_advance', 47), ('can_node', 176), ('can_esc_index', 177),
    ('can_require_arming', 178), ('can_telem_rate', 179),
    ('can_require_zero_throttle', 180), ('can_filter_hz', 181),
    ('can_debug_rate', 182),
)


def describe(e):
    '''print the settings a calibration data set needs to mirror'''
    if len(e) < EEPROM_SIZE:
        print('(short eeprom, not decoding)')
        return
    print('\nsettings (eeprom layout version %u, firmware %u.%02u):'
          % (e[1], e[3], e[4]))
    for entry in SETTINGS:
        name, off = entry[0], entry[1]
        if len(entry) > 2:
            print('  %-26s: %u.%02u' % (name, e[off], e[entry[2]]))
        else:
            print('  %-26s: %u' % (name, e[off]))
    # the stored byte is (kv - 20) / 40
    print('  %-26s: %u rpm/V' % ('motor_kv (decoded)', e[26] * 40 + 20))


if __name__ == '__main__':
    main()
