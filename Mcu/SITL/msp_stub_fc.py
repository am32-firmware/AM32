'''
fake Betaflight FC: a minimal MSP server bridged to the SITL's UDP
DShot input

Lets scripts/esc_capture_fc.py run against the SITL binary with no
hardware: the stub answers the MSP preflight queries, streams
bidirectional DShot600 to the SITL from the latest MSP_SET_MOTOR
value, decodes the BDShot/EDT replies and serves them back as
MSP_MOTOR_TELEMETRY, just like a real FC does.

Serves MSP on a pty; run() prints/returns the slave device path to
hand to --port. Linux/macOS only (pty).
'''

import os
import pty
import struct
import threading
import time

import sitl_dshot as sd

MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_FEATURE_CONFIG = 36
MSP_STATUS = 101
MSP_BOXIDS = 119
MSP_MOTOR_CONFIG = 131
MSP_MOTOR_TELEMETRY = 139
MSP_SET_MOTOR = 214


class MspStubFC(object):
    def __init__(self, sitl_host='127.0.0.1', sitl_port=57833,
                 poles=14, rate=500.0):
        self.poles = poles
        self.rate = rate
        # set to stop updating the telemetry while still answering MSP,
        # reproducing Betaflight serving its cached values after the
        # BDShot replies stop arriving (it never marks them stale)
        self.freeze = False
        self.port = sd.InputPort(sitl_host, sitl_port)
        self.motor_value = 1000        # latest MSP_SET_MOTOR, motor 1
        self.rpm = 0
        # raw EDT bytes as Betaflight caches them: voltage in 0.25V
        # steps, current in the ESC's own units (AM32: 0.5A), temp in C
        self.volt_raw = 0
        self.curr_raw = 0
        self.temp = 0
        self.invalid = 100.0           # no telemetry until frames arrive
        self.edt_seen = False
        self.last_edt_cmd = 0.0
        self.running = True
        self.master, self.slave = pty.openpty()
        # raw mode now: the default line discipline would echo the
        # client's bytes back at us until pyserial reconfigures it
        import tty
        tty.setraw(self.slave)
        self.slave_path = os.ttyname(self.slave)
        threading.Thread(target=self._dshot_loop, daemon=True).start()
        threading.Thread(target=self._msp_loop, daemon=True).start()

    def close(self):
        self.running = False
        try:
            os.close(self.master)
            os.close(self.slave)
        except OSError:
            pass
        self.port.close()

    # -- DShot side ----------------------------------------------------

    def _dshot_value(self):
        '''BF motor value 1000..2000 -> 11 bit DShot throttle'''
        v = self.motor_value
        if v <= 1000:
            return 0
        return 48 + int((min(v, 2000) - 1000) * (2047 - 48) / 1000)

    def _dshot_loop(self):
        nxt = time.time()
        while self.running:
            now = time.time()
            burst = 0
            while now >= nxt and burst < 10:
                nxt += 1.0 / self.rate
                value = self._dshot_value()
                # maintain EDT while stopped, like a real FC with
                # dshot_edt on (the firmware ignores commands once
                # spinning)
                if (value == 0 and not self.edt_seen
                        and now - self.last_edt_cmd > 0.5):
                    self.last_edt_cmd = now
                    for _ in range(20):
                        self.port.send_dshot(sd.DSHOT_CMD_EDT_ENABLE,
                                             ptype=sd.TYPE_DSHOT600,
                                             telem=True, bidir=True)
                        burst += 1
                    continue
                self.port.send_dshot(value, ptype=sd.TYPE_DSHOT600,
                                     bidir=True)
                burst += 1
            if now - nxt > 0.25:
                nxt = now
            self._drain_replies()
            time.sleep(0.0005)

    def _drain_replies(self):
        if self.freeze:
            self.port.get_replies()      # discard, keep the cached values
            return
        for r in self.port.get_replies():
            kind, val = sd.decode_reply(r[3], edt_expected=True)
            if kind == 'erpm':
                self.rpm = int(sd.erpm_period_to_rpm(val, self.poles))
                self.invalid = max(0.0, self.invalid - 1.0)
            elif kind == 'temp':
                self.temp = val
                self.edt_seen = True
            elif kind == 'volt':
                self.volt_raw = int(round(val / 0.25))
                self.edt_seen = True
            elif kind == 'current':
                self.curr_raw = int(round(val / 0.5))
                self.edt_seen = True
            elif kind == 'edt':
                self.edt_seen = True
            elif kind == 'badcrc':
                self.invalid = min(100.0, self.invalid + 0.1)

    # -- MSP side ------------------------------------------------------

    def _reply(self, cmd, payload=b''):
        hdr = struct.pack('<BB', len(payload), cmd)
        ck = 0
        for b in hdr + payload:
            ck ^= b
        os.write(self.master, b'$M>' + hdr + payload + bytes([ck]))

    def _handle(self, cmd, payload):
        if cmd == MSP_API_VERSION:
            self._reply(cmd, struct.pack('<BBB', 0, 1, 46))
        elif cmd == MSP_FC_VARIANT:
            self._reply(cmd, b'BTFL')
        elif cmd == MSP_STATUS:
            # cycletime, i2c errors, sensors, mode flags (disarmed), profile
            self._reply(cmd, struct.pack('<HHHIB', 125, 0, 0, 0, 0))
        elif cmd == MSP_BOXIDS:
            self._reply(cmd, bytes([0]))   # one box: ARM
        elif cmd == MSP_FEATURE_CONFIG:
            self._reply(cmd, struct.pack('<I', 0))   # no 3D mode
        elif cmd == MSP_MOTOR_CONFIG:
            self._reply(cmd, struct.pack('<HHHBBBB', 1070, 2000, 1000,
                                         4, self.poles, 1, 0))
        elif cmd == MSP_MOTOR_TELEMETRY:
            out = bytes([4])
            for i in range(4):
                if i == 0:
                    # matches Betaflight's DShot telemetry serialisation:
                    # voltage is the 0.25V-step EDT value >> 2, current
                    # is the raw EDT byte (msp.c MSP_MOTOR_TELEMETRY)
                    out += struct.pack('<IHBHHH', self.rpm,
                                       int(self.invalid * 100), int(self.temp),
                                       self.volt_raw >> 2,
                                       self.curr_raw, 0)
                else:
                    # unused outputs: no telemetry at all
                    out += struct.pack('<IHBHHH', 0, 10000, 0, 0, 0, 0)
            self._reply(cmd, out)
        elif cmd == MSP_SET_MOTOR:
            if len(payload) >= 2:
                self.motor_value = struct.unpack('<H', payload[0:2])[0]
            self._reply(cmd)
        else:
            os.write(self.master, b'$M!' + struct.pack('<BB', 0, cmd)
                     + bytes([cmd]))

    def _msp_loop(self):
        buf = b''
        while self.running:
            try:
                chunk = os.read(self.master, 256)
            except OSError:
                return
            if not chunk:
                continue
            buf += chunk
            while True:
                start = buf.find(b'$M<')
                if start < 0:
                    buf = b''
                    break
                buf = buf[start:]
                if len(buf) < 5:
                    break
                size = buf[3]
                if len(buf) < 6 + size:
                    break
                cmd = buf[4]
                payload = buf[5:5 + size]
                ck = 0
                for b in buf[3:5 + size]:
                    ck ^= b
                good = ck == buf[5 + size]
                buf = buf[6 + size:]
                if good:
                    self._handle(cmd, payload)


if __name__ == '__main__':
    stub = MspStubFC()
    print(stub.slave_path, flush=True)
    while True:
        time.sleep(1)
