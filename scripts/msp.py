'''
minimal MSP v1 client for talking to a Betaflight flight controller
over its USB CDC serial port

only the small command set the calibration capture tools need. A
reader thread parses reply frames as they arrive so callers can
pipeline requests (fire several commands, consume the replies
asynchronously) - Betaflight's MSP task runs at ~100Hz but drains all
buffered frames per invocation, so pipelining is what makes a ~100Hz
command+telemetry loop achievable over USB CDC.

frame format (v1): '$M<' dir + size u8 + cmd u8 + payload + xor
checksum over size/cmd/payload. Replies use '$M>', errors '$M!'.
'''

import struct
import threading
import time

import serial

MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_FC_VERSION = 3
MSP_FEATURE_CONFIG = 36
MSP_STATUS = 101
MSP_BOXIDS = 119
MSP_MOTOR_CONFIG = 131
MSP_MOTOR_TELEMETRY = 139
MSP_SET_MOTOR = 214
MSP_SET_PASSTHROUGH = 245

PERM_ID_ARM = 0        # BOXARM permanent id
FEATURE_3D = 1 << 12   # 3D mode: motor value 1000 means full reverse


def frame(cmd, payload=b''):
    hdr = struct.pack('<BB', len(payload), cmd)
    ck = 0
    for b in hdr + payload:
        ck ^= b
    return b'$M<' + hdr + payload + bytes([ck])


class MspPort(object):
    '''serial MSP connection with a reader thread. Latest reply per
    command is kept with a sequence number so a caller can tell a fresh
    reply from a stale one'''

    def __init__(self, port, baud=115200, callbacks=None):
        # baud is a formality on USB CDC. callbacks: {cmd: fn(payload)},
        # called from the reader thread as each reply arrives, so no
        # reply is lost and handling latency is the read latency
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.lock = threading.Lock()
        self.replies = {}          # cmd -> (seq, payload)
        self.reply_count = 0
        self.error_count = 0       # '$M!' frames (FC rejected the command)
        self.callbacks = callbacks or {}
        self.running = True
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def close(self):
        self.running = False
        try:
            self.ser.close()
        except Exception:
            pass

    def send(self, cmd, payload=b''):
        '''fire and forget; replies arrive via the reader thread'''
        self.ser.write(frame(cmd, payload))

    def get_reply(self, cmd):
        '''(seq, payload) of the latest reply for cmd, or (0, None)'''
        with self.lock:
            return self.replies.get(cmd, (0, None))

    def request(self, cmd, payload=b'', timeout=1.0, retries=3):
        '''round-trip convenience for preflight queries'''
        for _ in range(retries):
            seq0, _old = self.get_reply(cmd)
            self.send(cmd, payload)
            deadline = time.time() + timeout
            while time.time() < deadline:
                seq, data = self.get_reply(cmd)
                if seq != seq0:
                    return data
                time.sleep(0.005)
        raise TimeoutError('no MSP reply for command %u' % cmd)

    def _reader(self):
        buf = b''
        while self.running:
            try:
                # read(1) returns on the first byte; a fixed-size read
                # would sit out the full serial timeout batching replies
                chunk = self.ser.read(1)
                if chunk and self.ser.in_waiting:
                    chunk += self.ser.read(self.ser.in_waiting)
            except Exception:
                return
            if not chunk:
                continue
            buf += chunk
            while True:
                start = buf.find(b'$M')
                if start < 0:
                    # a trailing '$' may be the start of a frame whose
                    # 'M' has not been read yet: keep it
                    buf = buf[-1:] if buf.endswith(b'$') else b''
                    break
                buf = buf[start:]
                if len(buf) < 5:
                    break
                direction = buf[2:3]
                size = buf[3]
                if len(buf) < 5 + size + 1:
                    break
                cmd = buf[4]
                payload = buf[5:5 + size]
                ck = 0
                for b in buf[3:5 + size]:
                    ck ^= b
                good = ck == buf[5 + size]
                buf = buf[5 + size + 1:]
                if not good:
                    continue
                if direction == b'!':
                    self.error_count += 1
                    continue
                if direction != b'>':
                    continue
                with self.lock:
                    seq, _ = self.replies.get(cmd, (0, None))
                    self.replies[cmd] = (seq + 1, payload)
                    self.reply_count += 1
                cb = self.callbacks.get(cmd)
                if cb:
                    try:
                        cb(payload)
                    except Exception as ex:
                        print('MSP callback error: %s' % ex)


def parse_api_version(data):
    '''(msp_protocol, api_major, api_minor)'''
    return struct.unpack('<BBB', data[:3])


def parse_feature_config(data):
    '''enabled feature bitmask'''
    return struct.unpack('<I', data[:4])[0]


def parse_motor_config(data):
    '''dict with min/max throttle and, on API >= 1.42, motor_count,
    motor_poles and the dshot telemetry / esc sensor flags'''
    out = {}
    out['minthrottle'], out['maxthrottle'], out['mincommand'] = \
        struct.unpack('<HHH', data[:6])
    if len(data) >= 10:
        out['motor_count'] = data[6]
        out['motor_poles'] = data[7]
        out['use_dshot_telemetry'] = data[8]
        out['use_esc_sensor'] = data[9]
    return out


def parse_motor_telemetry(data):
    '''list of per-motor dicts: rpm (mechanical), invalid_pct (%),
    temp (C), volt (V), curr (A), consumption (mAh).

    With bidirectional DShot as the source Betaflight forwards the EDT
    bytes nearly raw (verified against BF 4.4/4.5 msp.c): voltage is
    the 0.25V-step EDT value >> 2 (so whole volts), current is the raw
    EDT byte, which AM32 encodes in 0.5A steps'''
    count = data[0]
    out = []
    ofs = 1
    for _ in range(count):
        rpm, invalid, temp, volt, curr, mah = \
            struct.unpack_from('<IHBHHH', data, ofs)
        ofs += 13
        out.append({'rpm': rpm, 'invalid_pct': invalid * 0.01, 'temp': temp,
                    'volt': float(volt), 'curr': curr * 0.5, 'mah': mah})
    return out


def parse_status_armed(status_data, boxids_data):
    '''True when the ARM box is active in MSP_STATUS flight mode flags.
    boxids maps box slots to permanent ids; ARM is permanent id 0'''
    flags = struct.unpack_from('<I', status_data, 6)[0]
    for slot, perm in enumerate(boxids_data):
        if perm == PERM_ID_ARM:
            return bool(flags & (1 << slot))
    return False
