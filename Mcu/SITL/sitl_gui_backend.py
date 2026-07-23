'''
UI-independent backends for the AM32 SITL control GUI: the PWM/DShot
sender, the DroneCAN node and rate counters. These run in their own
threads and expose plain attributes/queues, so they can be driven by any
front end or by headless tests without a display.
'''

import collections
import queue
import socket
import struct
import sys
import threading
import time

import sitl_dshot as sd

try:
    import dronecan
    import dronecan.app.node_monitor
    import dronecan.app.dynamic_node_id
    HAVE_DRONECAN = True

    # pydronecan's NodeMonitor assumes every reply to its GetNodeInfo
    # request is a Response and reads response.status/hardware_version
    # unguarded. With a second node monitor on the same bus (e.g. a
    # dronecan_gui_tool the user also has open - the mcast buses are not
    # isolated on loopback), request/response matching can hand a
    # GetNodeInfo.Request, which has no fields, to the response
    # callback, and the unguarded field access throws a noisy but
    # harmless traceback on every collision. Drop non-Response replies
    # at the top of the handler, before an entry is created, so no
    # half-initialised entry is left to crash the later stale sweep
    _orig_on_info = dronecan.app.node_monitor.NodeMonitor._on_info_response

    def _guarded_on_info(self, e):
        if e and 'status' not in getattr(e.response, '_fields', {}):
            return
        return _orig_on_info(self, e)

    dronecan.app.node_monitor.NodeMonitor._on_info_response = \
        _guarded_on_info

    # the released mcast driver's receive() busy-waits: it polls
    # rx_queue.get(block=0) in a tight loop until the timeout expires, so
    # every node.spin() burns a full core - and through the GIL that
    # starves the DShot sender and waveform threads whose timing the
    # firmware depends on. Replace it with a blocking get; a driver whose
    # receive() no longer busy-polls is left alone
    import inspect
    from dronecan.driver.mcast import mcast as _mcast_cls

    try:
        _busy_poll = 'block=0' in inspect.getsource(_mcast_cls.receive)
    except (OSError, TypeError):
        _busy_poll = False
    if _busy_poll:
        def _blocking_receive(self, timeout=None):
            deadline = None if timeout is None \
                else time.time() + max(timeout, 0.001)
            while True:
                try:
                    if deadline is None:
                        frame = self.rx_queue.get()
                    else:
                        remaining = deadline - time.time()
                        if remaining <= 0:
                            return
                        frame = self.rx_queue.get(block=True,
                                                  timeout=remaining)
                except queue.Empty:
                    if deadline is None:
                        continue
                    return
                self._rx_hook(frame)
                return frame

        _mcast_cls.receive = _blocking_receive
except ImportError:
    HAVE_DRONECAN = False


class RateCounter(object):
    def __init__(self):
        self.count = 0
        self.rate = 0.0
        self.last = time.time()
        self.last_count = 0

    def tick(self, n=1):
        self.count += n

    def hz(self):
        now = time.time()
        dt = now - self.last
        if dt >= 1.0:
            self.rate = (self.count - self.last_count) / dt
            self.last = now
            self.last_count = self.count
        return self.rate


class DshotPanel(object):
    '''PWM/DShot sender thread + state'''

    def __init__(self, host, port):
        self.port = sd.InputPort(host, port)
        self.enabled = False
        self.ptype = sd.TYPE_DSHOT300
        self.bidir = False
        self.telem_bit = False
        self.value = 0          # dshot value or pwm width
        self.rate = 500.0
        self.cmd_queue = queue.Queue()
        self.sent = RateCounter()
        self.replies = RateCounter()
        self.rpm = 0.0
        self.spinning = False
        self.badcrc = 0
        self.edt = {}           # kind -> (value, time received)
        self.edt_want = False
        self.last_edt_seen = 0.0
        self.last_edt_cmd = 0.0
        self.poles = 14
        self.status = ''
        self.running = True
        threading.Thread(target=self._sender, daemon=True).start()

    def _sender(self):
        next_send = time.time()
        while self.running:
            now = time.time()
            if not self.enabled:
                next_send = now
                time.sleep(0.02)
                self._collect()
                continue
            if now < next_send:
                time.sleep(next_send - now)
                now = time.time()
            # catch-up burst: keep the average rate under coarse sleep
            # granularity (VMs, CI runners) - the firmware's bidirectional
            # auto-detect needs >100 frames before arming completes
            burst = 0
            while now >= next_send and burst < 10:
                next_send += 1.0 / max(1.0, self.rate)
                try:
                    cmd = self.cmd_queue.get_nowait()
                except queue.Empty:
                    cmd = None
                if cmd is not None:
                    self.port.send_dshot(cmd, ptype=self.ptype, telem=True, bidir=self.bidir)
                elif self.ptype == sd.TYPE_PWM:
                    self.port.send_pwm(int(self.value))
                else:
                    self.port.send_dshot(int(self.value), ptype=self.ptype,
                                         telem=self.telem_bit, bidir=self.bidir)
                self.sent.tick()
                burst += 1
            if now - next_send > 0.25:
                next_send = now  # fell too far behind, resync
            self._collect()
            self._edt_maintain(now)
        self.port.close()

    def _collect(self):
        for r in self.port.get_replies():
            self.replies.tick()
            kind, val = sd.decode_reply(r[3], edt_expected=True)
            if kind == 'erpm':
                self.spinning = val < 65408
                self.rpm = sd.erpm_period_to_rpm(val, self.poles)
            elif kind == 'badcrc':
                self.badcrc += 1
            else:
                self.last_edt_seen = time.time()
                if kind == 'edt' and val in (0xE00, 0xEFF):
                    # EDT init/deinit acknowledgement frames, not data
                    continue
                self.edt[kind] = (val, time.time())

    def edt_active(self):
        '''true when EDT frames are actually arriving from the ESC'''
        return time.time() - self.last_edt_seen < 3.0

    def edt_fresh(self, max_age=15.0):
        '''EDT values received recently. The age allows for the slow EDT
        schedule at low frame rates (temp/voltage every ~400 replies)'''
        if not self.edt_active():
            return {}
        now = time.time()
        return {k: v for k, (v, t) in self.edt.items() if now - t < max_age}

    def send_command(self, cmd, count=8):
        for _ in range(count):
            self.cmd_queue.put(cmd)

    def _edt_maintain(self, now):
        '''EDT is a maintained state: the firmware only processes DShot
        commands while armed with the motor stopped, silently discarding
        them otherwise, and a reboot clears EDT. Keep (re)sending the
        enable/disable command until the reply stream matches the
        requested state'''
        if self.ptype == sd.TYPE_PWM or not self.bidir:
            return
        active = self.edt_active()
        if self.edt_want == active:
            if self.edt_want and self.status.startswith('EDT'):
                self.status = ''
            return
        if int(self.value) != 0 or self.spinning:
            if self.edt_want:
                self.status = 'EDT pending: needs the motor stopped at zero throttle'
            return
        if now - self.last_edt_cmd > 1.5:
            self.last_edt_cmd = now
            if self.edt_want:
                self.send_command(sd.DSHOT_CMD_EDT_ENABLE)
                self.status = 'EDT enable sent, waiting for EDT frames (arms after >1.5s at zero)'
            else:
                self.send_command(sd.DSHOT_CMD_EDT_DISABLE)
                self.status = 'EDT disable sent'


class CanPanel(object):
    '''DroneCAN node thread: RawCommand/ArmingStatus stream, telemetry
    handlers and parameter set requests'''

    def __init__(self, uri):
        self.uri = uri
        self.enabled = False
        self.dna_server = False
        self.send_rawcommand = True
        self.armed = True
        self.throttle = 0.0     # 0..1
        self.rate = 50.0
        self.esc_index = 0
        self.status = {}
        self.node_id = None
        self.uptime = 0
        self.esc_rate = RateCounter()
        self.sent = RateCounter()
        self.param_queue = queue.Queue()
        self.param_result = queue.Queue()
        self.error = None
        self.running = True
        # set once make_node has spawned the IO child (or failed), so the
        # caller can sequence signal handler setup around the spawn
        self.started = threading.Event()
        self.thread = threading.Thread(target=self._can_thread, daemon=True)
        self.thread.start()

    NODE_ID = 126

    def _can_thread(self):
        try:
            # passive (anonymous) mode: sends nothing at all, telemetry
            # RX still works. The node ID is only assigned while Enable
            # is on, so an idle GUI leaves the bus completely quiet -
            # the bootloader's no-CAN fallback needs a frame-free bus
            node = dronecan.make_node(self.uri, bitrate=1000000)
        except Exception as ex:
            self.error = str(ex)
            self.started.set()
            return
        self.started.set()

        def on_esc_status(e):
            m = e.message
            if m.esc_index != self.esc_index:
                return
            # the ESC is unambiguously the sender of esc.Status; other
            # nodes on the bus also send NodeStatus
            self.node_id = e.transfer.source_node_id
            self.esc_rate.tick()
            self.status = {
                'rpm': m.rpm,
                'voltage': m.voltage,
                'current': m.current,
                'temp': m.temperature - 273.15,
                'errors': m.error_count,
            }

        def on_node_status(e):
            if e.transfer.source_node_id == self.node_id:
                self.uptime = e.message.uptime_sec

        node.add_handler(dronecan.uavcan.equipment.esc.Status, on_esc_status)
        node.add_handler(dronecan.uavcan.protocol.NodeStatus, on_node_status)

        next_send = time.time()
        allocator = None
        monitor = None
        while self.running:
            try:
                node.spin(0.002)
            except Exception:
                pass
            self._handle_param(node)
            # the node only has an ID (and so only sends anything) while
            # commanding or serving DNA
            need_id = self.enabled or self.dna_server
            if need_id and node.is_anonymous:
                node.node_id = self.NODE_ID  # NodeStatus starts here
            elif not need_id and not node.is_anonymous:
                # no public API to return to passive mode; clearing the
                # id silences the 1Hz NodeStatus broadcast again
                node._node_id = None
            if self.dna_server and allocator is None:
                monitor = dronecan.app.node_monitor.NodeMonitor(node)
                allocator = dronecan.app.dynamic_node_id.CentralizedServer(node, monitor)
            elif not self.dna_server and allocator is not None:
                allocator.close()
                monitor.close()
                allocator = monitor = None
            if not self.enabled:
                next_send = time.time()
                continue
            now = time.time()
            if now >= next_send:
                next_send += 1.0 / max(1.0, self.rate)
                status = 255 if self.armed else 0
                node.broadcast(dronecan.uavcan.equipment.safety.ArmingStatus(status=status))
                if self.send_rawcommand:
                    cmds = [0] * (self.esc_index + 1)
                    cmds[self.esc_index] = int(8191 * self.throttle)
                    node.broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=cmds))
                    self.sent.tick()
        # orderly shutdown of the mcast IO child process
        try:
            node.close()
        except Exception:
            pass

    def _handle_param(self, node):
        try:
            name, value = self.param_queue.get_nowait()
        except queue.Empty:
            return
        if self.node_id is None:
            self.param_result.put('no node seen yet')
            return
        if node.is_anonymous:
            self.param_result.put('enable CAN first (GUI node is passive)')
            return
        target = self.node_id
        result = {}

        def cb(e):
            result['rsp'] = e.response if e is not None else None
            result['done'] = True

        def wait(req, timeout=2.0):
            result.clear()
            node.request(req, target, cb)
            deadline = time.time() + timeout
            while 'done' not in result and time.time() < deadline:
                node.spin(0.05)
            return result.get('rsp')

        req = dronecan.uavcan.protocol.param.GetSet.Request()
        req.name = name
        req.value = dronecan.uavcan.protocol.param.Value(integer_value=int(value))
        rsp = wait(req)
        if rsp is None or len(rsp.name) == 0:
            self.param_result.put('%s: set failed' % name)
            return
        req = dronecan.uavcan.protocol.param.ExecuteOpcode.Request()
        req.opcode = req.OPCODE_SAVE
        wait(req)
        req = dronecan.uavcan.protocol.RestartNode.Request()
        req.magic_number = req.MAGIC_NUMBER
        wait(req, timeout=1.0)
        self.param_result.put('%s=%d saved, node %d restarted' % (name, int(value), target))

    def set_param(self, name, value):
        self.param_queue.put((name, value))


class EepromClient(object):
    """read and write the simulated ESC's eeprom directly over the SITL
    state port (cmd 5 fetch / cmd 6 set), bypassing the 4-way and
    DroneCAN parameter paths - the simulator is the bench, so settings
    can be edited the way a technician reflashes them"""

    MAGIC_CMD = 0x5353
    MAGIC_REPLY = 0x5355
    MAGIC_EEPROM = 0x5358

    def __init__(self, host='127.0.0.1', port=57734, timeout=1.5):
        self.addr = (host, port)
        self.timeout = timeout

    def _sock(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(('127.0.0.1', 0))
        s.settimeout(0.4)      # short recv slices; _transact resends
        return s

    def _transact(self, s, pkt):
        """send pkt and yield received datagrams until the timeout,
        resending periodically: an unconfigured ESC reboot-loops on its
        signal timeout, and a single request can vanish into a reboot
        window - exactly when the eeprom editor is most needed"""
        deadline = time.time() + self.timeout
        next_send = 0.0
        while True:
            now = time.time()
            if now >= deadline:
                return
            if now >= next_send:
                try:
                    s.sendto(pkt, self.addr)
                except OSError:
                    return
                next_send = now + 0.4
            try:
                yield s.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                return

    def fetch(self):
        """returns (image bytes, {'kv':, 'poles':}) or (None, None)"""
        s = self._sock()
        try:
            for d in self._transact(s, struct.pack('<HBB', self.MAGIC_CMD, 5, 0)):
                if len(d) < 16:
                    continue
                magic, cmd, _pad, length = struct.unpack('<HBBH', d[:6])
                if magic != self.MAGIC_EEPROM or cmd != 5:
                    continue
                kv, poles = struct.unpack('<fB', d[8:13])
                return d[16:16 + length], {'kv': kv, 'poles': poles}
            return None, None
        finally:
            s.close()

    def set(self, offset, data):
        """write bytes at an eeprom offset; returns (ok, message).
        Retried writes are safe: rewriting the same bytes is idempotent"""
        s = self._sock()
        try:
            pkt = struct.pack('<HBBHH', self.MAGIC_CMD, 6, 0,
                              int(offset), len(data)) + bytes(data)
            for d in self._transact(s, pkt):
                if len(d) < 4:
                    continue
                magic, ok, _pad = struct.unpack('<HBB', d[:4])
                if magic != self.MAGIC_REPLY:
                    continue
                return bool(ok), d[4:].split(b'\0')[0].decode(errors='replace')
            return False, 'no reply from the simulator'
        finally:
            s.close()


class SimStream(object):
    """subscriber for the SITL simulation state stream (--state-port):
    high rate physics samples for graphs/animation, and runtime motor
    model loading. Samples are (t_s, omega, theta, theta_e, iu, iv, iw,
    vu, vv, vw, vbus, ibus, modes, comp_phase, comp_out)"""

    SAMPLE = struct.Struct('<Qfffffffffff3sBB3x')
    MAGIC_CMD = 0x5353
    MAGIC_DATA = 0x5354
    MAGIC_REPLY = 0x5355

    def __init__(self, host='127.0.0.1', port=57734, period_us=50, maxlen=40000):
        self.addr = (host, port)
        self.period_us = period_us
        self.enabled = False
        self.samples = collections.deque(maxlen=maxlen)
        self.lock = threading.Lock()  # guards samples against the reader
        self.rate = RateCounter()
        self.model_status = ''
        self.running = True
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 0))
        self.sock.settimeout(0.2)
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._subscriber, daemon=True).start()

    def _subscriber(self):
        while self.running:
            if self.enabled:
                # averaged sampling at coarse periods, so a slow scope
                # shows the mean over each period instead of aliased
                # point samples of the PWM
                flags = 1 if self.period_us >= 10 else 0
                pkt = struct.pack('<HBBI', self.MAGIC_CMD, 0, flags,
                                  int(round(self.period_us * 1000)))
                try:
                    self.sock.sendto(pkt, self.addr)
                except OSError:
                    pass
            time.sleep(1.0)

    def _reader(self):
        while self.running:
            try:
                d = self.sock.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            if len(d) < 4:
                continue
            magic, b2, b3 = struct.unpack('<HBB', d[:4])
            if magic == self.MAGIC_REPLY:
                self.model_status = d[4:].split(b'\0')[0].decode(errors='replace')
                continue
            if magic != self.MAGIC_DATA or b2 != 2:
                continue
            count = b3
            batch = []
            for k in range(count):
                off = 4 + k * self.SAMPLE.size
                if off + self.SAMPLE.size > len(d):
                    break
                smp = self.SAMPLE.unpack_from(d, off)
                batch.append((smp[0] * 1e-9,) + smp[1:])
            with self.lock:
                self.samples.extend(batch)
            self.rate.tick(len(batch))

    def latest(self):
        with self.lock:
            return self.samples[-1] if self.samples else None

    def window(self, seconds):
        """most recent samples spanning the given time window. Stops at a
        backward time step (a firmware reboot resets sim time to zero), so
        the returned run is contiguous and monotonic - a scope plotting it
        never draws across the reset"""
        # snapshot at C speed and scan outside the lock: latest() serves
        # the waveform thread, so a long python scan under the lock would
        # couple the throttle to scope redraw cost
        with self.lock:
            samples = list(self.samples)
        out = []
        if not samples:
            return out
        t_end = samples[-1][0]
        for smp in reversed(samples):
            if smp[0] > t_end or t_end - smp[0] > seconds:
                break
            out.append(smp)
        out.reverse()
        return out

    def set_speedup(self, speedup):
        pkt = struct.pack('<HBBf', self.MAGIC_CMD, 2, 0, speedup)
        try:
            self.sock.sendto(pkt, self.addr)
        except OSError:
            pass

    def set_stuck(self, fraction):
        """stuck rotor fraction 0..1 (prop blocked by an obstruction,
        e.g. a tree branch): 0 is free, 1.0 locks the rotor rigidly"""
        pkt = struct.pack('<HBBf', self.MAGIC_CMD, 7, 0, fraction)
        try:
            self.sock.sendto(pkt, self.addr)
        except OSError:
            pass

    def load_model(self, path):
        self.model_status = 'loading %s ...' % path
        pkt = struct.pack('<HBB', self.MAGIC_CMD, 1, 0) + path.encode()
        try:
            self.sock.sendto(pkt, self.addr)
        except OSError as ex:
            self.model_status = str(ex)

    def close(self):
        self.running = False
        self.sock.close()


class AudioStream(object):
    """subscriber for the SITL physics audio stream (--state-port cmd 4):
    what the motor radiates acoustically (torque ripple plus phase
    current magnitude, high pass filtered), sampled at 48kHz of
    simulated time. Pitch scales with speedup, as slow motion should
    sound. Batches arrive as (t0_ns, samples) tuples"""

    MAGIC_CMD = 0x5353
    MAGIC_AUDIO = 0x5357

    def __init__(self, host='127.0.0.1', port=57734, maxlen=2048):
        self.addr = (host, port)
        self.enabled = True
        self.batches = collections.deque(maxlen=maxlen)
        self.lock = threading.Lock()  # guards batches against the reader
        self.rate = RateCounter()  # arriving samples/s, wall clock
        self.last_rx = 0.0
        self.running = True
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 0))
        self.sock.settimeout(0.2)
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._subscriber, daemon=True).start()

    def _subscriber(self):
        # resubscribe quickly while stale so streams resume promptly
        # after an emulated reset (re-exec drops the subscriber list)
        pkt = struct.pack('<HBB', self.MAGIC_CMD, 4, 0)
        while self.running:
            if self.enabled:
                try:
                    self.sock.sendto(pkt, self.addr)
                except OSError:
                    pass
            time.sleep(0.1 if self.stale() else 0.5)

    def _reader(self):
        while self.running:
            try:
                d = self.sock.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            if len(d) < 16:
                continue
            magic, ver, count, t0, period = struct.unpack_from('<HBBQI', d)
            if magic != self.MAGIC_AUDIO or ver != 1 \
                    or len(d) < 16 + 4 * count:
                continue
            vals = struct.unpack_from('<%uf' % count, d, 16)
            self.last_rx = time.time()
            with self.lock:
                self.batches.append((t0, vals))
            self.rate.tick(count)

    def take_batches(self):
        """drain and return accumulated (t0_ns, samples) batches"""
        with self.lock:
            out = list(self.batches)
            self.batches.clear()
        return out

    def stale(self):
        return time.time() - self.last_rx > 0.5

    def close(self):
        self.running = False
        self.sock.close()


class CanFrameCounter(object):
    """counts CAN frames on a multicast UDP bus (mcast:N), straight off
    the UDP group and independent of the DroneCAN node: it sees every
    node's traffic even with CAN control disabled and works without the
    dronecan package. One datagram with a valid header is one CAN
    frame"""

    MAGIC = 0x2934  # mcast transport header magic
    PORT = 57732

    def __init__(self, uri):
        self.rate = RateCounter()
        self.available = False
        self.error = ''
        self.last_anon = 0.0
        self.running = True
        parts = uri.split(':')
        if parts[0] != 'mcast':
            self.error = 'not a mcast bus'
            return
        bus = int(parts[1]) if len(parts) > 1 and parts[1] else 0
        group = '239.65.82.%u' % bus
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # same scheme as the pydronecan mcast driver: windows cannot
            # bind the group address
            if sys.platform == 'win32':
                sock.bind(('0.0.0.0', self.PORT))
            else:
                sock.bind((group, self.PORT))
            mreq = struct.pack('4sl', socket.inet_aton(group), socket.INADDR_ANY)
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        except OSError as ex:
            self.error = str(ex)
            return
        sock.settimeout(0.2)
        self.sock = sock
        self.available = True
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        while self.running:
            try:
                d = self.sock.recv(128)
            except socket.timeout:
                continue
            except OSError:
                return
            if len(d) >= 10 and struct.unpack_from('<H', d)[0] == self.MAGIC:
                self.rate.tick()
                (mid,) = struct.unpack_from('<I', d, 6)
                if (mid & 0x7f) == 0:
                    # anonymous frame: a DNA allocation request
                    self.last_anon = time.time()

    def dna_request_seen(self):
        """an anonymous (DNA allocation request) frame arrived recently:
        some node on the bus is waiting for a node ID allocator"""
        return time.time() - self.last_anon < 1.5

    def close(self):
        self.running = False
        if self.available:
            self.sock.close()


class ToneStream(object):
    """subscriber for the SITL tone event stream (--state-port cmd 3).
    Beeps the firmware plays through the motor windings arrive as
    (freq_hz, amplitude) change events with simulated timestamps;
    amplitude is the raw PWM duty fraction (~0.002 typical), 0 is
    silence. Keepalive repeats are deduplicated and only refresh
    staleness."""

    MAGIC_CMD = 0x5353
    MAGIC_TONE = 0x5356
    EVENT = struct.Struct('<HBBQff')

    def __init__(self, host='127.0.0.1', port=57734, maxlen=4096):
        self.addr = (host, port)
        self.enabled = True
        self.events = collections.deque(maxlen=maxlen)  # (wall_t, t_ns, freq, amp, source)
        self.lock = threading.Lock()  # guards events against the reader
        self.last_rx = 0.0
        self.running = True
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 0))
        self.sock.settimeout(0.2)
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._subscriber, daemon=True).start()

    def _subscriber(self):
        # subscribe in a fast burst while the stream is stale, so the
        # first notes of a boot tune are caught even after an emulated
        # reset (which re-execs the SITL, dropping its subscriber list)
        pkt = struct.pack('<HBB', self.MAGIC_CMD, 3, 0)
        while self.running:
            if self.enabled:
                try:
                    self.sock.sendto(pkt, self.addr)
                except OSError:
                    pass
            time.sleep(0.02 if self.stale() else 0.5)

    def _reader(self):
        while self.running:
            try:
                d = self.sock.recv(64)
            except socket.timeout:
                continue
            except OSError:
                return
            if len(d) != self.EVENT.size:
                continue
            magic, ver, source, t_ns, freq, amp = self.EVENT.unpack(d)
            if magic != self.MAGIC_TONE or ver != 1:
                continue
            self.last_rx = time.time()
            with self.lock:
                if self.events and self.events[-1][2] == freq \
                        and self.events[-1][3] == amp:
                    continue  # keepalive repeat
                self.events.append((self.last_rx, t_ns, freq, amp, source))

    def stale(self):
        """the SITL keepalive is 20Hz, so a long gap means the SITL is
        gone, rebooting or we are not subscribed"""
        return time.time() - self.last_rx > 0.3

    def current(self):
        """current (freq_hz, amplitude), silent when stale"""
        if self.stale():
            return (0.0, 0.0)
        with self.lock:
            if not self.events:
                return (0.0, 0.0)
            e = self.events[-1]
        return (e[2], e[3])

    def take_events(self):
        """drain and return accumulated events"""
        with self.lock:
            out = list(self.events)
            self.events.clear()
        return out

    def close(self):
        self.running = False
        self.sock.close()
