'''
the work behind the calibration capture GUI: talking to the ESC over
either transport, running the capture battery, fitting the result and
writing the data set out

Kept apart from the UI so it can be driven headlessly and tested.
'''

import contextlib
import io
import json
import os
import re
import threading
import time
import types

def _import_mavlink_safely():
    """import pymavlink without letting a user's ~/.pymavlink/mavextra.py
    break us.

    pymavlink imports $HOME/.pymavlink/mavextra.py if it exists, and the
    stock one imports MAVProxy, which a packaged build has no reason to
    carry. Importing with HOME pointed at an empty directory skips that
    file; the module is cached afterwards, so later imports are normal.
    """
    import tempfile
    old = os.environ.get('HOME')
    tmp = tempfile.mkdtemp(prefix='am32-mavlink-')
    try:
        os.environ['HOME'] = tmp
        import pymavlink.mavutil        # noqa: F401
    finally:
        if old is None:
            os.environ.pop('HOME', None)
        else:
            os.environ['HOME'] = old
        try:
            os.rmdir(tmp)
        except OSError:
            pass


# dronecan decides whether mavcan is usable when its driver package is
# first imported, so this has to happen before anything pulls dronecan
# in - which esc_measure does at import time
try:
    _import_mavlink_safely()
except Exception:
    pass

import esc_analyze
import esc_chirp
import esc_measure
import esc_settings
import esc_square

# settings last: reading them over 4-way reboots the ESC into its
# bootloader, and the DShot reply path takes a moment to come back, so
# a capture running straight afterwards can find no telemetry
STEPS = [
    ('first_spin', 'First spin - 6s at 10% throttle'),
    ('sweep', 'Steady sweep - 0.1 to 0.5, 4s per level'),
    ('square', 'Square wave - step response timing'),
    ('chirp', 'Chirp - frequency response'),
    ('settings', 'ESC settings (eeprom / parameters)'),
]

LOG_NAMES = {
    'first_spin': 'first_spin_010.jsonl',
    'sweep': 'sweep1.jsonl',
    'square': 'square1.jsonl',
    'chirp': 'chirp_120s.jsonl',
}


def _param_value(value, kind):
    """a DroneCAN parameter as JSON. The union member says what the bytes
    mean, so nothing has to be guessed from their content: string_value
    is text by definition, anything else array-shaped stays a byte list
    rather than being mangled into a string"""
    if not kind or kind == 'empty':
        return None
    raw = getattr(value, kind)
    if isinstance(raw, (bool, int, float, str)):
        return raw
    try:
        items = [int(x) & 0xFF for x in raw]
    except TypeError:
        return str(raw)
    if kind == 'string_value':
        head = bytes(bytearray(items)).split(b'\x00')[0]
        if (all(32 <= c < 127 for c in head)
                and all(c == 0 for c in items[len(head):])):
            return head.decode('ascii')
        # not text: AM32's STARTUP_TUNE is raw tune bytes, and 0xFF
        # when unset. Keep them exactly, tagged so a reader cannot
        # mistake the result for a string
        return {'hex': bytes(bytearray(items)).hex()}
    return items


def serial_candidates():
    """(label, device) for every serial port, best name first.

    On Linux the /dev/serial/by-id names are preferred because they are
    stable across reboots and say what the device is, which is what the
    DroneCAN GUI tool shows. On Windows there are no such names, so the
    COM port is labelled with its description from the port properties.
    """
    import glob
    out = []
    seen = set()
    for link in sorted(glob.glob('/dev/serial/by-id/*')):
        try:
            real = os.path.realpath(link)
        except OSError:
            real = link
        out.append((os.path.basename(link), link))
        seen.add(real)
    try:
        import serial.tools.list_ports as list_ports
        for p in list_ports.comports():
            if os.path.realpath(p.device) in seen:
                continue        # already listed under its by-id name
            if p.vid is None and p.device.startswith('/dev/ttyS'):
                continue        # motherboard serial ports, never an ESC link
            desc = (p.description or p.product or '').strip()
            label = p.device
            if desc and desc.lower() != 'n/a':
                label = '%s - %s' % (p.device, desc)
            out.append((label, p.device))
            seen.add(os.path.realpath(p.device))
    except Exception:
        pass
    return out


def can_uri_candidates():
    """URIs to offer for DroneCAN, in the order most people want them"""
    out = []
    for label, dev in serial_candidates():
        out.append(('mavcan: %s' % label, 'mavcan:' + dev))
    out.append(('mcast:0 (local multicast / SITL)', 'mcast:0'))
    return out


class Config(object):
    def __init__(self, transport, port, uri, motor, node_id, poles, directory,
                 max_current, max_throttle, chirp_duration, meta):
        self.transport = transport
        self.port = port
        self.uri = uri
        self.motor = motor
        self.node_id = node_id
        self.poles = poles
        self.directory = directory
        self.max_current = max_current
        self.max_throttle = max_throttle
        self.chirp_duration = chirp_duration
        self.meta = meta


def _args(cfg, **extra):
    '''build the namespace the shared profile functions expect'''
    a = types.SimpleNamespace(
        uri=cfg.uri, node_id=cfg.node_id, client_node_id=100, esc_index=0,
        port=cfg.port, motor=cfg.motor, poles=cfg.poles,
        rate=200.0, log=None, levels='0.1,0.2,0.3,0.4,0.5', throttle=0.1,
        hold=4.0, coast=5.0, cycles=3, max_throttle=cfg.max_throttle,
        max_current=cfg.max_current, max_temp=80.0, ready_timeout=15.0,
        arm_time=3.0, countdown=0.0, telem_rate=None, debug_rate=None,
        sim_state=None, param=None,
        stall_timeout=0.5, status_timeout=3.0, freeze_timeout=5.0,
        throttle_mid=0.35, throttle_amp=0.15, f_start=0.5, f_stop=40.0,
        duration=cfg.chirp_duration, fade_in=3.0, fade_out=1.0,
        deltas='0.03,0.06,0.09,0.12,0.15,0.18,0.21,0.24',
        settle_tol=0.01, settle_window=0.4, settle_min=0.6, settle_max=5.0)
    for k, v in extra.items():
        setattr(a, k, v)
    return a


class Session(object):
    def __init__(self, cfg, log_fn, prompt_fn=None):
        self.cfg = cfg
        self.log = log_fn
        self.prompt = prompt_fn or (lambda t, m: None)
        self.connected = False
        self.aborting = False
        self.backend = None
        # abort() runs on the UI thread while the capture runs on a
        # worker; the lock keeps a late abort from re-arming the backend
        # after the shutdown has already cleared it
        self._lock = threading.Lock()
        self.captured = {}
        self.settings = None
        self.restore_actions = []

    # ------------------------------------------------------------ connect

    def connect(self):
        self.aborting = False
        if not (self.cfg.directory or '').strip():
            raise RuntimeError('choose an output directory first - the '
                               'captures and the settings are written there')
        os.makedirs(self.cfg.directory, exist_ok=True)
        if self.cfg.transport == 'betaflight':
            return self._connect_bf()
        return self._connect_can()

    def _connect_bf(self):
        import msp
        port = msp.find_fc_port((self.cfg.port or '').strip() or None)
        self.cfg.port = port
        # Betaflight answers one MSP request per serial task tick, so the
        # stock rate caps telemetry near 50Hz. Raised here and put back
        # by restore()
        self.set_fc_rate()
        port = self.cfg.port
        p = msp.MspPort(port)
        try:
            variant = p.request(msp.MSP_FC_VARIANT).decode('ascii', 'replace')
            api = msp.parse_api_version(p.request(msp.MSP_API_VERSION))
            cfgm = msp.parse_motor_config(p.request(msp.MSP_MOTOR_CONFIG))
            if not cfgm.get('use_dshot_telemetry'):
                raise RuntimeError('bidirectional DShot is off on the FC: '
                                   'set dshot_bidir = ON, then save')
            if cfgm.get('motor_poles') != self.cfg.poles:
                raise RuntimeError('FC motor_poles is %s but %u was entered; '
                                   'they must agree or every rpm is wrong'
                                   % (cfgm.get('motor_poles'), self.cfg.poles))
            d = p.request(msp.MSP_MOTOR_TELEMETRY)
            t = msp.parse_motor_telemetry(d)[self.cfg.motor - 1]
            live = t['invalid_pct'] < 99
            try:
                bat = msp.parse_battery_state(
                    p.request(msp.MSP_BATTERY_STATE))
            except Exception:
                bat = {}
        finally:
            p.close()
        self.connected = True
        msg = '%s API %u.%u on %s, motor %u %s' % (
            variant, api[1], api[2], port, self.cfg.motor,
            'BDShot rpm replies OK' if live else 'NO BDShot telemetry')
        self.log(msg)
        if not live:
            self.log('the ESC is not returning DShot telemetry - check it is '
                     'powered and wired to that motor output')
        if bat and not bat['present']:
            self.log('WARNING: no battery detected (%.2fV). An ESC powered '
                     'only from the FC will talk BDShot and read its '
                     'settings, but the motor cannot turn' % bat['volts'])
        elif bat:
            self.log('battery %.2fV, %u cells' % (bat['volts'], bat['cells']))
        return msg

    def _make_can_node(self, node_id=100):
        """dronecan Node objects sit in reference cycles, so a previous
        driver can still be alive when the next one forks its IO child.
        The child inherits it and its destructor then asserts on exit
        ("can only test a child process"). Collect before forking"""
        import gc
        import dronecan
        gc.collect()
        return dronecan.make_node(self.cfg.uri, node_id=node_id,
                                  bitrate=1000000)

    @staticmethod
    def _check_can_device(uri):
        """mavcan and slcan open the port inside a child process, where
        the failure is invisible - in a windowed build it is not even
        logged. Fail here, where the message reaches the user"""
        scheme, _, dev = uri.partition(':')
        if scheme not in ('mavcan', 'slcan') or not dev:
            return
        dev = dev.split(',')[0]                 # optional baud suffix
        if dev.startswith(('udp', 'tcp', 'mcast')):
            return
        import serial
        try:
            serial.Serial(dev).close()
            return
        except Exception as ex:
            text = str(ex)
        if 'FileNotFound' in text or 'No such file' in text:
            why = 'there is no such port'
        elif 'Access is denied' in text or 'Permission' in text:
            why = 'it is already open in another program'
        else:
            why = text
        raise RuntimeError('cannot open %s: %s.\nPress "Refresh ports" and '
                           'pick the adapter from the list' % (dev, why))

    def _connect_can(self):
        uri = (self.cfg.uri or '').strip()
        if not uri or uri.rstrip(':') in ('mavcan', 'slcan'):
            raise RuntimeError('choose a CAN interface first - pick an entry '
                               'from the CAN URI list (or mcast:0 for SITL)')
        self.cfg.uri = uri
        self._check_can_device(uri)
        if self.cfg.uri.startswith('mavcan:'):
            _import_mavlink_safely()
        import dronecan
        node = self._make_can_node()
        seen = {}

        def on_status(e):
            seen[e.transfer.source_node_id] = e.message.mode
        node.add_handler(dronecan.uavcan.protocol.NodeStatus, on_status)
        # mavcan runs its IO in a child process, and in a packaged build
        # that child is a fresh copy of the whole binary, so the link can
        # take several seconds before any traffic appears. Wait until a
        # node answers rather than for a fixed window
        limit = 15.0 if self.cfg.uri.startswith('mavcan:') else 5.0
        t0 = time.time()
        while time.time() - t0 < limit:
            try:
                node.spin(0.1)
            except Exception:
                pass
            if seen and time.time() - t0 > 2.0:
                break
        node.close()
        # the driver's threads and the serial port take a moment to let
        # go; opening the capture node too soon leaves the first link
        # half-alive and the transmit queue backs up (queue.Full)
        time.sleep(2.0)
        self.connected = True
        msg = 'DroneCAN %s, nodes seen: %s' % (
            self.cfg.uri, ', '.join(str(n) for n in sorted(seen)) or 'none')
        self.log(msg)
        if self.cfg.node_id not in seen:
            self.log('note: node %u was not seen; it may be parked in its '
                     'bootloader until a command stream starts (normal)'
                     % self.cfg.node_id)
        return msg

    def abort(self):
        '''stop the capture in flight, not just at the next step boundary'''
        self.aborting = True
        with self._lock:
            if self.backend is not None:
                self.backend.aborted = 'aborted by user'

    # ------------------------------------------------------------- capture

    def _backend(self, args):
        if self.cfg.transport == 'dronecan' and self.cfg.uri.startswith('mavcan:'):
            _import_mavlink_safely()
        if self.cfg.transport == 'betaflight':
            import esc_capture_fc
            return esc_capture_fc.BetaflightBackend(args)
        m = esc_measure.EscMeasure(args)
        # put the ESC's telemetry rates back the way we found them
        if not any(a[0] == 'can_rates' for a in self.restore_actions):
            self.restore_actions.append(('can_rates', None))
        return m

    def run_step(self, key):
        if key == 'settings':
            return self.read_settings()
        path = os.path.join(self.cfg.directory, LOG_NAMES[key])
        args = _args(self.cfg, log=path)
        if self.cfg.transport == 'dronecan':
            args.telem_rate, args.debug_rate = 200, 100
        self.log('--- %s -> %s' % (key, os.path.basename(path)))
        out = io.StringIO()
        m = None
        try:
            with contextlib.redirect_stdout(out):
                m = self._backend(args)
                with self._lock:
                    self.backend = m
                    # an abort during construction had nothing to set
                    if self.aborting:
                        m.aborted = 'aborted by user'
                if key == 'first_spin':
                    args.throttle, args.hold = 0.1, 6.0
                    esc_measure.profile_hold(m, args)
                elif key == 'sweep':
                    esc_measure.profile_sweep(m, args)
                elif key == 'square':
                    esc_square.run_profile(
                        m, args, [float(x) for x in args.deltas.split(',')])
                elif key == 'chirp':
                    m.wait_ready()
                    esc_chirp.run_chirp(m, args)
                else:
                    raise ValueError('unknown step %s' % key)
        finally:
            with self._lock:
                self.backend = None
                if m is not None:
                    m.aborted = None
            text = out.getvalue()
            if text:
                self.log(text.rstrip())
            if m is not None:
                # the zero-throttle stream must not be skipped, so the
                # close happens whatever the wind-down raises
                try:
                    if hasattr(m, 'shutdown'):
                        m.shutdown()
                    else:
                        m.throttle = 0.0
                        m.spin_for(0.3)
                except BaseException as ex:
                    self.log('WARNING: could not confirm the motor was '
                             'commanded to stop (%s) - CUT POWER TO THE ESC'
                             % ex)
                # close both before reporting: a failed log flush must
                # not leak the CAN node and break the next connect
                err = None
                for name in ('rec', 'node'):
                    obj = getattr(m, name, None)
                    if obj is not None:
                        try:
                            obj.close()
                        except BaseException as ex:
                            err = err or ex
                if err is not None and not self.aborting:
                    raise err
        self.captured[key] = path
        self.log('%s complete' % key)
        return path

    def _probe_bootloader(self, fw, limit):
        import esc_eeprom_fc
        t0 = time.time()
        while time.time() - t0 < limit and not self.aborting:
            fw.send(esc_eeprom_fc.CMD_DEVICE_INIT_FLASH,
                    param=bytes([self.cfg.motor - 1]))
            r = fw.read_reply(timeout=0.12)
            if r and r[2] == esc_eeprom_fc.ACK_OK:
                return r[1]
        return None

    def _bf_replying(self):
        """is the ESC answering BDShot again after the 4-way reset"""
        import msp
        try:
            p = msp.MspPort(self.cfg.port)
            try:
                t = msp.parse_motor_telemetry(
                    p.request(msp.MSP_MOTOR_TELEMETRY))[self.cfg.motor - 1]
            finally:
                p.close()
        except Exception:
            return False
        return t['invalid_pct'] < 99

    def _wait_bf_replying(self, limit):
        t0 = time.time()
        while time.time() - t0 < limit and not self.aborting:
            if self._bf_replying():
                return True
            time.sleep(1.0)
        return False

    def read_settings(self):
        '''raw eeprom plus the values as the config tools display them'''
        if self.cfg.transport == 'betaflight':
            import esc_eeprom_fc
            ser, count = esc_eeprom_fc.enter_passthrough(self.cfg.port)
            fw = esc_eeprom_fc.FourWay(ser)
            try:
                time.sleep(0.3)
                try:
                    fw.cmd(esc_eeprom_fc.CMD_INTERFACE_TEST_ALIVE)
                except IOError:
                    fw.resync()
                    fw.cmd(esc_eeprom_fc.CMD_INTERFACE_TEST_ALIVE)
                info = self._probe_bootloader(fw, 5.0)
                if info is None:
                    # entering passthrough normally reboots the ESC into
                    # its bootloader on its own; a power cycle is only
                    # worth asking for once that has not happened
                    self.prompt('Power cycle the ESC',
                                'The ESC did not answer in its bootloader.\n\n'
                                'Power cycle it now, then press OK - the tool '
                                'keeps probing and picks it up when it '
                                'reboots.')
                    info = self._probe_bootloader(fw, 60.0)
                if info is None:
                    raise RuntimeError('the ESC bootloader never answered')
                ee = fw.cmd(esc_eeprom_fc.CMD_DEVICE_READ,
                            addr=esc_eeprom_fc.ADDR_MAGIC_EEPROM,
                            param=bytes([esc_settings.EEPROM_SIZE]), timeout=5.0)
            finally:
                try:
                    fw.send(esc_eeprom_fc.CMD_DEVICE_RESET,
                            param=bytes([self.cfg.motor - 1]))
                    time.sleep(0.2)
                    fw.send(esc_eeprom_fc.CMD_INTERFACE_EXIT)
                    time.sleep(0.2)
                except Exception:
                    pass
                ser.close()
            self._save_settings(ee)
            # it was just rebooted out of its bootloader, so give the
            # reply path a chance before declaring it dead
            if not self._wait_bf_replying(10.0):
                self.prompt('Power cycle the ESC',
                            'The ESC is not answering DShot after leaving its '
                            'bootloader.\n\nPower cycle it, then press OK.')
            else:
                self.log('ESC is answering DShot again')
        else:
            self.settings = self._read_can_params()
            self._write_settings_json()
        return self.settings

    def _read_can_params(self):
        '''over DroneCAN only the computed values exist, not the raw image'''
        import dronecan
        node = self._make_can_node()
        values = {}
        unanswered = None
        try:
            for index in range(0, 64):
                got = {}

                def cb(e, _i=index):
                    if e is None:
                        return
                    got['answered'] = True
                    if not e.response.name:
                        return          # an empty name marks the end
                    v = e.response.value
                    got['n'] = str(e.response.name)
                    got['v'] = _param_value(
                        v, dronecan.get_active_union_field(v))

                # a dropped frame must not look like the end of the
                # list: that silently truncates the settings record
                for _try in range(3):
                    got.clear()
                    node.request(
                        dronecan.uavcan.protocol.param.GetSet.Request(
                            index=index), self.cfg.node_id, cb)
                    t0 = time.time()
                    while 'answered' not in got and time.time() - t0 < 1.0:
                        try:
                            node.spin(0.05)
                        except Exception:
                            pass
                    if 'answered' in got:
                        break
                if 'answered' not in got:
                    unanswered = index
                    break
                if 'n' not in got:
                    break
                values[got['n']] = got['v']
        finally:
            node.close()
        if unanswered is not None:
            self.log('WARNING: no answer for parameter %u after 3 tries - '
                     'the settings record is incomplete' % unanswered)
        self.log('read %u parameters over DroneCAN' % len(values))
        return {'settings': values, 'source': 'dronecan'}

    def _save_settings(self, eeprom):
        d = self.cfg.directory
        with open(os.path.join(d, 'sitl_eeprom.bin'), 'wb') as f:
            f.write(eeprom)
        self.settings = esc_settings.as_record(eeprom)
        self.settings['source'] = '4way'
        self._write_settings_json()
        self.log('ESC settings:\n' + esc_settings.format_text(eeprom))

    def _write_settings_json(self):
        with open(os.path.join(self.cfg.directory, 'esc_settings.json'), 'w') as f:
            json.dump(self.settings, f, indent=2, sort_keys=True)

    # -------------------------------------------------------------- analyse

    def _find(self, key):
        name = LOG_NAMES.get(key)
        if not name:
            return None
        p = self.captured.get(key) or os.path.join(self.cfg.directory, name)
        return p if os.path.isfile(p) else None

    def plot_series(self):
        out = []
        for key, _label in STEPS:
            p = self._find(key)
            if not p:
                continue
            xs, ys = [], []
            for line in open(p):
                r = json.loads(line)
                if r['type'] == 'status':
                    xs.append(r['t'])
                    ys.append(r['rpm'])
            if xs:
                out.append((key, xs, ys))
        return out

    def fit(self):
        res = {}
        p = self._find('sweep')
        if p:
            rows = [json.loads(l) for l in open(p)]
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                segs = esc_analyze.steady(rows)
            self.log(out.getvalue().rstrip())
            # steady() returns {throttle: {'rpm':.., 'volt':.., ...}}
            levels = {}
            for thr, vals in (segs or {}).items():
                if float(thr) > 0:
                    levels['%.2f' % float(thr)] = round(vals['rpm'])
            res['steady'] = levels
        p = self._find('square')
        if p:
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                trans = esc_square.analyze_log(p)
            down = {}
            for t in trans or []:
                if not t.get('up') and not t.get('desync'):
                    down.setdefault('%.3f' % t['delta'], []).append(t['t1090'] * 1000.0)
            res['square_down_ms'] = {k: round(sum(v) / len(v))
                                     for k, v in sorted(down.items())}
        p = self._find('chirp')
        if p:
            # numbers straight from the fitter - no matplotlib needed, so
            # the packaged build stays small
            fr = esc_chirp.fit_numbers(p, max_freq=30.0, progress=self.log)
            res['chirp'] = {'model': fr['model'],
                            'accel_3db_hz': round(fr['accel_3db_hz'], 2)
                            if fr['accel_3db_hz'] else None,
                            'brake_3db_hz': round(fr['brake_3db_hz'], 2)
                            if fr['brake_3db_hz'] else None,
                            'tau_up_ms': fr['tau_up_ms'],
                            'tau_dn_ms': fr['tau_dn_ms'],
                            'empirical': fr['empirical']}
        return res

    # --------------------------------------------------------------- export

    def readme_text(self, results):
        m = self.cfg.meta
        lines = ['# %s calibration data set' % (m.get('esc') or 'ESC'), '']
        lines.append('Captured %s with the AM32 SITL calibration capture tool '
                     'over %s.' % (time.strftime('%Y-%m-%d'),
                                   'Betaflight DShot' if self.cfg.transport ==
                                   'betaflight' else 'DroneCAN'))
        lines += ['', '## Rig', '']
        for key, label in (('esc', 'ESC'), ('motor', 'Motor'),
                           ('prop', 'Propeller'), ('power', 'Power source'),
                           ('fc', 'Flight controller')):
            lines.append('- %s: %s' % (label, m.get(key) or '(not given)'))
        lines.append('- Motor poles: %u' % self.cfg.poles)
        if m.get('notes'):
            lines += ['', m['notes']]
        lines += ['', '## Captures', '']
        for key, label in STEPS:
            p = self._find(key)
            if key == 'settings':
                continue
            lines.append('- `%s` - %s%s' % (LOG_NAMES[key], label,
                                            '' if p else '  (not captured)'))
        if self.settings:
            lines += ['', '## ESC settings', '',
                      'Raw eeprom in `sitl_eeprom.bin`; the values as the '
                      'configurator and DroneCAN GUI show them are in '
                      '`esc_settings.json`.', '']
            s = self.settings.get('settings', {})
            for k in ('motor_kv', 'motor_poles', 'timing_advance',
                      'pwm_frequency', 'current_limit', 'can_telem_rate'):
                if k in s:
                    lines.append('- %s: %s' % (k, s[k]))
        if results:
            lines += ['', '## Measured', '']
            if results.get('steady'):
                lines.append('Steady rpm by throttle:')
                lines.append('')
                lines.append('| throttle | rpm |')
                lines.append('|---|---|')
                for k in sorted(results['steady']):
                    lines.append('| %s | %s |' % (k, results['steady'][k]))
                lines.append('')
            if results.get('square_down_ms'):
                lines.append('Square down-step times (ms): %s' %
                             ', '.join('%s: %s' % kv for kv in
                                       sorted(results['square_down_ms'].items())))
                lines.append('')
            if results.get('chirp'):
                c = results['chirp']
                lines.append('Chirp 3 dB: accel %.2f Hz, brake %.2f Hz '
                             '(%s model)' % (c['accel_3db_hz'],
                                             c['brake_3db_hz'], c['model']))
        return '\n'.join(lines) + '\n'

    def expected_json(self, results):
        out = {'comment': 'reference values measured from this data set'}
        if results.get('steady'):
            out['steady'] = {'levels': results['steady'], 'tolerance_pct': 5.0}
        if results.get('square_down_ms'):
            out['square'] = {'down_ms': results['square_down_ms'],
                             'down_tolerance_ms': 30, 'down_tolerance_pct': 25}
        if results.get('chirp'):
            out['chirp'] = {'accel_3db_hz': results['chirp']['accel_3db_hz'],
                            'brake_3db_hz': results['chirp']['brake_3db_hz'],
                            'tolerance_pct': 25}
        out['desync_budget'] = 25
        return out

    def model_json(self, results):
        '''a starting SITL model seeded from what was measured. The
        remaining terms need the iteration in scripts/CALIBRATION.md'''
        kv = 2000.0
        volts = 12.0
        if self.settings:
            s = self.settings.get('settings', {})
            kv = float(s.get('motor_kv', kv) or kv)
        # effective kv from the steady sweep is a better starting point
        if results.get('steady'):
            pts = [(float(k), v) for k, v in results['steady'].items()]
            pts = [p for p in pts if p[0] > 0]
            if pts:
                thr, rpm = max(pts)
                kv = round(rpm / max(thr * volts, 1e-6), 1)
        return {
            'motor': {'kv': kv, 'poles': self.cfg.poles, 'resistance': 0.4,
                      'inductance': 2e-05, 'inertia': 1.75e-07,
                      'damping': 9e-08, 'static_friction': 2e-05,
                      'load_k_omega2': 1.2e-10},
            'battery': {'voltage': volts, 'resistance': 0.15,
                        'capacitance': 0.002, 'sink_resistance': 15.0,
                        'sink_current_max': 0.045},
            'esc': {'rds_on': 0.01, 'diode_vf': 0.7, 'temperature_c': 25.0,
                    'commutation_transfer': 1.0},
            'sim': {'comparator_hysteresis_mv': 0.0, 'comparator_noise_mv': 5.0,
                    'comparator_phase_rc_ns': 800,
                    'comparator_neutral_rc_ns': 800},
        }

    def export(self, results):
        d = self.cfg.directory
        os.makedirs(d, exist_ok=True)
        paths = []
        p = os.path.join(d, 'README.md')
        with open(p, 'w') as f:
            f.write(self.readme_text(results))
        paths.append(p)
        p = os.path.join(d, 'expected.json')
        with open(p, 'w') as f:
            json.dump(self.expected_json(results), f, indent=4, sort_keys=True)
        paths.append(p)
        p = os.path.join(d, 'model.json')
        with open(p, 'w') as f:
            json.dump(self.model_json(results), f, indent=4)
        paths.append(p)
        return paths

    # -------------------------------------------------------------- restore

    def restore(self):
        '''put back anything we changed to make the capture work'''
        for kind, payload in self.restore_actions:
            try:
                if kind == 'msp_cli':
                    self._restore_cli(payload)
                elif kind == 'can_rates':
                    self.log('note: ESC TELEM_RATE/DEBUG_RATE were set for the '
                             'capture and were not saved to eeprom, so a power '
                             'cycle restores them')
            except Exception as ex:
                self.log('restore (%s) failed: %s' % (kind, ex))
        self.restore_actions = []

    def _wait_fc_back(self, timeout=25.0):
        """Betaflight reboots when it saves, so the port drops and can
        come back on a different device node"""
        import msp
        t0 = time.time()
        while time.time() - t0 < timeout and not self.aborting:
            time.sleep(1.0)
            for port in (self.cfg.port, None):
                try:
                    found = msp.find_fc_port(port)
                    p = msp.MspPort(found)
                    try:
                        msp.parse_api_version(p.request(msp.MSP_API_VERSION))
                    finally:
                        p.close()
                except BaseException:
                    continue
                if found != self.cfg.port:
                    self.log('the FC came back on %s' % found)
                    self.cfg.port = found
                return True
        self.log('WARNING: the FC did not answer within %.0fs of saving'
                 % timeout)
        return False

    def _restore_cli(self, settings):
        import serial
        s = serial.Serial(self.cfg.port, 115200, timeout=0.3,
                          write_timeout=2.0)
        try:
            s.write(b'#')
            time.sleep(1.2)
            s.reset_input_buffer()
            for k, v in settings.items():
                s.write(('set %s = %s\r\n' % (k, v)).encode())
                time.sleep(0.4)
                self.log('restored %s = %s' % (k, v))
            s.write(b'save\r\n')
            time.sleep(1.0)
        finally:
            try:
                s.close()
            except Exception:
                pass
        self._wait_fc_back()

    def set_fc_rate(self, hz=1000):
        '''raise the MSP serial rate, remembering the old value'''
        import serial
        s = serial.Serial(self.cfg.port, 115200, timeout=0.3,
                          write_timeout=2.0)
        old = None
        rebooted = False
        try:
            s.write(b'#')
            time.sleep(1.2)
            s.reset_input_buffer()
            s.write(b'get serial_update_rate_hz\r\n')
            time.sleep(0.8)
            txt = s.read(4096).decode('ascii', 'replace')
            m = re.search(r'serial_update_rate_hz\s*=\s*(\d+)', txt)
            if m:
                old = int(m.group(1))
            # never change what cannot be put back: without the old
            # value there is no restore action to register
            if old is None:
                self.log('WARNING: could not read serial_update_rate_hz - '
                         'leaving it alone. Telemetry will be capped near '
                         '50Hz unless it is already raised')
            elif old != hz:
                s.write(('set serial_update_rate_hz = %u\r\n' % hz).encode())
                time.sleep(0.4)
                s.write(b'save\r\n')
                time.sleep(1.0)
                rebooted = True
        finally:
            try:
                s.close()
            except Exception:
                pass
        if rebooted:
            self._wait_fc_back()
        if old is not None and old != hz:
            self.restore_actions.append(
                ('msp_cli', {'serial_update_rate_hz': old}))
            self.log('serial_update_rate_hz %u -> %u (saved, FC rebooted; '
                     'will be put back on exit)' % (old, hz))
        return old
