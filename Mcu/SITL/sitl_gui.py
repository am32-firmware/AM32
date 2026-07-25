#!/usr/bin/env python3
'''
control GUI for the AM32 SITL: drives PWM/DShot input over UDP and
DroneCAN input over mcast, with live telemetry from BDShot (including
extended DShot telemetry) and DroneCAN esc.Status.

each input has an Enable checkbox that instantly starts/stops its stream,
for exercising failover between inputs. The parameter panel sets
INPUT_SIGNAL_TYPE etc over DroneCAN (needed because the DRONECAN_IN
default disables the PWM/DShot input interrupts).

built on PySide6 (Qt): pyqtgraph plots and QGraphicsScene animation
panels can be added on this foundation. Install the dependencies with
    python3 Mcu/SITL/make_gui_env.py

usage: sitl_gui.py [--port 57733] [--can-uri mcast:0]

with --control-port N the UI can additionally be driven by commands over
a localhost TCP connection (one per line), for scripted testing of the
actual UI paths:
  ds_enable 0|1, ds_type pwm|dshot300|..., ds_bidir 0|1, ds_value N,
  ds_rate N, ds_edt 0|1, zero, edt_enable, edt_disable, can_enable 0|1,
  can_value X, can_rate N, param NAME VALUE, rpm_graph 0|1,
  rpm_window SECONDS, wave sine|square FREQ AMP BASE, wave off,
  snap FILE [rpm], status, quit
responses go back to the client prefixed with OK/STATUS/ERR. A client
disconnect leaves the GUI running.
--log FILE records every UI action with a timestamp; --replay FILE plays
a recording back with its original timing.
'''

import argparse
import os
import queue
import signal
import socket
import sys
import threading
import time

try:
    from PySide6.QtCore import Qt, QTimer, QRectF, QLineF
    from PySide6.QtGui import (QFontDatabase, QPen, QBrush, QColor, QPainter,
                               QIcon)
    from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox,
                                   QGraphicsScene, QGraphicsView,
                                   QGridLayout, QGroupBox, QHBoxLayout,
                                   QDoubleSpinBox, QLabel, QPushButton,
                                   QSlider, QSpinBox, QVBoxLayout, QWidget)
except ImportError:
    _here = os.path.dirname(os.path.abspath(__file__))
    if sys.platform == 'win32':
        _venv_py = os.path.join(_here, 'venv', 'Scripts', 'python.exe')
    else:
        _venv_py = os.path.join(_here, 'venv', 'bin', 'python3')
    _run_cmd = ' '.join([_venv_py, os.path.abspath(__file__)] + sys.argv[1:])
    if os.path.exists(_venv_py):
        sys.stderr.write(
            'PySide6 is required for the SITL GUI. The GUI environment is\n'
            'already set up, run:\n'
            '    %s\n' % _run_cmd)
    else:
        sys.stderr.write(
            'PySide6 is required for the SITL GUI. Either install the packages\n'
            'from %s into the system python,\n'
            'or create the self-contained environment with:\n'
            '    python3 %s\n'
            'and then run:\n'
            '    %s\n'
            % (os.path.join(_here, 'requirements-gui.txt'),
               os.path.join(_here, 'make_gui_env.py'),
               _run_cmd))
    sys.exit(1)

try:
    import pyqtgraph as pg
    import numpy as np    # a pyqtgraph dependency, so always present here
    HAVE_PYQTGRAPH = True
except ImportError:
    HAVE_PYQTGRAPH = False

import bisect
import collections
import glob
import math

import sitl_dshot as sd
import sitl_tones
from sitl_gui_backend import (DshotPanel, CanPanel, CanFrameCounter, EepromClient,
                              SimStream, ToneStream, AudioStream,
                              HAVE_DRONECAN)
from sitl_wave_dialog import wave_pixmap


class SimTimeAxis(object):
    '''monotonic simulated-time coordinate for the rpm graph: sim time
    from the state stream with only forward steps accumulated, so a
    firmware reboot (which resets the sim clock to zero) cannot run the
    axis backwards. In sim time the waveform periods read true at any
    speedup'''

    def __init__(self):
        self.acc = 0.0
        self.last = None

    def update(self, t):
        if self.last is not None and t > self.last:
            self.acc += t - self.last
        self.last = t
        return self.acc


class ScopeWindow(QWidget):
    '''a pyqtgraph plot in a window with an Auto Y control and a cursor
    readout. Auto Y checked rescales every vertical axis to its data
    (all traces, both axes on the rpm/throttle graph); unchecked freezes
    them so the mouse can zoom/pan. Hovering the plot shows the cursor
    position read off each vertical scale (independent of the traces),
    in the bar above the graph. readout is (name, viewbox, unit) per
    vertical axis'''

    def __init__(self, plot, yboxes, on_close, title, readout=()):
        super().__init__()
        self._on_close = on_close
        self._plot = plot
        self._readout_axes = list(readout)
        self._last_pos = None
        self.setWindowTitle(title)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.setSpacing(4)
        bar = QHBoxLayout()
        self.auto_y = QCheckBox('Auto Y')
        self.auto_y.setChecked(True)
        self.auto_y.setToolTip(
            'Rescale every vertical axis to fit its data (all traces,\n'
            'both axes on the rpm/throttle graph). Uncheck to freeze\n'
            'the scales, then zoom or pan with the mouse (the\n'
            'right-click menu has more view options).')

        def apply_auto():
            for vb in yboxes:
                if self.auto_y.isChecked():
                    vb.enableAutoRange(axis=vb.YAxis, enable=True)
                else:
                    vb.disableAutoRange(axis=vb.YAxis)

        self.auto_y.toggled.connect(apply_auto)
        bar.addWidget(self.auto_y)
        self.pause = QCheckBox('Pause')
        self.pause.setToolTip(
            'Freeze the display for a close look; the simulation and the\n'
            'data stream keep running. Unpausing jumps straight to the\n'
            'current data.')
        bar.addWidget(self.pause)
        bar.addStretch(1)
        self.bar = bar          # callers may insert extra controls
        self.readout = QLabel('')
        self.readout.setToolTip(
            'The cursor position read off each vertical scale (not the\n'
            'trace values - move the cursor onto a trace to read it).')
        bar.addWidget(self.readout)
        lay.addLayout(bar)
        lay.addWidget(plot)
        apply_auto()
        if self._readout_axes:
            # rate limited: mouse moves arrive far faster than the label
            # is worth updating
            self._proxy = pg.SignalProxy(plot.scene().sigMouseMoved,
                                         rateLimit=30,
                                         slot=self._cursor_moved)

    @staticmethod
    def _fmt(v):
        a = abs(v)
        if a >= 100:
            return '%.0f' % v
        return '%.2f' % v if a >= 1 else '%.3f' % v

    def _cursor_moved(self, evt):
        self._last_pos = evt[0]
        self._update_readout()

    def refresh_readout(self):
        '''re-evaluate the readout at the remembered cursor position:
        with Auto Y the axis ranges follow the data, so the scale value
        under a stationary cursor changes between redraws'''
        if self._readout_axes and self._last_pos is not None:
            self._update_readout()

    def _update_readout(self):
        pos = self._last_pos
        if not self._plot.sceneBoundingRect().contains(pos):
            self.readout.setText('')
            return
        # the cursor height mapped through each vertical axis's view box:
        # a pure scale reading, deliberately independent of the traces
        parts = []
        for name, vb, unit in self._readout_axes:
            y = vb.mapSceneToView(pos).y()
            parts.append('%s=%s%s' % (name, self._fmt(y), unit))
        self.readout.setText('  '.join(parts))

    def closeEvent(self, ev):
        self._on_close()
        ev.accept()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=57733)
    ap.add_argument('--state-port', type=int, default=57734)
    ap.add_argument('--can-uri', default='mcast:0')
    ap.add_argument('--poles', type=int, default=14)
    ap.add_argument('--control-port', type=int, default=0,
                    help='TCP port on localhost accepting UI control commands '
                         '(for scripted tests, default off)')
    ap.add_argument('--log', metavar='FILE',
                    help='log all UI actions with timestamps, for later --replay')
    ap.add_argument('--replay', metavar='FILE',
                    help='replay a --log action file with its original timing')
    args = ap.parse_args()

    t0 = time.time()
    logf = open(args.log, 'w') if args.log else None

    def log_action(cmd):
        if logf is not None:
            logf.write('%.3f %s\n' % (time.time() - t0, cmd))
            logf.flush()

    ds = DshotPanel(args.host, args.port)
    ds.poles = args.poles
    sim = SimStream(args.host, args.state_port)

    # throttle waveform override state and the rpm/throttle history, both
    # clocked off the simulation state stream so they follow the speedup.
    # 'run'/'value' belong to the waveform thread (see below)
    wave = {'kind': None, 'freq': 1.0, 'amp': 0.2, 'base': 0.3, 't0': None,
            'run': True, 'value': 0}
    # rpm/throttle history for the graph. The x axis is accumulated
    # SIMULATED time (SimTimeAxis) so waveform periods read true at any
    # speedup and firmware reboots (which reset the raw sim clock) cannot
    # run it backwards; the rolling window spans WALL time, so the view
    # stays equally live at any speedup - at 0.2x a sim-time window would
    # hold a full minute of history and recent events would crawl in as
    # a sliver at the right edge
    # sampled by the waveform thread every 5ms of SIM time (the GUI
    # tick's 20Hz aliases badly against sine/square tests above a few
    # Hz, and sim-time spacing keeps the waveform resolution the same at
    # any speedup); the deque holds the longest selectable window, the
    # graph slices what its window control asks for
    RPM_WINDOW_MAX_S = 60.0        # simulated seconds
    rpm_hist = collections.deque(maxlen=15000)
    rpm_lock = threading.Lock()    # sampler thread vs redraw snapshot
    rpm_taxis = SimTimeAxis()

    # spawn the DroneCAN node with SIGINT ignored: its multiprocessing IO
    # child inherits the kernel-level SIG_IGN disposition, so a terminal
    # Ctrl-C only interrupts this process and the child is shut down
    # through node.close() instead of dying with a traceback
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    can = CanPanel(args.can_uri) if HAVE_DRONECAN else None
    if can is not None:
        can.started.wait(5.0)

    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    win = QWidget()
    win.setWindowTitle('AM32 SITL control')
    top = QGridLayout(win)

    # ---- PWM/DShot input panel
    f1 = QGroupBox('PWM/DShot input (udp %s:%u)' % (args.host, args.port))
    f1.setToolTip(
        'The classic ESC signal wire, emulated over UDP: each packet is one\n'
        'frame on the wire, either a servo PWM pulse or a complete DShot\n'
        'frame. The frames are decoded by the unmodified AM32 firmware,\n'
        'including input auto-detection, CRC checking and arming.')
    g1 = QGridLayout(f1)
    top.addWidget(f1, 0, 0)

    ds_enable = QCheckBox('Enable')

    def ds_enable_changed():
        ds.enabled = ds_enable.isChecked()
        log_action('ds_enable %d' % int(ds.enabled))

    ds_enable.setToolTip(
        'Start/stop the frame stream. The ESC arms after seeing zero\n'
        'throttle for about 1.5 seconds, like on the bench. Unchecking\n'
        'simulates signal loss: the firmware signal timeout zeroes the\n'
        'motor and reboots the ESC, which is the behaviour exercised in\n'
        'input failover testing.')
    ds_enable.toggled.connect(ds_enable_changed)
    g1.addWidget(ds_enable, 0, 0)

    g1.addWidget(QLabel('Type:'), 1, 0)
    ds_type = QComboBox()
    ds_type.addItems(sorted(sd.TYPE_NAMES.keys()))
    ds_type.setCurrentText('dshot300')

    def type_changed(*a):
        ds.ptype = sd.TYPE_NAMES[ds_type.currentText()]
        is_pwm = ds.ptype == sd.TYPE_PWM
        ds_value.setRange(1000 if is_pwm else 0, 2000 if is_pwm else 2047)
        ds_value.setValue(1000 if is_pwm else 0)
        if ds.ptype == sd.TYPE_DSHOT150:
            # checkDshot() only has detection bands for dshot300/600
            ds.status = 'note: AM32 input detection does not support dshot150, use 300/600'
        else:
            ds.status = ''
        log_action('ds_type %s' % ds_type.currentText())
        value_changed()

    ds_type.setToolTip(
        'Signal protocol on the wire.\n'
        'pwm: the classic servo pulse, 1000..2000us pulse width = throttle,\n'
        '  sent at 50..490Hz.\n'
        'dshot300/600: digital frames at 300/600 kbit/s. Each frame is 16\n'
        '  bits: 11 bit throttle (48..2047; 0=stop, 1..47 are commands),\n'
        '  a telemetry request bit and a 4 bit checksum.\n'
        'dshot150 exists in the protocol but AM32 input auto-detection has\n'
        'no timing band for it, so the ESC never recognises it.')
    ds_type.currentTextChanged.connect(type_changed)
    g1.addWidget(ds_type, 1, 1)

    ds_bidir = QCheckBox('bidir (BDShot)')

    def ds_bidir_changed():
        ds.bidir = ds_bidir.isChecked()
        log_action('ds_bidir %d' % int(ds.bidir))

    ds_bidir.setToolTip(
        'Bidirectional DShot (BDShot): the signal line idles high and the\n'
        'frame checksum is inverted. After each received frame the ESC\n'
        'answers on the same wire with a GCR-encoded reply carrying the\n'
        'electrical rotation period, which flight controllers use for RPM\n'
        'notch filtering. The reply also carries extended telemetry frames\n'
        'when EDT is enabled. Required for the BDShot telemetry line below.')
    ds_bidir.toggled.connect(ds_bidir_changed)
    g1.addWidget(ds_bidir, 1, 2)

    g1.addWidget(QLabel('Throttle:'), 2, 0)
    ds_value = QSlider(Qt.Horizontal)
    ds_value.setRange(0, 2047)
    ds_value.setMinimumWidth(260)

    def value_changed(*a):
        v = ds_value.value()
        log_action('ds_value %d' % v)
        # never stream DShot values 1..47 from the throttle slider: they
        # are commands (direction, bi_direction, save, programming mode)
        # and a slider drag dwells long enough to execute them
        if ds.ptype != sd.TYPE_PWM and 0 < v < 48:
            v = 0
        ds.value = v

    ds_value.setToolTip(
        'Throttle value sent in every frame. DShot: 48..2047 (0 = motor\n'
        'stop). Values 1..47 are DShot commands (beacons, direction,\n'
        'settings save, programming mode) - the slider skips them because\n'
        'a drag would dwell long enough to execute one. PWM: pulse width\n'
        'in microseconds, 1000 = stop, 2000 = full throttle.')
    # valueChanged fires for both drags and programmatic setValue()
    ds_value.valueChanged.connect(value_changed)
    g1.addWidget(ds_value, 2, 1, 1, 2)
    ds_value_label = QLabel('0')
    g1.addWidget(ds_value_label, 2, 3)

    g1.addWidget(QLabel('Rate Hz:'), 3, 0)
    ds_rate = QSpinBox()
    ds_rate.setRange(10, 4000)
    ds_rate.setValue(500)

    def rate_changed(*a):
        ds.rate = float(ds_rate.value())
        log_action('ds_rate %d' % ds_rate.value())

    ds_rate.setToolTip(
        'Frame rate on the wire. Flight controllers send DShot at 1..8kHz\n'
        'and servo PWM at 50..490Hz. The ESC needs a continuous stream:\n'
        'arming, the signal-loss timeout and BDShot reply rate all follow\n'
        'the frame rate. Note the rate is wall clock, so at low simulation\n'
        'speedups frames arrive faster in simulated time and some are\n'
        'dropped while the virtual wire is busy, as on real hardware.')
    ds_rate.valueChanged.connect(rate_changed)
    g1.addWidget(ds_rate, 3, 1)

    bf = QHBoxLayout()
    zero_btn = QPushButton('Zero throttle')
    zero_btn.setToolTip(
        'Set the throttle to the stop value (DShot 0 / PWM 1000us).\n'
        'Needed for arming and before DShot commands such as the EDT\n'
        'enable are accepted by the firmware.')
    zero_btn.clicked.connect(
        lambda: ds_value.setValue(1000 if ds.ptype == sd.TYPE_PWM else 0))
    bf.addWidget(zero_btn)

    sine_btn = QPushButton(' Sine')
    sine_btn.setIcon(QIcon(wave_pixmap('sine')))
    sine_btn.setToolTip(
        'Override the throttle with a sine wave. Opens a dialog for the\n'
        'frequency, amplitude and base throttle. The frequency is in\n'
        'simulated time, so the waveform tracks the speedup slider. Enable\n'
        'the input and arm at zero throttle before starting.')
    bf.addWidget(sine_btn)

    square_btn = QPushButton(' Square')
    square_btn.setIcon(QIcon(wave_pixmap('square')))
    square_btn.setToolTip(
        'Override the throttle with a square wave, alternating between\n'
        'base+amplitude and base-amplitude. Opens a dialog for frequency,\n'
        'amplitude and base throttle, running in simulated time so it\n'
        'tracks the speedup slider.')
    bf.addWidget(square_btn)

    ds_edt = QCheckBox('EDT (extended telemetry)')

    def ds_edt_changed():
        ds.edt_want = ds_edt.isChecked()
        log_action('ds_edt %d' % int(ds.edt_want))

    ds_edt.setToolTip(
        'Extended DShot Telemetry: temperature, voltage and current frames\n'
        'interleaved into the BDShot replies (temperature/voltage every\n'
        '~200 replies, current every ~40). The firmware only accepts the\n'
        'enable command (DShot command 13) while armed with the motor\n'
        'stopped, and a reboot clears it, so this checkbox keeps re-sending\n'
        'the command until the replies show EDT frames. Needs bidir.')
    ds_edt.toggled.connect(ds_edt_changed)
    bf.addWidget(ds_edt)
    bf.addStretch(1)
    g1.addLayout(bf, 4, 0, 1, 4)

    ds_status = QLabel('arm: enable + hold zero throttle >1.5s')
    ds_status.setToolTip(
        'Guidance from the DShot sender: arming procedure, EDT progress\n'
        'and protocol notes.')
    g1.addWidget(ds_status, 5, 0, 1, 4)

    # ---- throttle waveform override: drive the PWM/DShot throttle as a
    # sine or square wave. The phase advances on simulation time from the
    # state stream, so the frequency is in simulated seconds and the
    # waveform tracks the speedup slider like everything else in the sim
    def throttle_frac():
        # current commanded throttle as a 0..1 fraction, from whichever
        # input is driving; feeds the rpm/throttle graph
        if wave['kind'] is not None or ds.enabled:
            if ds.ptype == sd.TYPE_PWM:
                return max(0.0, min(1.0, (ds.value - 1000) / 1000.0))
            return min(1.0, ds.value / 2047.0)
        if can is not None and can.enabled:
            return can.throttle
        return 0.0

    def wave_value():
        # current waveform throttle in DShot/PWM units, or None when the
        # override is off. Runs on the waveform thread: NO Qt calls here
        smp = sim.latest()
        if wave['kind'] is None or smp is None:
            return None
        t = smp[0]
        if wave['t0'] is None or t < wave['t0']:
            wave['t0'] = t              # (re-)anchor; a reboot resets sim time
        phase = 2.0 * math.pi * wave['freq'] * (t - wave['t0'])
        if wave['kind'] == 'sine':
            shape = math.sin(phase)
        else:
            shape = 1.0 if math.sin(phase) >= 0 else -1.0
        frac = max(0.0, min(1.0, wave['base'] + wave['amp'] * shape))
        if ds.ptype == sd.TYPE_PWM:
            return int(round(1000 + frac * 1000))
        v = int(round(frac * 2047))
        if 0 < v < 48:                  # never sit in the command range
            v = 48 if frac > 0 else 0
        return v

    # the waveform drives the throttle from its own thread, so the command
    # stays smooth however busy the Qt thread is repainting graphs - a Qt
    # timer would stall behind a slow repaint and the stuttered throttle
    # would desync the motor. The thread only writes ds.value (a plain
    # attribute the sender reads); the slider and label are refreshed
    # cosmetically from the Qt thread in update()
    def wave_loop():
        # also the rpm/throttle graph sampler: 200Hz beats the GUI tick's
        # 20Hz (which aliases against waveform tests above a few Hz), and
        # this thread is the throttle writer, so the sampled pair is
        # coherent. Entries are (sim_t, rpm, throttle, wall_t)
        while wave['run']:
            v = wave_value()
            if v is not None:
                ds.value = v
                wave['value'] = v
            smp = sim.latest()
            if smp is not None:
                t = rpm_taxis.update(smp[0])
                with rpm_lock:
                    if not rpm_hist or t >= rpm_hist[-1][0] + 0.005:
                        rpm_hist.append((t, smp[1] * 60.0 / (2 * math.pi),
                                         throttle_frac(), time.monotonic()))
            time.sleep(0.005)

    threading.Thread(target=wave_loop, daemon=True).start()

    WAVE_ACTIVE_STYLE = 'background-color: #cfe8ff; font-weight: bold'

    def wave_apply(kind, freq, amp, base):
        wave.update(kind=kind, freq=float(freq), amp=float(amp),
                    base=float(base), t0=None)
        log_action('wave %s %.4f %.4f %.4f' % (kind, freq, amp, base))
        ds_value.setEnabled(False)      # the waveform owns the throttle now
        sine_btn.setStyleSheet(WAVE_ACTIVE_STYLE if kind == 'sine' else '')
        square_btn.setStyleSheet(WAVE_ACTIVE_STYLE if kind == 'square' else '')

    def wave_stop():
        if wave['kind'] is not None:
            log_action('wave off')
        wave['kind'] = None
        ds_value.setEnabled(True)
        sine_btn.setStyleSheet('')
        square_btn.setStyleSheet('')

    wave_dialogs = {}

    def open_wave_dialog(kind):
        from sitl_wave_dialog import WaveDialog
        d = wave_dialogs.get(kind)
        if d is None:
            d = WaveDialog(kind, wave_apply, wave_stop, wave,
                           icon=wave_pixmap(kind, 44, 22), parent=win)
            wave_dialogs[kind] = d
        d.show()
        d.raise_()

    sine_btn.clicked.connect(lambda: open_wave_dialog('sine'))
    square_btn.clicked.connect(lambda: open_wave_dialog('square'))

    # ---- DroneCAN input panel
    f2 = QGroupBox('DroneCAN input (%s)' % args.can_uri)
    f2.setToolTip(
        'DroneCAN: the CAN bus protocol used on larger vehicles, here\n'
        'carried over multicast UDP. Throttle goes as esc.RawCommand\n'
        'broadcasts and the ESC sends esc.Status telemetry back. In the\n'
        'current firmware, once any RawCommand has been received the CAN\n'
        'input overrides the PWM/DShot wire until a reboot - the\n'
        'arbitration behaviour the failover parameter work is about.')
    g2 = QGridLayout(f2)
    top.addWidget(f2, 0, 1)

    # raw bus frame rate, counted straight off the multicast group so it
    # works with CAN control disabled and without the dronecan package
    can_fps = CanFrameCounter(args.can_uri)
    can_fps_label = QLabel('FPS: -')
    can_fps_label.setToolTip(
        'CAN frames per second seen on the bus, from every node: the\n'
        'ESC\'s own traffic, this GUI and anything else on the same\n'
        'multicast group. Counted independently of the Enable switch.\n'
        'Useful to check the bus really is quiet, e.g. the bootloader\'s\n'
        'no-CAN fallback (DShot/PWM boot) only arms when no frame at all\n'
        'arrives in its first 250ms.')
    g2.addWidget(can_fps_label, 0, 3)

    if can is not None:
        can_enable = QCheckBox('Enable')

        def can_enable_changed():
            can.enabled = can_enable.isChecked()
            log_action('can_enable %d' % int(can.enabled))

        can_enable.setToolTip(
            'Master switch for this GUI\'s CAN traffic: ArmingStatus at the\n'
            'configured rate plus RawCommand when its box is ticked.\n'
            'Cutting everything simulates losing the flight controller\n'
            'entirely: the firmware zeroes throttle 250ms after commands\n'
            'stop and the CAN signal timeout then reboots the ESC. Use the\n'
            'RawCommand box instead to cut only the command stream.')
        can_enable.toggled.connect(can_enable_changed)
        g2.addWidget(can_enable, 0, 0)

        can_dna = QCheckBox('DNA server')
        can_dna.setToolTip(
            'Serve dynamic node ID allocation, as an autopilot would. An\n'
            'ESC whose eeprom has no static node ID broadcasts anonymous\n'
            'allocation requests and stays off the bus (and the CAN\n'
            'bootloader stays put) until some allocator answers. The DNA\n'
            'label flashes while such requests are seen. While serving,\n'
            'the GUI node is on the bus even with Enable unchecked.')

        def can_dna_changed():
            can.dna_server = can_dna.isChecked()
            log_action('can_dna %d' % int(can.dna_server))

        can_dna.toggled.connect(can_dna_changed)
        g2.addWidget(can_dna, 0, 1)
        can_dna_label = QLabel('')
        can_dna_label.setToolTip(
            'Flashes while anonymous DNA allocation requests are seen on\n'
            'the bus: a node is waiting for an allocator. Tick DNA server\n'
            '(or run another allocator) to give it a node ID.')
        g2.addWidget(can_dna_label, 0, 2)

        can_rawcmd = QCheckBox('RawCommand')
        can_rawcmd.setChecked(True)
        can_rawcmd.setToolTip(
            'Send the esc.RawCommand throttle broadcasts. Unchecking\n'
            'simulates a flight controller that stops commanding while the\n'
            'rest of its CAN traffic continues (the ArmingStatus stream\n'
            'keeps flowing) - the firmware is expected to zero the input\n'
            '250ms after commands stop.')

        def can_rawcmd_changed():
            can.send_rawcommand = can_rawcmd.isChecked()
            log_action('can_rawcmd %d' % int(can.send_rawcommand))

        can_rawcmd.toggled.connect(can_rawcmd_changed)
        g2.addWidget(can_rawcmd, 1, 0)

        can_armed = QCheckBox('Armed')
        can_armed.setChecked(True)
        can_armed.setToolTip(
            'Value carried in the ArmingStatus broadcasts: checked sends\n'
            'FULLY_ARMED (255), unchecked sends disarmed (0). With the\n'
            'default REQUIRE_ARM setting the ESC only applies throttle\n'
            'while armed, so unchecking this while spinning must stop the\n'
            'motor. The ArmingStatus stream itself follows Enable; note\n'
            'the ESC ignores NodeStatus, so ArmingStatus is what keeps its\n'
            'CAN signal timeout fed.')

        def can_armed_changed():
            can.armed = can_armed.isChecked()
            log_action('can_armed %d' % int(can.armed))

        can_armed.toggled.connect(can_armed_changed)
        g2.addWidget(can_armed, 1, 1)

        g2.addWidget(QLabel('Throttle:'), 2, 0)
        # slider in 0..1000 -> throttle 0..1
        can_value = QSlider(Qt.Horizontal)
        can_value.setRange(0, 1000)
        can_value.setMinimumWidth(200)

        def can_value_changed(*a):
            can.throttle = can_value.value() / 1000.0
            log_action('can_value %.4f' % can.throttle)

        can_value.setToolTip(
            'Throttle 0..1, sent as esc.RawCommand 0..8191. The default\n'
            'firmware settings also require an ArmingStatus broadcast\n'
            '(sent automatically while enabled) before spinning.')
        can_value.valueChanged.connect(can_value_changed)
        g2.addWidget(can_value, 2, 1, 1, 2)
        can_value_label = QLabel('0.00')
        g2.addWidget(can_value_label, 2, 3)

        g2.addWidget(QLabel('Rate Hz:'), 3, 0)
        can_rate = QSpinBox()
        can_rate.setRange(1, 1000)
        can_rate.setValue(50)

        def can_rate_changed(*a):
            can.rate = float(can_rate.value())
            log_action('can_rate %d' % can_rate.value())

        can_rate.setToolTip(
            'RawCommand broadcast rate. Autopilots typically send ESC\n'
            'commands at 50..400Hz on CAN.')
        can_rate.valueChanged.connect(can_rate_changed)
        g2.addWidget(can_rate, 3, 1)

        can_zero_btn = QPushButton('Zero throttle')
        can_zero_btn.setToolTip('Set the CAN throttle to zero (keeps streaming commands).')
        can_zero_btn.clicked.connect(lambda: can_value.setValue(0))
        g2.addWidget(can_zero_btn, 4, 0)

        # parameter panel
        pf = QGroupBox('parameters (set + save + restart)')
        pf.setToolTip(
            'Sets an ESC setting over the DroneCAN parameter protocol, saves\n'
            'to eeprom and reboots the ESC so it takes effect at startup.\n'
            'INPUT_SIGNAL_TYPE selects which input the firmware listens to:\n'
            'the default 5 (dronecan) disables the PWM/DShot input\n'
            'interrupts entirely, so set 0..2 to test the signal wire.')
        gp = QGridLayout(pf)
        g2.addWidget(pf, 5, 0, 1, 4)
        gp.addWidget(QLabel('INPUT_SIGNAL_TYPE:'), 0, 0)
        ptype_var = QSpinBox()
        ptype_var.setToolTip(
            'INPUT_SIGNAL_TYPE value: 0 = auto detect the wire protocol,\n'
            '1 = dshot, 2 = servo PWM, 5 = dronecan only (disables the\n'
            'PWM/DShot input interrupts at boot).')
        ptype_var.setRange(0, 5)
        ptype_var.setValue(1)
        gp.addWidget(ptype_var, 0, 1)
        gp.addWidget(QLabel('(0=auto 1=dshot 2=servo 5=dronecan)'), 0, 2)
        param_status = QLabel('')
        gp.addWidget(param_status, 1, 0, 1, 3)

        def param_apply():
            log_action('param INPUT_SIGNAL_TYPE %d' % ptype_var.value())
            can.set_param('INPUT_SIGNAL_TYPE', ptype_var.value())

        apply_btn = QPushButton('Apply')
        apply_btn.setToolTip('Set the parameter over CAN, save to eeprom and reboot the ESC.')
        apply_btn.clicked.connect(param_apply)
        gp.addWidget(apply_btn, 0, 3)
    else:
        g2.addWidget(QLabel('pydronecan not available'), 0, 0)

    # ---- simulation panel: motor model selection and the optional high
    # rate graph/animation views fed by the SITL state stream
    f4 = QGroupBox('simulation')
    f4.setToolTip(
        'Controls for the physics simulation itself (not the ESC firmware):\n'
        'the motor/battery model, the simulation pace and the high rate\n'
        'views fed by the simulation state stream.')
    g4 = QGridLayout(f4)
    top.addWidget(f4, 3, 0, 1, 2)

    models_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'models')

    g4.addWidget(QLabel('Motor model:'), 0, 0)
    model_combo = QComboBox()
    model_paths = {}
    for path in sorted(glob.glob(os.path.join(models_dir, '*.json'))):
        name = os.path.splitext(os.path.basename(path))[0]
        model_paths[name] = path
        model_combo.addItem(name)
    model_combo.setToolTip(
        'Motor/battery models from Mcu/SITL/models/*.json: winding\n'
        'resistance and inductance, Kv (rpm per volt), pole count, rotor\n'
        'inertia and the propeller load (torque proportional to speed\n'
        'squared), plus battery voltage and internal resistance. These\n'
        'set how the virtual motor behaves; the ESC settings should\n'
        'match the motor, as on a real bench.')
    g4.addWidget(model_combo, 0, 1)

    def model_load():
        name = model_combo.currentText()
        if name in model_paths:
            log_action('model %s' % name)
            sim.load_model(model_paths[name])

    load_btn = QPushButton('Load')
    load_btn.setToolTip(
        'Apply the selected model to the running simulation. Switching\n'
        'while spinning is like swapping the motor mid-flight - expect\n'
        'desyncs; switch at zero throttle for clean results.')
    load_btn.clicked.connect(model_load)
    g4.addWidget(load_btn, 0, 2)
    model_status = QLabel('')
    g4.addWidget(model_status, 0, 3, 1, 2)

    graph_i_check = QCheckBox('Current graph')
    graph_v_check = QCheckBox('Voltage graph')
    graph_rpm_check = QCheckBox('RPM/throttle graph')
    motorview_check = QCheckBox('Motor view')
    graph_i_check.setToolTip(
        'Open a scope window with the three phase currents and the battery\n'
        'current, sampled from the physics. In a 6-step drive two phases\n'
        'conduct at a time, so each phase carries a quasi-trapezoidal\n'
        'current envelope that steps at every commutation (six steps per\n'
        'electrical revolution).')
    graph_v_check.setToolTip(
        'Open a scope window with the three phase terminal voltages and\n'
        'the battery voltage. At coarse sample periods it shows the duty\n'
        'proportional average; at fine periods (with a low speedup) the\n'
        'raw PWM switching, the back-EMF ramp on the floating phase (what\n'
        'the comparator senses for zero crossings) and the dead time diode\n'
        'spikes at each PWM edge become visible.')
    motorview_check.setToolTip(
        'Open the animated motor/bridge view. Slow the simulation right\n'
        'down with the speedup slider to watch individual commutation\n'
        'steps rotate the stator field ahead of the rotor.')
    graph_rpm_check.setToolTip(
        'Open a graph of the true motor rpm (left axis) and the commanded\n'
        'throttle (right axis, 0..1) against simulated time. Use it with\n'
        'the sine/square throttle overrides to watch the motor track the\n'
        'commanded waveform, or with the speedup slider to see the rpm\n'
        'settle after a step.')
    g4.addWidget(graph_i_check, 1, 0)
    g4.addWidget(graph_v_check, 1, 1)
    g4.addWidget(motorview_check, 1, 2)
    g4.addWidget(graph_rpm_check, 1, 3)
    sim_rate_label = QLabel('')
    sim_rate_label.setToolTip(
        'State stream rate actually arriving from the simulation (wall\n'
        'clock). It is the sample period in simulated time divided by the\n'
        'speedup, capped at about 200k samples/s.')
    g4.addWidget(sim_rate_label, 1, 4)

    # simulation speedup, logarithmic 0.001x .. 2x, for slow motion in
    # the motor view
    g4.addWidget(QLabel('Speedup:'), 2, 0)
    speed_slider = QSlider(Qt.Horizontal)
    speed_slider.setRange(0, 165)
    speed_slider.setValue(150)
    speed_slider.setMinimumWidth(200)
    speed_label = QLabel('1.000x')

    def slider_to_speedup(v):
        return 10.0 ** ((v - 150) / 50.0)

    def speed_changed(*a):
        speedup = slider_to_speedup(speed_slider.value())
        speed_label.setText('%.3fx' % speedup)
        log_action('speedup %.4f' % speedup)
        sim.set_speedup(speedup)

    speed_slider.setToolTip(
        'Simulation pace relative to wall clock, 0.001x to 2x. Everything\n'
        'inside the simulation (arming times, timeouts, telemetry rates)\n'
        'runs in simulated time, so at 0.01x the ESC responds 100x slower\n'
        'from the outside. Use slow motion to watch the motor view and to\n'
        'capture fine waveforms; the input frames you send arrive faster\n'
        'in simulated time, so some are dropped as on a busy wire.')
    speed_slider.valueChanged.connect(speed_changed)
    g4.addWidget(speed_slider, 2, 1, 1, 2)
    g4.addWidget(speed_label, 2, 3)
    speed_1x = QPushButton('1x')
    speed_1x.setToolTip('Back to real time.')
    speed_1x.clicked.connect(lambda: speed_slider.setValue(150))
    g4.addWidget(speed_1x, 2, 4)

    # stuck rotor: block the prop with a virtual obstruction, from
    # free to completely stuck, to exercise the firmware's stuck
    # rotor protection
    g4.addWidget(QLabel('Stuck rotor:'), 5, 0)
    stuck_slider = QSlider(Qt.Horizontal)
    stuck_slider.setRange(0, 100)
    stuck_slider.setValue(0)
    stuck_label = QLabel('free')

    def stuck_changed(*a):
        value = stuck_slider.value()
        stuck_label.setText('free' if value == 0 else
                            'locked' if value == 100 else '%d%%' % value)
        log_action('stuck %.2f' % (value / 100.0))
        sim.set_stuck(value / 100.0)

    stuck_slider.setToolTip(
        'Simulate the prop being blocked by an obstruction, e.g. hitting\n'
        'a tree branch. Partial values drag on the rotor with a fraction\n'
        'of the model\'s holding torque (motor.stuck_torque); fully right\n'
        'locks the rotor rigidly. With STUCK_ROTOR_PROTECTION enabled the\n'
        'firmware cuts the output after repeated commutation timeouts and\n'
        'stays off until the throttle is lowered to zero.')
    stuck_slider.valueChanged.connect(stuck_changed)
    g4.addWidget(stuck_slider, 5, 1, 1, 2)
    g4.addWidget(stuck_label, 5, 3)

    def stuck_release_clicked():
        # resend even when already showing free: the command is
        # unacknowledged UDP and another client may have left the
        # simulator stuck
        if stuck_slider.value() == 0:
            stuck_changed()
        else:
            stuck_slider.setValue(0)

    stuck_release = QPushButton('Release')
    stuck_release.setToolTip('Clear the obstruction, back to a free rotor.')
    stuck_release.clicked.connect(stuck_release_clicked)
    g4.addWidget(stuck_release, 5, 4)

    # audio: play the ESC beeps (startup tune, DShot beacons) through
    # the host sound output, from the tone event stream on the state
    # port. Stream and synth are created lazily on first enable
    audio_check = QCheckBox('Audio')
    audio_check.setToolTip(
        'Play the beeps the ESC firmware makes (startup tune, arm beeps,\n'
        'DShot beacons) through the sound output. On a real ESC these are\n'
        'the motor windings driven with PWM at an audible frequency; here\n'
        'the same PWM is synthesized as a sine. Pitch is unaffected by\n'
        'the speedup slider, duration scales with it.')
    g4.addWidget(audio_check, 4, 0)
    audio_slider = QSlider(Qt.Horizontal)
    audio_slider.setRange(0, 100)
    audio_slider.setValue(50)
    audio_slider.setToolTip('Playback volume, shared by both audio sources.')
    g4.addWidget(audio_slider, 4, 1, 1, 2)
    motor_audio_check = QCheckBox('Motor audio')
    motor_audio_check.setToolTip(
        'Play what the motor itself radiates, from the physics model:\n'
        'torque ripple and phase current magnitude, so commutation\n'
        'noise, PWM whine and beeps sound as a real motor would (Audio\n'
        'plays clean synthesized tones instead). Pitch follows the\n'
        'speedup slider - slow motion sounds lower, as physics should.')
    g4.addWidget(motor_audio_check, 4, 3)
    tones = None
    tone_synth = None
    phys_stream = None
    phys_player = None

    def audio_toggled(*a):
        nonlocal tones, tone_synth
        log_action('audio %d' % int(audio_check.isChecked()))
        if audio_check.isChecked() and tone_synth is None:
            tones = ToneStream(args.host, args.state_port)
            tone_synth = sitl_tones.ToneSynth(tones,
                                              volume=audio_slider.value() / 100.0)
            if not tone_synth.active:
                sys.stderr.write('audio: %s\n' % tone_synth.error)
                audio_check.setToolTip('Not available: %s' % tone_synth.error)
                tones.close()
                tones, tone_synth = None, None
                audio_check.setChecked(False)
                return
        if tones is not None:
            tones.enabled = audio_check.isChecked()

    def motor_audio_toggled(*a):
        nonlocal phys_stream, phys_player
        log_action('motor_audio %d' % int(motor_audio_check.isChecked()))
        if motor_audio_check.isChecked() and phys_player is None:
            phys_stream = AudioStream(args.host, args.state_port)
            phys_player = sitl_tones.PhysicsAudio(
                phys_stream, volume=audio_slider.value() / 100.0)
            if not phys_player.active:
                sys.stderr.write('motor audio: %s\n' % phys_player.error)
                motor_audio_check.setToolTip('Not available: %s'
                                             % phys_player.error)
                phys_stream.close()
                phys_stream, phys_player = None, None
                motor_audio_check.setChecked(False)
                return
        if phys_stream is not None:
            phys_stream.enabled = motor_audio_check.isChecked()

    def audio_volume_changed(*a):
        log_action('audio_volume %d' % audio_slider.value())
        if tone_synth is not None:
            tone_synth.set_volume(audio_slider.value() / 100.0)
        if phys_player is not None:
            phys_player.set_volume(audio_slider.value() / 100.0)

    audio_check.toggled.connect(audio_toggled)
    motor_audio_check.toggled.connect(motor_audio_toggled)
    audio_slider.valueChanged.connect(audio_volume_changed)

    # scope controls: sample period and window, shared by both graph
    # windows. Fine sample periods (down to the 500ns physics step,
    # where the dead time windows are visible on the phase voltages) are
    # meant to be used together with a low speedup to keep the wall
    # clock data rate sane
    sample_spin = QDoubleSpinBox()
    sample_spin.setRange(0.5, 1000.0)
    sample_spin.setValue(50.0)
    sample_spin.setSuffix(' us sample')
    sample_spin.setDecimals(1)
    g4.addWidget(sample_spin, 3, 0, 1, 2)
    window_spin = QDoubleSpinBox()
    window_spin.setRange(0.05, 2000.0)
    window_spin.setValue(50.0)
    window_spin.setSuffix(' ms window')
    window_spin.setDecimals(2)
    g4.addWidget(window_spin, 3, 2)

    def sample_changed(*a):
        update_sim_enable()
        log_action('sample_us %.1f' % sample_spin.value())

    sample_spin.setToolTip(
        'Scope sample period in simulated microseconds. At 10us and above\n'
        'each sample is the average over its period (the honest duty\n'
        'proportional envelope - point sampling would alias against the\n'
        'PWM and show false gaps). Below 10us samples are instantaneous\n'
        'levels, down to the 500ns physics step, showing PWM edges and\n'
        'dead time. The wall clock rate is capped at ~200k samples/s, so\n'
        'fine periods only take full effect at low speedups.')
    sample_spin.valueChanged.connect(sample_changed)

    def window_changed(*a):
        log_action('window_ms %.2f' % window_spin.value())

    window_spin.setToolTip(
        'Scope x axis span in simulated milliseconds, newest sample at the\n'
        'right edge. For commutation-scale viewing use 10..50ms; for PWM\n'
        'and dead time detail use 0.1..1ms together with a fine sample\n'
        'period and a low speedup.')
    window_spin.valueChanged.connect(window_changed)

    # the scopes: each signal set gets its own top level pyqtgraph
    # window, created lazily on first enable. Closing a window unchecks
    # its box
    SIGNAL_SETS = {
        # key -> ((label, colour, sample column), ...), y axis label/unit
        'i': ((('iu', 'r', 4), ('iv', 'g', 5), ('iw', 'b', 6),
               ('ibus', 'w', 11)), 'current', 'A'),
        'v': ((('vu', 'r', 7), ('vv', 'g', 8), ('vw', 'b', 9),
               ('vbus', 'w', 10)), 'voltage', 'V'),
    }
    graph_windows = {}
    rpm_graph = {}

    def update_sim_enable():
        # the status pane always needs the stream; scopes and the motor
        # view raise the rate from the coarse status sampling
        fine = (graph_i_check.isChecked() or graph_v_check.isChecked()
                or motorview_check.isChecked())
        sim.period_us = sample_spin.value() if fine else 2000
        sim.enabled = True

    update_sim_enable()   # status pane streams from startup

    def graph_toggled(key, check, title):
        log_action('graph_%s %d' % (key, int(check.isChecked())))
        if check.isChecked() and key not in graph_windows:
            if not HAVE_PYQTGRAPH:
                model_status.setText('pyqtgraph not available')
                check.setChecked(False)
                return

            plot = pg.PlotWidget()
            # decimate to screen resolution so a busy repaint cannot stall
            # the Qt thread (and starve the sender/reader threads)
            plot.setDownsampling(mode='peak', auto=True)
            plot.setClipToView(True)
            if key == 'i':
                plot.setToolTip(
                    'Phase currents iu/iv/iw (red/green/blue) and battery\n'
                    'current ibus (white). Two phases conduct at a time in a\n'
                    '6-step drive: current flows in one phase, back through\n'
                    'another, and the third floats near zero. Each commutation\n'
                    'advances the pattern; torque is proportional to the\n'
                    'conducted current. ibus is the PWM-chopped share drawn\n'
                    'from the battery - averaged it equals duty times phase\n'
                    'current, which is why battery current is far below phase\n'
                    'current at partial throttle.')
            else:
                plot.setToolTip(
                    'Phase terminal voltages vu/vv/vw (red/green/blue) and\n'
                    'battery voltage vbus (white). At coarse sample periods\n'
                    'the trace is the duty proportional average. With a fine\n'
                    'sample period and low speedup the raw switching shows:\n'
                    'the PWM square wave on the driven phase, the low side\n'
                    'held near 0V, the floating phase ramping with back-EMF\n'
                    'through the zero crossing the comparator detects, and\n'
                    '500ns dead time spikes to -0.7V or vbus+0.7V where the\n'
                    'body diodes conduct at each PWM edge.')
            plot.addLegend(offset=(10, 10))
            plot.setLabel('bottom', 'time', 's')
            defs, label, unit = SIGNAL_SETS[key]
            plot.setLabel('left', label, unit)
            curves = [(plot.plot(pen=pg.mkPen(color, width=1), name=name), col)
                      for name, color, col in defs]
            vb = plot.getPlotItem().getViewBox()
            w = ScopeWindow(plot, [vb], lambda: check.setChecked(False),
                            'AM32 SITL %s' % title,
                            readout=[(label, vb, unit)])
            w.resize(700, 340)
            graph_windows[key] = (w, plot, curves)
        if key in graph_windows:
            graph_windows[key][0].setVisible(check.isChecked())
        update_sim_enable()

    graph_i_check.toggled.connect(
        lambda: graph_toggled('i', graph_i_check, 'phase currents'))
    graph_v_check.toggled.connect(
        lambda: graph_toggled('v', graph_v_check, 'phase voltages'))

    # rpm and throttle share a time axis but not a scale, so the throttle
    # gets its own right hand axis (0..1) on a linked view box
    def make_rpm_graph():
        plot = pg.PlotWidget()
        plot.setDownsampling(mode='peak', auto=True)
        plot.setClipToView(True)
        plot.setToolTip(
            'True motor rpm from the physics (yellow, left axis) and the\n'
            'commanded throttle (cyan dashed, right axis 0..1) over the\n'
            'selected window of SIMULATED time (0 = now): waveform\n'
            'periods read true at any speedup. Sampled every 5ms of sim\n'
            'time; shrink the window to resolve fast waveforms. Auto Y\n'
            'rescales both the rpm and throttle axes.')
        p1 = plot.plotItem
        p1.setLabel('bottom', 'sim time', 's')
        p1.setLabel('left', 'rpm')
        p1.getAxis('left').enableAutoSIPrefix(False)   # show plain rpm
        p1.showGrid(x=True, y=True, alpha=0.2)
        leg = p1.addLegend(offset=(10, 10))
        rpm_curve = p1.plot(pen=pg.mkPen('y', width=2), name='rpm')
        # throttle on a second view box sharing the x axis
        thr_vb = pg.ViewBox()
        p1.showAxis('right')
        p1.scene().addItem(thr_vb)
        p1.getAxis('right').linkToView(thr_vb)
        thr_vb.setXLink(p1)
        thr_vb.setYRange(0, 1, padding=0)
        p1.getAxis('right').enableAutoSIPrefix(False)
        p1.getAxis('right').setLabel('throttle', '0..1')
        thr_curve = pg.PlotCurveItem(pen=pg.mkPen('c', width=2, style=Qt.DashLine))
        thr_vb.addItem(thr_curve)
        leg.addItem(thr_curve, 'throttle')

        def sync_vb():
            thr_vb.setGeometry(p1.vb.sceneBoundingRect())
            thr_vb.linkedViewChanged(p1.vb, thr_vb.XAxis)

        p1.vb.sigResized.connect(sync_vb)
        w = ScopeWindow(plot, [p1.vb, thr_vb],
                        lambda: graph_rpm_check.setChecked(False),
                        'AM32 SITL rpm / throttle',
                        readout=[('rpm', p1.vb, ''), ('throttle', thr_vb, '')])
        # window length control: shrink it to resolve fast waveform
        # tests - 84 cycles of a 7Hz sine across a 12s window can only
        # alias, a 2s window shows them cleanly
        win_spin = QDoubleSpinBox()
        win_spin.setRange(0.5, RPM_WINDOW_MAX_S)
        win_spin.setValue(12.0)
        win_spin.setDecimals(1)
        win_spin.setSuffix(' s window')
        win_spin.setToolTip(
            'Displayed history length in SIMULATED seconds, matching the\n'
            'axis units and the waveform test frequencies. Reduce it to\n'
            'resolve fast sine/square tests; widening shows history\n'
            'already recorded.')
        win_spin.valueChanged.connect(
            lambda v: log_action('rpm_window %.1f' % v))
        w.bar.insertWidget(2, win_spin)
        w.resize(700, 340)
        rpm_graph.update(win=w, p1=p1, rpm_curve=rpm_curve,
                         thr_curve=thr_curve, win_spin=win_spin)
        sync_vb()

    def graph_rpm_toggled():
        log_action('rpm_graph %d' % int(graph_rpm_check.isChecked()))
        if graph_rpm_check.isChecked() and 'win' not in rpm_graph:
            if not HAVE_PYQTGRAPH:
                model_status.setText('pyqtgraph not available')
                graph_rpm_check.setChecked(False)
                return
            make_rpm_graph()
        if 'win' in rpm_graph:
            rpm_graph['win'].setVisible(graph_rpm_check.isChecked())

    graph_rpm_check.toggled.connect(graph_rpm_toggled)

    # motor/bridge animation, created lazily on first enable
    view = None
    scene = None
    anim = {}

    def make_motor_view():
        nonlocal view, scene
        scene = QGraphicsScene(0, 0, 420, 220)
        view = QGraphicsView(scene)
        view.setToolTip(
            'Live motor and bridge state.\n'
            'Dial: the orange needle is the mechanical rotor angle, the\n'
            'cyan needle the electrical angle - with 7 pole pairs the\n'
            'electrical needle turns 7x faster, one electrical turn per 6\n'
            'commutation steps.\n'
            'U/V/W bars: the bridge output for each phase - green = PWM\n'
            'driven high side, blue = tied low, grey = floating (undriven,\n'
            'used for back-EMF sensing), orange = brake PWM.\n'
            'The comparator line shows which floating phase is being\n'
            'watched for its back-EMF zero crossing, which is how a\n'
            'sensorless ESC knows the rotor position.\n'
            'Use the speedup slider for slow motion.')
        view.setRenderHint(QPainter.Antialiasing)
        view.setFixedHeight(240)
        # rotor dial
        scene.addEllipse(20, 20, 180, 180, QPen(QColor('gray'), 2))
        anim['needle'] = scene.addLine(QLineF(110, 110, 110, 30), QPen(QColor('orange'), 4))
        anim['e_needle'] = scene.addLine(QLineF(110, 110, 110, 60), QPen(QColor('cyan'), 2))
        scene.addSimpleText('rotor').setPos(95, 202)
        # bridge legs: three vertical phase bars, coloured by mode
        anim['legs'] = []
        for p, name in enumerate(('U', 'V', 'W')):
            x = 240 + p * 55
            rect = scene.addRect(QRectF(x, 40, 36, 120), QPen(Qt.NoPen), QBrush(QColor('gray')))
            anim['legs'].append(rect)
            label = scene.addSimpleText(name)
            label.setPos(x + 12, 165)
        anim['comp'] = scene.addSimpleText('')
        anim['comp'].setPos(240, 190)
        anim['rpm'] = scene.addSimpleText('')
        anim['rpm'].setPos(240, 12)
        top.addWidget(view, 4, 0, 1, 2)

    MODE_COLORS = {
        0: QColor(90, 90, 90),      # FLOAT
        1: QColor(60, 100, 220),    # LOW
        2: QColor(60, 190, 60),     # PWM
        3: QColor(60, 190, 120),    # PWM_NOCOMP
        4: QColor(220, 140, 40),    # BRAKE_PWM
    }

    def motorview_changed():
        log_action('motorview %d' % int(motorview_check.isChecked()))
        if motorview_check.isChecked() and view is None:
            make_motor_view()
        if view is not None:
            view.setVisible(motorview_check.isChecked())
        update_sim_enable()
        win.adjustSize()

    motorview_check.toggled.connect(motorview_changed)

    scope_pace = {'tick': 0, 'every': 1}

    def update_sim_views():
        # paused windows keep their frozen display; the stream keeps
        # running so unpausing lands on the current data
        active = [t for t in graph_windows.values()
                  if t[0].isVisible() and not t[0].pause.isChecked()]
        if active:
            # huge windows (fine sample periods) redraw less often: the
            # data prep holds the GIL on the Qt thread, and physics-facing
            # threads (DShot sender, waveform) must keep their cadence
            scope_pace['tick'] += 1
            if scope_pace['tick'] >= scope_pace['every']:
                scope_pace['tick'] = 0
                win_s = window_spin.value() * 1e-3
                w = sim.window(win_s)
                if w:
                    scope_pace['every'] = 1 + len(w) // 10000
                    # one C-speed array build instead of a python
                    # comprehension per curve; numpy arrays are also
                    # pyqtgraph's fast path in setData
                    arr = np.array([smp[:12] for smp in w])
                    ts = arr[:, 0] - (w[-1][0] - win_s)
                    for cont, plot, curves in active:
                        for curve, col in curves:
                            curve.setData(ts, arr[:, col])
                        # x axis is the window control, not auto range
                        plot.setXRange(0, win_s, padding=0)
                        cont.refresh_readout()
        if motorview_check.isChecked() and view is not None:
            smp = sim.latest()
            if smp is not None:
                t, omega, theta, theta_e = smp[0], smp[1], smp[2], smp[3]
                modes, comp_ph, comp_out = smp[12], smp[13], smp[14]
                cx, cy, r = 110, 110, 80
                anim['needle'].setLine(QLineF(cx, cy, cx + r * math.sin(theta),
                                              cy - r * math.cos(theta)))
                re = r * 0.6
                anim['e_needle'].setLine(QLineF(cx, cy, cx + re * math.sin(theta_e),
                                                cy - re * math.cos(theta_e)))
                for p in range(3):
                    anim['legs'][p].setBrush(QBrush(MODE_COLORS.get(modes[p], QColor('gray'))))
                anim['comp'].setText('comparator: phase %s out=%d' % ('UVW'[comp_ph], comp_out))
                anim['rpm'].setText('%.0f rpm' % (omega * 60 / (2 * math.pi)))
        # rpm/throttle sampling lives in the waveform thread (200Hz);
        # here we only redraw. The history keeps rolling while paused,
        # so unpausing shows the current window, and it holds the
        # longest selectable window so widening shows history at once
        rg = rpm_graph.get('win')
        if rg is not None and rg.isVisible() and not rg.pause.isChecked():
            with rpm_lock:
                hist = list(rpm_hist)          # C-speed snapshot
            if hist:
                # the window is SIM seconds, matching the axis units and
                # the waveform frequencies
                win_s = rpm_graph['win_spin'].value()
                i0 = bisect.bisect_left(hist, hist[-1][0] - win_s,
                                        key=lambda s: s[0])
                hist = hist[i0:]
            if len(hist) > 1:
                # x is sim seconds before now (0 at the right edge); the
                # spanned range varies with speedup, the tick spacing
                # stays honest sim time
                arr = np.asarray(hist)
                ts = arr[:, 0] - arr[-1, 0]
                rpm_graph['rpm_curve'].setData(ts, arr[:, 1])
                rpm_graph['thr_curve'].setData(ts, arr[:, 2])
                rpm_graph['p1'].setXRange(float(ts[0]), 0, padding=0)
                rg.refresh_readout()

    sim_view_timer = QTimer()
    sim_view_timer.timeout.connect(update_sim_views)
    # 20fps: the repaints run on the Qt thread, so a lower rate leaves the
    # event loop more time to service the audio pull between frames
    sim_view_timer.start(50)

    # ---- simulator status: the true state of the simulated machine,
    # straight from the physics stream, as opposed to the telemetry
    # panel below which shows what the firmware reports over its links
    fixed = QFontDatabase.systemFont(QFontDatabase.FixedFont)
    fs = QGroupBox('simulator status (true state)')
    fs.setToolTip(
        'The actual state of the simulation, read from the state stream:\n'
        'not what the firmware measures or reports. Differences between\n'
        'this and the telemetry panel below are the ESC firmware\n'
        'mismeasuring (ADC scaling, commutation timing), which is often\n'
        'exactly what you want to see.')
    gs = QGridLayout(fs)
    top.addWidget(fs, 1, 0, 1, 2)
    sim_status = QLabel('rpm=- volt=- current=- armed=-')
    sim_status.setFont(fixed)
    gs.addWidget(sim_status, 0, 0)
    param_btn = QPushButton('Parameters...')
    param_btn.setToolTip(
        'Edit the simulated ESC eeprom directly over the simulator link\n'
        '(no 4-way or DroneCAN parameter protocol involved).')
    gs.addWidget(param_btn, 0, 1)
    eeprom_client = EepromClient(args.host, args.state_port)
    param_state = {'dialog': None, 'mismatch': None, 'next_check': 0.0,
                   'input_hint': None}

    def open_params():
        from sitl_param_dialog import ParamDialog
        d = param_state.get('dialog')
        if d is None:
            d = ParamDialog(eeprom_client, win)
            param_state['dialog'] = d
        else:
            d.refresh()
        d.show()
        d.raise_()
    param_btn.clicked.connect(open_params)

    # ---- telemetry panel
    f3 = QGroupBox('telemetry (as the firmware reports it)')
    g3 = QGridLayout(f3)
    top.addWidget(f3, 2, 0, 1, 2)
    bds_label = QLabel('BDShot: -')
    bds_label.setToolTip(
        'Telemetry decoded from the BDShot replies on the signal wire:\n'
        'rpm: from the eRPM period in each reply and the pole count.\n'
        'spinning/stopped: whether the replies report rotation.\n'
        'EDT on/off: whether extended telemetry frames are arriving.\n'
        'sent/replies: frame rates on the virtual wire; replies stop when\n'
        '  the wire is saturated or the ESC is rebooting.\n'
        'badcrc: replies that failed their checksum.\n'
        'temp/volt/current: extended telemetry values when EDT is on.')
    bds_label.setFont(fixed)
    g3.addWidget(bds_label, 0, 0)
    can_label = QLabel('DroneCAN: -')
    can_label.setToolTip(
        'Telemetry from the DroneCAN esc.Status broadcasts:\n'
        'rpm/volt/cur/temp: as reported by the firmware (voltage and\n'
        '  current from the simulated ADC path).\n'
        'err: desync count - commutation losses detected by the firmware.\n'
        'esc.Status: telemetry rate (TELEM_RATE parameter, sim time).\n'
        'cmds: RawCommand rate this GUI is sending.\n'
        'node: the ESC node id; up: firmware uptime in simulated seconds\n'
        '  (resets on every reboot, so it exposes signal-timeout reboots).')
    can_label.setFont(fixed)
    g3.addWidget(can_label, 1, 0)

    # ---- optional TCP control interface, driving the same widgets and
    # handlers as the mouse, so scripted tests cover the UI paths.
    # Commands are one per line; OK/STATUS/ERR responses go back to the
    # issuing client. A client disconnect leaves the GUI running; the
    # quit command closes it
    cmd_queue = queue.Queue()   # (line, reply function)

    def emit(msg):
        print(msg)
        sys.stdout.flush()

    def control_client(conn):
        def reply(msg):
            try:
                conn.sendall((msg + '\n').encode())
            except OSError:
                pass
        f = conn.makefile('r')
        for line in f:
            cmd_queue.put((line, reply))
        conn.close()

    def control_server():
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('127.0.0.1', args.control_port))
        srv.listen(4)
        while True:
            conn, _ = srv.accept()
            threading.Thread(target=control_client, args=(conn,), daemon=True).start()

    def handle_command(line, reply):
        parts = line.split()
        if not parts:
            return
        cmd, cargs = parts[0], parts[1:]
        if cmd == 'ds_enable':
            ds_enable.setChecked(bool(int(cargs[0])))
        elif cmd == 'ds_type':
            ds_type.setCurrentText(cargs[0])
        elif cmd == 'ds_value':
            ds_value.setValue(int(cargs[0]))
        elif cmd == 'ds_bidir':
            ds_bidir.setChecked(bool(int(cargs[0])))
        elif cmd == 'ds_rate':
            ds_rate.setValue(int(cargs[0]))
        elif cmd == 'zero':
            ds_value.setValue(1000 if ds.ptype == sd.TYPE_PWM else 0)
        elif cmd == 'ds_edt':
            ds_edt.setChecked(bool(int(cargs[0])))
        elif cmd == 'edt_enable':
            ds_edt.setChecked(True)
        elif cmd == 'edt_disable':
            ds_edt.setChecked(False)
        elif cmd == 'can_enable' and can is not None:
            can_enable.setChecked(bool(int(cargs[0])))
        elif cmd == 'can_dna' and can is not None:
            can_dna.setChecked(bool(int(cargs[0])))
        elif cmd == 'can_rawcmd' and can is not None:
            can_rawcmd.setChecked(bool(int(cargs[0])))
        elif cmd == 'can_armed' and can is not None:
            can_armed.setChecked(bool(int(cargs[0])))
        elif cmd == 'can_value' and can is not None:
            can_value.setValue(int(float(cargs[0]) * 1000))
        elif cmd == 'can_rate' and can is not None:
            can_rate.setValue(int(cargs[0]))
        elif cmd == 'param' and can is not None:
            can.set_param(cargs[0], int(cargs[1]))
        elif cmd == 'motorview':
            motorview_check.setChecked(bool(int(cargs[0])))
        elif cmd == 'rpm_graph':
            graph_rpm_check.setChecked(bool(int(cargs[0])))
        elif cmd == 'rpm_window':
            if 'win_spin' in rpm_graph:
                rpm_graph['win_spin'].setValue(float(cargs[0]))
        elif cmd == 'wave':
            if cargs and cargs[0] == 'off':
                wave_stop()
            else:
                wave_apply(cargs[0], float(cargs[1]), float(cargs[2]),
                           float(cargs[3]))
        elif cmd == 'model':
            if cargs[0] in model_paths:
                model_combo.setCurrentText(cargs[0])
                model_load()
            else:
                sim.load_model(cargs[0])
        elif cmd == 'graph_i' or cmd == 'graphs':
            graph_i_check.setChecked(bool(int(cargs[0])))
        elif cmd == 'graph_v':
            graph_v_check.setChecked(bool(int(cargs[0])))
        elif cmd == 'signals':
            # compatibility with recordings from the single-graph UI
            if cargs[0] == 'voltages':
                graph_v_check.setChecked(True)
            else:
                graph_i_check.setChecked(True)
        elif cmd == 'sample_us':
            sample_spin.setValue(float(cargs[0]))
        elif cmd == 'window_ms':
            window_spin.setValue(float(cargs[0]))
        elif cmd == 'speedup':
            x = float(cargs[0])
            pos = int(round(150 + 50 * math.log10(max(0.001, min(2.0, x)))))
            if pos == speed_slider.value():
                speed_changed()
            else:
                speed_slider.setValue(pos)
        elif cmd == 'stuck':
            x = float(cargs[0])
            if not math.isfinite(x):
                raise ValueError('stuck %r' % cargs[0])
            pos = int(round(max(0.0, min(1.0, x)) * 100))
            if pos == stuck_slider.value():
                stuck_changed()
            else:
                stuck_slider.setValue(pos)
        elif cmd == 'audio':
            audio_check.setChecked(bool(int(cargs[0])))
        elif cmd == 'motor_audio':
            motor_audio_check.setChecked(bool(int(cargs[0])))
        elif cmd == 'audio_volume':
            audio_slider.setValue(int(cargs[0]))
        elif cmd == 'snap':
            # screenshot a window to a file, for scripted visual checks
            tgt = win
            if len(cargs) > 1 and cargs[1] == 'rpm' and 'win' in rpm_graph:
                tgt = rpm_graph['win']
            tgt.grab().save(cargs[0])
        elif cmd == 'status':
            reply('STATUS %s' % bds_label.text())
            reply('STATUS %s' % can_label.text())
            reply('STATUS ds: %s' % (param_state['input_hint'] or ds.status
                                     or '-'))
            reply('STATUS sim: %s rate=%.0f/s' % (sim.model_status or '-', sim.rate.hz()))
            smp = sim.latest()
            reply('STATUS rpmhist: n=%d stream_t=%.3f taxis_last=%.3f acc=%.3f'
                  % (len(rpm_hist),
                     smp[0] if smp else -1,
                     rpm_taxis.last if rpm_taxis.last is not None else -1,
                     rpm_taxis.acc))
            reply('STATUS canbus: %s%s' %
                  ('%.0f fps' % can_fps.rate.hz() if can_fps.available
                   else can_fps.error,
                   ' dna-request' if can_fps.available
                   and can_fps.dna_request_seen() else ''))
            if tones is not None:
                freq, amp = tones.current()
                reply('STATUS audio: on vol=%d freq=%.1f amp=%.5f'
                      % (audio_slider.value(), freq, amp))
            else:
                reply('STATUS audio: off')
            if phys_stream is not None:
                reply('STATUS motor_audio: on rate=%.0f/s'
                      % phys_stream.rate.hz())
            else:
                reply('STATUS motor_audio: off')
            return
        elif cmd == 'quit':
            app.quit()
            return
        else:
            reply('ERR unknown command: %s' % line.strip())
            return
        reply('OK %s' % line.strip())

    def cmd_poll():
        while True:
            try:
                line, reply = cmd_queue.get_nowait()
            except queue.Empty:
                return
            try:
                handle_command(line, reply)
            except Exception as ex:
                reply('ERR %s: %s' % (line.strip(), ex))

    def replay_reader():
        with open(args.replay) as f:
            entries = []
            for line in f:
                parts = line.strip().split(None, 1)
                if len(parts) == 2:
                    entries.append((float(parts[0]), parts[1]))
        rt0 = time.time()
        for when, cmd in entries:
            delay = rt0 + when - time.time()
            if delay > 0:
                time.sleep(delay)
            cmd_queue.put((cmd + '\n', emit))
        emit('REPLAY done (%u actions)' % len(entries))

    cmd_timer = QTimer()
    cmd_timer.timeout.connect(cmd_poll)
    if args.control_port > 0:
        threading.Thread(target=control_server, daemon=True).start()
    if args.replay:
        threading.Thread(target=replay_reader, daemon=True).start()
    if args.control_port > 0 or args.replay:
        cmd_timer.start(50)

    def update_sim_status():
        smp = sim.latest()
        if smp is None:
            sim_status.setText('rpm=- volt=- current=- armed=-   '
                               '(enable a scope or motor view to stream state)')
        else:
            rpm = smp[1] * 60.0 / (2 * math.pi)
            armed = 'yes' if (ds.spinning or (can is not None and
                                              can.status.get('rpm', 0))) else '-'
            sim_status.setText(
                'rpm=%-8.0f volt=%-6.2f current=%-6.2f armed=%s'
                % (rpm, smp[10], smp[11], armed))
        # tint the parameters button while the stored settings disagree
        # with the simulated motor: silent mismatches (a stale MOTOR_KV
        # especially) look like physics faults, not configuration
        now = time.time()
        if now >= param_state['next_check']:
            param_state['next_check'] = now + 3.0
            image, model = eeprom_client.fetch()
            if image is not None:
                import sitl_params
                bad = sitl_params.mismatches(image, model)
                if bad != param_state['mismatch']:
                    param_state['mismatch'] = bad
                    if bad:
                        param_btn.setStyleSheet(
                            'background-color: #ffd6d6; font-weight: bold')
                        param_btn.setText('Parameters (%s)...' % ', '.join(bad))
                    else:
                        param_btn.setStyleSheet('')
                        param_btn.setText('Parameters...')
                # the freshly seeded eeprom default is 5 (dronecan only),
                # which disables the PWM/DShot input at boot: an enabled
                # DShot stream then gets zero replies, never arms, and
                # with no CAN commander the ESC reboot-loops on its
                # signal timeout. Say so instead of leaving replies=0/s
                # as the only clue
                if len(image) > 46 and image[46] == 5 and ds.enabled:
                    param_state['input_hint'] = (
                        'eeprom INPUT_SIGNAL_TYPE=5 (dronecan): the '
                        'PWM/DShot input is disabled at boot - set 1 '
                        '(dshot) or 2 (servo) in Parameters')
                else:
                    param_state['input_hint'] = None
                # the INPUT_SIGNAL_TYPE editor tracks the stored byte so
                # it shows what the ESC actually has, not a stale entry
                # (it used to sit at its initial value forever); leave it
                # alone while the user is editing
                if (can is not None and len(image) > 46
                        and not ptype_var.hasFocus()
                        and ptype_var.value() != image[46]):
                    ptype_var.setValue(image[46])

    def update():
        update_sim_status()
        if wave['kind'] is not None:
            # follow the waveform thread's throttle on the slider (display
            # only; the real command already went out on the wave thread)
            ds_value.blockSignals(True)
            ds_value.setValue(wave['value'])
            ds_value.blockSignals(False)
        ds_value_label.setText(str(ds_value.value()))
        ds_status.setText(param_state['input_hint'] or ds.status
                          or 'arm: enable + hold zero throttle >1.5s')
        edt = ' '.join('%s=%s' % kv for kv in sorted(ds.edt_fresh().items()))
        bds_label.setText('BDShot:   rpm=%-6.0f %-8s EDT:%-3s sent=%.0f/s replies=%.0f/s badcrc=%u %s'
                          % (ds.rpm, 'spinning' if ds.spinning else 'stopped',
                             'on' if ds.edt_active() else 'off',
                             ds.sent.hz(), ds.replies.hz(), ds.badcrc, edt))
        if can is not None:
            can_value_label.setText('%.2f' % (can_value.value() / 1000.0))
            if can.error:
                can_label.setText('DroneCAN: error: %s' % can.error)
            else:
                s = can.status
                can_label.setText(
                    'DroneCAN: rpm=%-6s volt=%-5s cur=%-5s temp=%-5s err=%-3s '
                    'esc.Status=%.0f/s cmds=%.0f/s node=%s up=%us'
                    % (s.get('rpm', '-'),
                       ('%.1f' % s['voltage']) if 'voltage' in s else '-',
                       ('%.1f' % s['current']) if 'current' in s else '-',
                       ('%.0f' % s['temp']) if 'temp' in s else '-',
                       s.get('errors', '-'),
                       can.esc_rate.hz(), can.sent.hz(),
                       can.node_id, can.uptime))
            try:
                param_status.setText(can.param_result.get_nowait())
            except queue.Empty:
                pass
            if can_fps.available and can_fps.dna_request_seen():
                # flash: a node on the bus is asking for an allocation
                if int(time.time() * 2.5) % 2:
                    can_dna_label.setText('DNA request')
                    can_dna_label.setStyleSheet('color: red; font-weight: bold')
                else:
                    can_dna_label.setText('DNA request')
                    can_dna_label.setStyleSheet('color: gray')
            else:
                can_dna_label.setText('')
                can_dna_label.setStyleSheet('')
        model_status.setText(sim.model_status)
        sim_rate_label.setText('%.0f samples/s' % sim.rate.hz())
        if can_fps.available:
            can_fps_label.setText('FPS: %.0f' % can_fps.rate.hz())

    update_timer = QTimer()
    update_timer.timeout.connect(update)
    update_timer.start(100)

    # graceful Ctrl-C / SIGTERM: quit the event loop. The handler runs
    # from the 100ms update timer, the next time python bytecode executes
    signal.signal(signal.SIGINT, lambda *a: app.quit())
    signal.signal(signal.SIGTERM, lambda *a: app.quit())

    win.show()
    try:
        app.exec()
    finally:
        wave['run'] = False
        if tone_synth is not None:      # stop the audio threads cleanly
            tone_synth.stop()
        if phys_player is not None:
            phys_player.stop()
        ds.running = False
        sim.close()
        if can is not None:
            can.running = False
            # let the CAN thread close the node and its IO child
            can.thread.join(2.0)


if __name__ == '__main__':
    main()
