#!/usr/bin/env python3
'''
AM32 SITL calibration capture tool

Walks a user through capturing the data needed to build a SITL model of
an ESC/motor combination: connect, describe the rig, run the capture
battery, then plot, fit and write the data set out.

Works over either transport:
  - Betaflight  : DShot/BDShot through an FC's motor test path over MSP
  - DroneCAN    : RawCommand straight to a CAN ESC

Anything it changes on the flight controller or the ESC to make the
capture work (telemetry rates and so on) is recorded and put back when
the session finishes.

with --control-port N the UI can also be driven over a socket, one
command per line, which is how the automated tests exercise it
'''

import argparse
import json
import multiprocessing
import os
import queue
import socket
import sys
import tempfile
import threading
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from PySide6.QtCore import Qt, QObject, QTimer, Signal
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QPushButton, QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox, QTextEdit,
    QTabWidget, QGroupBox, QCheckBox, QFileDialog, QMessageBox, QProgressBar,
    QPlainTextEdit)

import capture_session as cs

APP_NAME = 'AM32 ESC Capture'


class Bridge(QObject):
    '''thread-safe channel from the worker into the UI'''
    log = Signal(str)
    status = Signal(str)
    done = Signal(str, bool)      # step name, ok
    prompt = Signal(str, str, object)   # title, message, done event


class SimplePlot(QWidget):
    """rpm against time, drawn with nothing but QtWidgets.

    pyqtgraph is nicer, but it needs PySide6.QtOpenGL, which several
    distributions package separately - plotting your own capture should
    not depend on that being installed"""

    def __init__(self, title, xs, ys):
        super().__init__()
        self.title, self.xs, self.ys = title, xs, ys
        self.setMinimumHeight(180)

    def paintEvent(self, _ev):
        q = QPainter(self)
        q.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        left, right, top, bottom = 62, 12, 22, 26
        q.fillRect(self.rect(), QColor('#ffffff'))
        q.setPen(QPen(QColor('#333333')))
        q.drawText(left, 15, self.title)
        q.setPen(QPen(QColor('#999999')))
        q.drawRect(left, top, w - left - right, h - top - bottom)
        if not self.xs:
            return
        x0, x1 = min(self.xs), max(self.xs)
        y0, y1 = min(self.ys), max(self.ys)
        xr = (x1 - x0) or 1.0
        yr = (y1 - y0) or 1.0
        pw, ph = w - left - right, h - top - bottom

        def px(x):
            return left + (x - x0) / xr * pw

        def py(y):
            return top + ph - (y - y0) / yr * ph

        q.setPen(QPen(QColor('#666666')))
        q.drawText(4, int(py(y1)) + 4, '%.0f' % y1)
        q.drawText(4, int(py(y0)) + 4, '%.0f' % y0)
        q.drawText(left, h - 8, '%.1fs' % x0)
        q.drawText(w - right - 44, h - 8, '%.1fs' % x1)
        q.setPen(QPen(QColor('#3399ff'), 1))
        cols = max(1, int(pw))
        if len(self.xs) <= cols:
            prev = None
            for i in range(len(self.xs)):
                pt = (px(self.xs[i]), py(self.ys[i]))
                if prev is not None:
                    q.drawLine(int(prev[0]), int(prev[1]),
                               int(pt[0]), int(pt[1]))
                prev = pt
            return
        # more samples than columns: draw each column from its lowest
        # to its highest sample. Taking every Nth sample instead turns
        # an oscillating capture such as the chirp into a moire pattern
        # rather than showing its envelope
        lo = [None] * cols
        hi = [None] * cols
        for i, x in enumerate(self.xs):
            c = int((x - x0) / xr * (cols - 1))
            y = self.ys[i]
            if lo[c] is None or y < lo[c]:
                lo[c] = y
            if hi[c] is None or y > hi[c]:
                hi[c] = y
        prev = None
        for c in range(cols):
            if lo[c] is None:
                continue
            x = left + c
            top_y, bot_y = int(py(hi[c])), int(py(lo[c]))
            q.drawLine(x, top_y, x, bot_y)
            # bridge any column with no samples in it
            if prev is not None and prev[0] < x - 1:
                q.drawLine(prev[0], prev[1], x, top_y)
            prev = (x, bot_y)


class MainWindow(QWidget):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.session = None
        self.worker = None
        self.bridge = Bridge()
        self.bridge.log.connect(self.append_log)
        self.bridge.status.connect(self.set_status)
        self.bridge.done.connect(self.step_done)
        self.bridge.prompt.connect(self.show_prompt)
        self.pending_prompt = None
        self.prompt_box = None
        self.results = {}
        self.setWindowTitle(APP_NAME)
        self._build()
        if args.dir:
            self.dir_edit.setText(args.dir)

    # ---------------------------------------------------------------- UI

    def _build(self):
        top = QVBoxLayout(self)
        self.tabs = QTabWidget()
        top.addWidget(self.tabs)
        self.tabs.addTab(self._tab_setup(), '1. Setup')
        self.tabs.addTab(self._tab_capture(), '2. Capture')
        self.tabs.addTab(self._tab_analyse(), '3. Analyse && export')

        self.status_label = QLabel('not connected')
        self.status_label.setStyleSheet('font-weight: bold')
        top.addWidget(self.status_label)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(5000)
        top.addWidget(self.log_view, 1)
        self.resize(940, 800)

    def _tab_setup(self):
        w = QWidget()
        g = QVBoxLayout(w)

        conn = QGroupBox('Connection')
        cg = QGridLayout(conn)
        self.transport = QComboBox()
        self.transport.addItems(['Betaflight (DShot)', 'DroneCAN'])
        self.transport.currentIndexChanged.connect(self._transport_changed)
        cg.addWidget(QLabel('Transport'), 0, 0)
        cg.addWidget(self.transport, 0, 1)

        self.port_edit = QComboBox()
        self.port_edit.setEditable(True)
        self.port_edit.lineEdit().setPlaceholderText(
            'auto-detect, or pick a port')
        cg.addWidget(QLabel('FC port'), 1, 0)
        cg.addWidget(self.port_edit, 1, 1)

        self.uri_edit = QComboBox()
        self.uri_edit.setEditable(True)
        cg.addWidget(QLabel('CAN URI'), 2, 0)
        cg.addWidget(self.uri_edit, 2, 1)

        refresh = QPushButton('Refresh ports')
        refresh.clicked.connect(self.refresh_ports)
        cg.addWidget(refresh, 2, 2)

        self.motor_spin = QSpinBox()
        self.motor_spin.setRange(1, 8)
        self.motor_spin.setValue(1)
        cg.addWidget(QLabel('Motor output'), 3, 0)
        cg.addWidget(self.motor_spin, 3, 1)

        self.node_spin = QSpinBox()
        self.node_spin.setRange(1, 127)
        self.node_spin.setValue(123)
        cg.addWidget(QLabel('CAN node id'), 4, 0)
        cg.addWidget(self.node_spin, 4, 1)

        self.poles_spin = QSpinBox()
        self.poles_spin.setRange(2, 64)
        self.poles_spin.setValue(14)
        cg.addWidget(QLabel('Motor poles'), 5, 0)
        cg.addWidget(self.poles_spin, 5, 1)

        self.connect_btn = QPushButton('Connect')
        self.connect_btn.clicked.connect(self.do_connect)
        cg.addWidget(self.connect_btn, 6, 0, 1, 2)
        g.addWidget(conn)

        out = QGroupBox('Output directory')
        og = QHBoxLayout(out)
        self.dir_edit = QLineEdit()
        self.dir_edit.setPlaceholderText('where the data set will be written')
        og.addWidget(self.dir_edit, 1)
        b = QPushButton('Browse...')
        b.clicked.connect(self.pick_dir)
        og.addWidget(b)
        b2 = QPushButton('New folder')
        b2.clicked.connect(self.new_dir)
        og.addWidget(b2)
        g.addWidget(out)

        meta = QGroupBox('Describe the rig (goes into README.md)')
        mg = QGridLayout(meta)
        self.meta_edits = {}
        rows = [('esc', 'ESC model'), ('motor', 'Motor model'),
                ('prop', 'Propeller (or "none")'),
                ('power', 'Power source'),
                ('fc', 'Flight controller'),
                ('notes', 'Notes')]
        for i, (key, label) in enumerate(rows):
            e = QLineEdit()
            self.meta_edits[key] = e
            mg.addWidget(QLabel(label), i, 0)
            mg.addWidget(e, i, 1)
        g.addWidget(meta)
        g.addStretch(1)
        self.refresh_ports()
        self._transport_changed()
        return w

    def _tab_capture(self):
        w = QWidget()
        g = QVBoxLayout(w)

        warn = QLabel('Strap the motor down carefully if props are on, '
                      'and keep a way to cut power within reach.')
        warn.setStyleSheet('color: #b00; font-weight: bold')
        warn.setWordWrap(True)
        g.addWidget(warn)

        opts = QGroupBox('Limits')
        og = QGridLayout(opts)
        self.max_current = QDoubleSpinBox()
        self.max_current.setRange(0.5, 200.0)
        self.max_current.setValue(5.0)
        self.max_current.setSuffix(' A')
        og.addWidget(QLabel('Abort above'), 0, 0)
        og.addWidget(self.max_current, 0, 1)
        self.max_throttle = QDoubleSpinBox()
        self.max_throttle.setRange(0.05, 1.0)
        self.max_throttle.setSingleStep(0.05)
        self.max_throttle.setValue(0.6)
        og.addWidget(QLabel('Max throttle'), 0, 2)
        og.addWidget(self.max_throttle, 0, 3)
        self.chirp_secs = QSpinBox()
        self.chirp_secs.setRange(20, 300)
        self.chirp_secs.setValue(120)
        self.chirp_secs.setSuffix(' s')
        og.addWidget(QLabel('Chirp length'), 0, 4)
        og.addWidget(self.chirp_secs, 0, 5)
        g.addWidget(opts)

        steps = QGroupBox('Capture battery')
        sg = QGridLayout(steps)
        self.step_boxes = {}
        self.step_btns = {}
        for i, (key, label) in enumerate(cs.STEPS):
            cb = QCheckBox(label)
            cb.setChecked(True)
            self.step_boxes[key] = cb
            sg.addWidget(cb, i, 0)
            btn = QPushButton('Run')
            btn.clicked.connect(lambda _=False, k=key: self.run_step(k))
            self.step_btns[key] = btn
            sg.addWidget(btn, i, 1)
        g.addWidget(steps)

        row = QHBoxLayout()
        self.run_all_btn = QPushButton('Run selected steps')
        self.run_all_btn.clicked.connect(self.run_all)
        row.addWidget(self.run_all_btn)
        self.abort_btn = QPushButton('Abort')
        self.abort_btn.clicked.connect(self.do_abort)
        self.abort_btn.setEnabled(False)
        row.addWidget(self.abort_btn)
        g.addLayout(row)
        self.progress = QProgressBar()
        g.addWidget(self.progress)
        g.addStretch(1)
        return w

    def _tab_analyse(self):
        w = QWidget()
        g = QVBoxLayout(w)
        row = QHBoxLayout()
        b = QPushButton('Plot captures')
        b.clicked.connect(self.do_plot)
        row.addWidget(b)
        b = QPushButton('Fit')
        b.clicked.connect(self.do_fit)
        row.addWidget(b)
        b = QPushButton('Write data set (README, expected.json, model)')
        b.clicked.connect(self.do_export)
        row.addWidget(b)
        g.addLayout(row)
        self.result_view = QTextEdit()
        self.result_view.setReadOnly(True)
        g.addWidget(self.result_view, 1)
        self.plot_holder = QVBoxLayout()
        g.addLayout(self.plot_holder, 2)
        return w

    # ------------------------------------------------------------ helpers

    def refresh_ports(self):
        """re-scan the serial ports, keeping whatever the user typed"""
        keep_port = self.port_edit.currentText().strip()
        keep_uri = self.uri_edit.currentText().strip()
        self.port_edit.clear()
        self.port_edit.addItem('')          # blank = auto-detect
        for label, dev in cs.serial_candidates():
            self.port_edit.addItem(label, dev)
        self.uri_edit.clear()
        for label, uri in cs.can_uri_candidates():
            self.uri_edit.addItem(label, uri)
        if keep_port:
            self.port_edit.setCurrentText(keep_port)
        if keep_uri:
            self.uri_edit.setCurrentText(keep_uri)
        elif self.uri_edit.count():
            self.uri_edit.setCurrentIndex(0)

    def _selected(self, combo):
        """the value behind the shown label, or the typed text"""
        idx = combo.findText(combo.currentText())
        if idx >= 0 and combo.itemData(idx):
            return combo.itemData(idx)
        return combo.currentText().strip()

    def _transport_changed(self):
        bf = self.transport.currentIndex() == 0
        self.port_edit.setEnabled(bf)
        self.motor_spin.setEnabled(bf)
        self.uri_edit.setEnabled(not bf)
        self.node_spin.setEnabled(not bf)

    def append_log(self, text):
        for line in text.rstrip('\n').split('\n'):
            self.log_view.appendPlainText(line)

    def set_status(self, text):
        self.status_label.setText(text)

    def ask(self, title, message):
        '''called from a worker: block it until the user has answered.

        Emitting alone only queues the dialog, so the capture used to
        carry on while the user was still being told to power cycle'''
        done = threading.Event()
        self.bridge.prompt.emit(title, message, done)
        while not done.wait(0.2):
            if self.session is not None and self.session.aborting:
                return

    def show_prompt(self, title, message, done=None):
        self.pending_prompt = (title, message)
        box = QMessageBox(QMessageBox.Information, title, message,
                          QMessageBox.Ok, self)
        self.prompt_box = box
        try:
            box.exec()
        finally:
            self.prompt_box = None
            self.pending_prompt = None
            if done is not None:
                done.set()

    def pick_dir(self):
        d = QFileDialog.getExistingDirectory(self, 'Data set directory',
                                             self.dir_edit.text() or os.getcwd())
        if d:
            self.dir_edit.setText(d)

    def new_dir(self):
        base = self.dir_edit.text() or os.getcwd()
        d, ok = QFileDialog.getSaveFileName(self, 'Create data set directory', base)
        if d:
            os.makedirs(d, exist_ok=True)
            self.dir_edit.setText(d)

    def config(self):
        return cs.Config(
            transport='betaflight' if self.transport.currentIndex() == 0 else 'dronecan',
            port=self._selected(self.port_edit) or None,
            uri=self._selected(self.uri_edit),
            motor=self.motor_spin.value(),
            node_id=self.node_spin.value(),
            poles=self.poles_spin.value(),
            directory=self.dir_edit.text().strip(),
            max_current=self.max_current.value(),
            max_throttle=self.max_throttle.value(),
            chirp_duration=float(self.chirp_secs.value()),
            meta={k: e.text() for k, e in self.meta_edits.items()})

    def busy(self, on):
        self.run_all_btn.setEnabled(not on)
        self.connect_btn.setEnabled(not on)
        for b in self.step_btns.values():
            b.setEnabled(not on)
        self.abort_btn.setEnabled(on)
        self.progress.setRange(0, 0 if on else 1)
        if not on:
            self.progress.reset()

    # ------------------------------------------------------------ actions

    def busy_now(self):
        '''a second capture would command the motor behind the first'''
        if self.run_all_btn.isEnabled():
            return False
        self.append_log('busy - abort the running step first')
        return True

    def do_connect(self):
        if self.busy_now():
            return
        if not self.dir_edit.text().strip():
            self.append_log('choose an output directory first')
            self.dir_edit.setFocus()
            return
        cfg = self.config()
        self.session = cs.Session(cfg, self.bridge.log.emit, self.ask)
        self.busy(True)

        def work():
            ok, msg = False, ''
            try:
                msg = self.session.connect()
                ok = True
            except BaseException as ex:
                msg = str(ex)
                self.bridge.log.emit('connect failed: %s' % ex)
            self.bridge.done.emit('connect:' + msg, ok)
        self._start(work)

    def run_step(self, key):
        if not self._ready() or self.busy_now():
            return
        self.session.aborting = False
        self.busy(True)

        def work():
            ok = False
            try:
                self.session.run_step(key)
                ok = True
            except BaseException as ex:
                if self.session.aborting:
                    self.bridge.log.emit('%s aborted' % key)
                else:
                    self.bridge.log.emit('%s failed: %s' % (key, ex))
                    self.bridge.log.emit(traceback.format_exc(limit=3))
            self.bridge.done.emit(key, ok)
        self._start(work)

    def run_all(self):
        if not self._ready() or self.busy_now():
            return
        keys = [k for k, _ in cs.STEPS if self.step_boxes[k].isChecked()]
        self.session.aborting = False
        self.busy(True)

        def work():
            ok = True
            for k in keys:
                if self.session.aborting:
                    ok = False
                    break
                try:
                    self.session.run_step(k)
                except BaseException as ex:
                    self.bridge.log.emit(
                        '%s aborted' % k if self.session.aborting
                        else '%s failed: %s' % (k, ex))
                    ok = False
                    break
            self.bridge.done.emit('battery', ok)
        self._start(work)

    def do_abort(self):
        if self.session:
            self.session.abort()
            self.append_log('abort requested')

    def do_plot(self):
        if not self.session:
            self.append_log('connect and capture first')
            return
        try:
            import pyqtgraph as pg
        except BaseException as ex:
            pg = None
            self.append_log('pyqtgraph unavailable (%s), using the built-in '
                            'plot' % ex)
        while self.plot_holder.count():
            item = self.plot_holder.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        series = self.session.plot_series()
        if not series:
            self.append_log('no captures to plot yet')
            return
        for title, xs, ys in series:
            if pg is None:
                self.plot_holder.addWidget(SimplePlot(title, xs, ys))
                continue
            p = pg.PlotWidget(title=title)
            p.plot(xs, ys, pen=pg.mkPen('#39f', width=1))
            p.setLabel('bottom', 'time', units='s')
            p.setLabel('left', 'rpm')
            self.plot_holder.addWidget(p)
        self.append_log('plotted %u capture(s)' % len(series))

    def do_fit(self):
        if not self._ready(need_dir_only=True):
            return
        self.busy(True)

        def work():
            ok = False
            try:
                self.results = self.session.fit()
                self.bridge.log.emit(json.dumps(self.results, indent=2))
                ok = True
            except BaseException as ex:
                self.bridge.log.emit('fit failed: %s' % ex)
            self.bridge.done.emit('fit', ok)
        self._start(work)

    def do_export(self):
        if not self._ready(need_dir_only=True):
            return
        try:
            paths = self.session.export(self.results)
            self.append_log('wrote:\n  ' + '\n  '.join(paths))
            self.result_view.setPlainText(self.session.readme_text(self.results))
        except Exception as ex:
            self.append_log('export failed: %s' % ex)

    def _ready(self, need_dir_only=False):
        if not self.dir_edit.text().strip():
            self.append_log('choose an output directory first')
            return False
        if need_dir_only:
            if self.session is None:
                self.session = cs.Session(self.config(), self.bridge.log.emit,
                                          self.ask)
            return True
        if self.session is None or not self.session.connected:
            self.append_log('connect first')
            return False
        return True

    def _start(self, fn):
        self.worker = threading.Thread(target=fn, daemon=True)
        self.worker.start()

    def step_done(self, name, ok):
        self.busy(False)
        if ok:
            state = 'ok'
        elif self.session is not None and self.session.aborting:
            state = 'ABORTED'
        else:
            state = 'FAILED'
        self.append_log('== %s %s ==' % (name.split(':')[0], state))
        if name.startswith('connect:') and ok:
            self.set_status(name.split(':', 1)[1])

    def closeEvent(self, ev):
        if self.session:
            try:
                self.session.restore()
            except Exception as ex:
                print('restore failed: %s' % ex)
        ev.accept()

    # ------------------------------------------------------- control port

    def handle_command(self, line, reply):
        parts = line.split()
        if not parts:
            return
        cmd, a = parts[0], parts[1:]
        try:
            if cmd == 'transport':
                self.transport.setCurrentIndex(0 if a[0].startswith('beta') else 1)
            elif cmd == 'port':
                self.port_edit.setCurrentText(a[0])
            elif cmd == 'uri':
                self.uri_edit.setCurrentText(a[0])
            elif cmd == 'motor':
                self.motor_spin.setValue(int(a[0]))
            elif cmd == 'node':
                self.node_spin.setValue(int(a[0]))
            elif cmd == 'poles':
                self.poles_spin.setValue(int(a[0]))
            elif cmd == 'dir':
                os.makedirs(a[0], exist_ok=True)
                self.dir_edit.setText(a[0])
            elif cmd == 'meta':
                self.meta_edits[a[0]].setText(' '.join(a[1:]))
            elif cmd == 'max_current':
                self.max_current.setValue(float(a[0]))
            elif cmd == 'chirp_secs':
                self.chirp_secs.setValue(int(a[0]))
            elif cmd == 'select':
                for k in self.step_boxes:
                    self.step_boxes[k].setChecked(k in a)
            elif cmd == 'connect':
                self.do_connect()
            elif cmd == 'run':
                self.run_step(a[0])
            elif cmd == 'run_all':
                self.run_all()
            elif cmd == 'fit':
                self.do_fit()
            elif cmd == 'plot':
                self.do_plot()
            elif cmd == 'export':
                self.do_export()
            elif cmd == 'abort':
                self.do_abort()
            elif cmd == 'prompt':
                reply('prompt %s' % ('|'.join(self.pending_prompt)
                                     if self.pending_prompt else ''))
                return
            elif cmd == 'testprompt':
                title, _, msg = ' '.join(a).partition('|')
                self.busy(True)

                def work(t=title, m=msg):
                    self.ask(t, m)
                    self.bridge.done.emit('prompt', True)
                self._start(work)
            elif cmd == 'ack':
                if self.prompt_box is not None:
                    self.prompt_box.accept()
            elif cmd == 'restore':
                self.session and self.session.restore()
            elif cmd == 'busy':
                reply('busy %d' % (0 if self.run_all_btn.isEnabled() else 1))
                return
            elif cmd == 'status':
                reply('status %s' % self.status_label.text())
                return
            elif cmd == 'results':
                reply('results %s' % json.dumps(self.results))
                return
            elif cmd == 'log':
                reply('log %s' % self.log_view.toPlainText().replace('\n', '\\n'))
                return
            elif cmd == 'quit':
                reply('OK quit')
                QApplication.instance().quit()
                return
            else:
                reply('ERR unknown %s' % cmd)
                return
            reply('OK %s' % line.strip())
        except Exception as ex:
            reply('ERR %s: %s' % (cmd, ex))


def _ensure_stdio():
    """a --windowed frozen build has no console, so sys.stdout and
    sys.stderr are None. Anything that writes to them then dies with
    AttributeError - including multiprocessing's own crash handler,
    which turns a child process error into a meaningless traceback"""
    for name in ('stdout', 'stderr'):
        if getattr(sys, name, None) is not None:
            continue
        try:
            f = open(os.path.join(tempfile.gettempdir(), 'am32-capture.log'),
                     'a', buffering=1)
        except Exception:
            f = open(os.devnull, 'w')
        setattr(sys, name, f)


def main():
    # must come first: the child process re-enters here, and a crash
    # while reporting a crash tells the user nothing
    _ensure_stdio()
    # the DroneCAN mavcan driver runs its IO in a child process. In a
    # packaged single-file build that child re-executes this binary, so
    # without this it starts a second copy of the GUI instead of the
    # worker and the link dies with a connection reset
    multiprocessing.freeze_support()
    # Python 3.14 no longer defaults to fork on Linux, and the other
    # methods re-execute this binary for every child - which for a
    # packaged build means a fresh copy of the whole GUI each time, so
    # the DroneCAN IO child never runs and the app multiplies instead
    if sys.platform.startswith('linux'):
        try:
            multiprocessing.set_start_method('fork')
        except RuntimeError:
            pass

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--control-port', type=int, default=0,
                    help='accept UI commands on this TCP port (for testing)')
    ap.add_argument('--dir', default=None, help='output directory')
    args = ap.parse_args()

    app = QApplication(sys.argv)
    win = MainWindow(args)
    win.show()

    if args.control_port > 0:
        cmd_queue = queue.Queue()

        def client(conn):
            def reply(msg):
                try:
                    conn.sendall((msg + '\n').encode())
                except OSError:
                    pass
            f = conn.makefile('r')
            for line in f:
                cmd_queue.put((line, reply))
            conn.close()

        def server():
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind(('127.0.0.1', args.control_port))
            srv.listen(4)
            while True:
                conn, _ = srv.accept()
                threading.Thread(target=client, args=(conn,), daemon=True).start()

        threading.Thread(target=server, daemon=True).start()

        def pump():
            while True:
                try:
                    line, reply = cmd_queue.get_nowait()
                except queue.Empty:
                    return
                win.handle_command(line, reply)
        t = QTimer()
        t.timeout.connect(pump)
        t.start(50)
        win._pump_timer = t

    sys.exit(app.exec())


if __name__ == '__main__':
    main()
