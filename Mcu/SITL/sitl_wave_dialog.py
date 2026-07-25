#!/usr/bin/env python3
'''
throttle waveform override dialog for the SITL GUI.

Drives the throttle as a sine or square wave. The frequency is in
simulated time, so the waveform tracks the speedup slider: at 0.1x it
runs ten times slower on the wall clock but is still the same frequency
in the simulation the firmware sees.
'''

import math

from PySide6.QtCore import Qt, QLineF
from PySide6.QtGui import QColor, QPainter, QPainterPath, QPen, QPixmap
from PySide6.QtWidgets import (QDialog, QDoubleSpinBox, QFormLayout,
                               QHBoxLayout, QLabel, QPushButton, QVBoxLayout)


def wave_pixmap(kind, w=26, h=16):
    '''a little sine or square wave glyph for the override buttons'''
    pm = QPixmap(w, h)
    pm.fill(Qt.transparent)
    p = QPainter(pm)
    p.setRenderHint(QPainter.Antialiasing)
    p.setPen(QPen(QColor(30, 120, 220), 2))
    m = 2
    mid = h / 2.0
    amp = h / 2.0 - m
    x0, x1 = m, w - m
    if kind == 'sine':
        path = QPainterPath()
        n = int(x1 - x0)
        for i in range(n + 1):
            x = x0 + i
            y = mid - amp * math.sin(2 * math.pi * 1.5 * i / n)
            path.moveTo(x, y) if i == 0 else path.lineTo(x, y)
        p.drawPath(path)
    else:
        seg = (x1 - x0) / 3.0
        pts = [(x0, mid + amp), (x0, mid - amp), (x0 + seg, mid - amp),
               (x0 + seg, mid + amp), (x0 + 2 * seg, mid + amp),
               (x0 + 2 * seg, mid - amp), (x1, mid - amp)]
        for a, b in zip(pts, pts[1:]):
            p.drawLine(QLineF(a[0], a[1], b[0], b[1]))
    p.end()
    return pm


class WaveDialog(QDialog):
    '''entry of frequency, amplitude and base throttle for a throttle
    waveform override. Start applies it, Stop cancels it; the dialog
    stays open so the parameters can be tweaked live'''

    def __init__(self, kind, on_apply, on_stop, defaults, icon=None, parent=None):
        super().__init__(parent)
        self.kind = kind
        self.on_apply = on_apply
        self.on_stop = on_stop
        self.setWindowTitle('%s throttle override' % kind.capitalize())

        lay = QVBoxLayout(self)
        head = QHBoxLayout()
        if icon is not None:
            glyph = QLabel()
            glyph.setPixmap(icon)
            head.addWidget(glyph)
        head.addWidget(QLabel('%s wave throttle override' % kind.capitalize()))
        head.addStretch(1)
        lay.addLayout(head)

        form = QFormLayout()
        self.freq = QDoubleSpinBox()
        self.freq.setRange(0.01, 10.0)
        self.freq.setDecimals(2)
        self.freq.setSingleStep(0.1)
        self.freq.setSuffix(' Hz (sim time)')
        self.freq.setValue(defaults.get('freq', 1.0))
        self.freq.setToolTip(
            'Waveform frequency in simulated seconds. Because it is sim\n'
            'time, the waveform follows the speedup slider - at 0.1x it\n'
            'takes ten wall-clock seconds per cycle but is unchanged in\n'
            'the simulation.')
        self.amp = QDoubleSpinBox()
        self.amp.setRange(0.0, 1.0)
        self.amp.setDecimals(2)
        self.amp.setSingleStep(0.05)
        self.amp.setValue(defaults.get('amp', 0.2))
        self.amp.setToolTip('Peak throttle swing either side of the base, 0..1.')
        self.base = QDoubleSpinBox()
        self.base.setRange(0.0, 1.0)
        self.base.setDecimals(2)
        self.base.setSingleStep(0.05)
        self.base.setValue(defaults.get('base', 0.3))
        self.base.setToolTip('Throttle the waveform is centred on, 0..1.')
        form.addRow('Frequency:', self.freq)
        form.addRow('Amplitude:', self.amp)
        form.addRow('Base throttle:', self.base)
        lay.addLayout(form)

        note = QLabel(
            'throttle = base + amplitude * %s(2*pi*f*t), clamped to 0..1.\n'
            'Enable the input and arm at zero throttle before starting; a\n'
            'waveform whose base never reaches zero will not let the ESC arm.'
            % ('sin' if kind == 'sine' else 'square'))
        note.setWordWrap(True)
        lay.addWidget(note)

        btns = QHBoxLayout()
        btns.addStretch(1)
        for text, slot, tip in (
                ('Start', self._start, 'Apply the waveform to the throttle.'),
                ('Stop', lambda: self.on_stop(), 'Stop overriding the throttle.'),
                ('Close', self.accept, 'Close this dialog (leaves the '
                                       'waveform running if started).')):
            b = QPushButton(text)
            b.setToolTip(tip)
            b.clicked.connect(slot)
            btns.addWidget(b)
        lay.addLayout(btns)

    def _start(self):
        self.on_apply(self.kind, self.freq.value(), self.amp.value(),
                      self.base.value())
