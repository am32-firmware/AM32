#!/usr/bin/env python3
'''
parameter editor for the simulated ESC

Edits the eeprom directly over the SITL state port, bypassing the
4-way and DroneCAN parameter paths - the simulator is the bench, so
settings are changed the way a technician reflashes them. Parameters
whose value has to agree with the simulated motor (Kv, pole count)
are highlighted when they disagree, but wrong values can still be
set: testing a misconfigured ESC is a legitimate thing to simulate.
'''

from PySide6.QtCore import Qt
from PySide6.QtGui import QBrush, QColor
from PySide6.QtWidgets import (QDialog, QHBoxLayout, QHeaderView, QLabel,
                               QPushButton, QSpinBox, QTableWidget,
                               QTableWidgetItem, QVBoxLayout)

import sitl_params

MISMATCH_BG = QColor(255, 214, 214)
CHANGED_BG = QColor(255, 246, 200)


class ParamDialog(QDialog):
    def __init__(self, eeprom_client, parent=None):
        super().__init__(parent)
        self.client = eeprom_client
        self.image = None
        self.model = None
        self.spins = {}
        self.reset_btns = {}
        self.setWindowTitle('ESC parameters (simulated eeprom)')
        self.resize(760, 620)

        lay = QVBoxLayout(self)
        self.info = QLabel('')
        self.info.setWordWrap(True)
        lay.addWidget(self.info)

        self.table = QTableWidget(len(sitl_params.PARAMS), 7, self)
        self.table.setHorizontalHeaderLabels(
            ['parameter', 'raw value', '', 'value', 'range', 'default',
             'description'])
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(
            6, QHeaderView.Stretch)
        lay.addWidget(self.table)

        btns = QHBoxLayout()
        self.status = QLabel('')
        btns.addWidget(self.status, 1)
        for text, slot, tip in (
                ('Refresh', self.refresh, 'Re-read the eeprom from the simulator.'),
                ('Match model', self.match_model,
                 'Set the model-critical parameters (Kv, poles) to what the\n'
                 'simulated motor needs.'),
                ('All defaults', self.all_defaults,
                 'Set every parameter to its default; for the model-linked\n'
                 'parameters (Kv, poles) the default is what the simulated\n'
                 'motor needs. Nothing is written until Apply.'),
                ('Apply', self.apply, 'Write changed values to the simulated eeprom.'),
                ('Close', self.accept, '')):
            b = QPushButton(text)
            if tip:
                b.setToolTip(tip)
            b.clicked.connect(slot)
            btns.addWidget(b)
        lay.addLayout(btns)

        self.build_rows()
        self.refresh()

    def build_rows(self):
        for row, (off, name, lo, hi, dflt, helptext) in enumerate(
                sitl_params.PARAMS):
            item = QTableWidgetItem(name)
            item.setFlags(item.flags() & ~Qt.ItemIsEditable)
            item.setToolTip('eeprom offset %d' % off)
            self.table.setItem(row, 0, item)

            spin = QSpinBox()
            spin.setRange(0, 255)
            spin.setToolTip(helptext)
            spin.valueChanged.connect(self.value_changed)
            self.table.setCellWidget(row, 1, spin)
            self.spins[name] = spin

            # per-row reset: enabled (and coloured) only while the value
            # differs from the default, so it doubles as the non-default
            # indicator
            btn = QPushButton('↺')
            btn.setFixedWidth(26)
            btn.clicked.connect(lambda _=False, n=name: self.reset_one(n))
            self.table.setCellWidget(row, 2, btn)
            self.reset_btns[name] = btn

            calc = QTableWidgetItem('')
            calc.setFlags(calc.flags() & ~Qt.ItemIsEditable)
            calc.setForeground(QBrush(QColor(40, 90, 160)))
            self.table.setItem(row, 3, calc)

            for col, text in ((4, '%d..%d' % (lo, hi)), (5, str(dflt)),
                              (6, helptext.replace('\n', ' '))):
                it = QTableWidgetItem(text)
                it.setFlags(it.flags() & ~Qt.ItemIsEditable)
                self.table.setItem(row, col, it)

    def refresh(self):
        image, model = self.client.fetch()
        if image is None:
            self.status.setText('no reply from the simulator state port')
            return
        self.image = bytearray(image)
        self.model = model
        for name, spin in self.spins.items():
            off = sitl_params.PARAMS_BY_NAME[name][0]
            spin.blockSignals(True)
            spin.setValue(self.image[off] if off < len(self.image) else 0)
            spin.blockSignals(False)
        kv = (model or {}).get('kv', 0)
        poles = (model or {}).get('poles', 0)
        self.info.setText(
            'Simulated motor: %.0f Kv, %d poles. Highlighted rows disagree '
            'with it - the firmware scales low rpm power protection from '
            'MOTOR_KV and every rpm from MOTOR_POLES, so a mismatch makes '
            'the motor behave oddly. Values are the raw stored bytes; '
            'MOTOR_KV holds (Kv-20)/40.' % (kv, poles))
        self.status.setText('read %d bytes' % len(self.image))
        self.recolour()
        # size to the populated calculated column, not the empty one it
        # was at build time; keep the description column stretching
        for col in range(6):
            self.table.resizeColumnToContents(col)
        self.table.setColumnWidth(3, max(self.table.columnWidth(3), 110))

    def value_changed(self):
        self.recolour()

    def recolour(self):
        checks = sitl_params.model_checks(self.model)
        for row, (off, name, lo, hi, dflt, _h) in enumerate(
                sitl_params.PARAMS):
            spin = self.spins[name]
            item = self.table.item(row, 0)
            self.table.item(row, 3).setText(
                sitl_params.calc_value(name, spin.value()))
            # the default column and the per-row reset track the model
            # for the model-linked parameters
            dval, from_model = self.default_for(name)
            self.table.item(row, 5).setText(
                '%d (model)' % dval if from_model else str(dval))
            btn = self.reset_btns[name]
            at_default = spin.value() == dval
            btn.setEnabled(not at_default)
            btn.setStyleSheet('' if at_default
                              else 'color: #1560c0; font-weight: bold')
            btn.setToolTip(
                'at the default (%d)' % dval if at_default else
                'not at the default: reset to %d%s' %
                (dval, ' (from the motor model)' if from_model else ''))
            stored = self.image[off] if (self.image and off < len(self.image)) else None
            tips = []
            colour = None
            if name in checks:
                want, why = checks[name]
                if spin.value() != want:
                    colour = MISMATCH_BG
                    tips.append('does not match the simulated motor '
                                '(needs %d)\n%s' % (want, why))
            if stored is not None and spin.value() != stored:
                colour = colour or CHANGED_BG
                tips.append('changed from %d, not yet applied' % stored)
            item.setBackground(QBrush(colour) if colour
                               else QBrush(Qt.transparent))
            item.setToolTip('eeprom offset %d\n%s' % (off, '\n'.join(tips))
                            if tips else 'eeprom offset %d' % off)

    def default_for(self, name):
        '''the default byte for a parameter: the motor model's required
        value for the model-linked ones (Kv, poles), the table default
        otherwise. Returns (value, from_model)'''
        checks = sitl_params.model_checks(self.model)
        if name in checks:
            return checks[name][0], True
        return sitl_params.PARAMS_BY_NAME[name][4], False

    def reset_one(self, name):
        dflt, _ = self.default_for(name)
        self.spins[name].setValue(dflt)
        self.recolour()

    def all_defaults(self):
        changed = 0
        for name, spin in self.spins.items():
            dflt, _ = self.default_for(name)
            if spin.value() != dflt:
                spin.setValue(dflt)
                changed += 1
        self.recolour()
        self.status.setText(
            'set %d parameter%s to defaults - Apply to write'
            % (changed, '' if changed == 1 else 's') if changed
            else 'everything already at defaults')

    def match_model(self):
        for name, (want, _why) in sitl_params.model_checks(self.model).items():
            if name in self.spins:
                self.spins[name].setValue(want)
        self.recolour()

    def apply(self):
        if self.image is None:
            return
        wrote = 0
        for name, spin in self.spins.items():
            off = sitl_params.PARAMS_BY_NAME[name][0]
            if off >= len(self.image) or spin.value() == self.image[off]:
                continue
            ok, msg = self.client.set(off, bytes([spin.value()]))
            if not ok:
                self.status.setText('%s: %s' % (name, msg))
                return
            self.image[off] = spin.value()
            wrote += 1
        self.status.setText(
            'applied %d parameter%s - restart the ESC for settings read at '
            'boot (MOTOR_KV, MOTOR_POLES, INPUT_SIGNAL_TYPE)'
            % (wrote, '' if wrote == 1 else 's') if wrote
            else 'nothing changed')
        self.recolour()
