#!/usr/bin/env python3
'''
AM32 eeprom parameter table for the SITL GUI parameter editor

Mirrors the layout in Inc/eeprom.h (offsets are byte positions in the
192 byte eeprom image). Reserved slots and the 128 byte startup tune
are not listed.

Each entry is (offset, name, minimum, maximum, default, description).
Values are single bytes as stored; where the firmware scales a stored
byte (MOTOR_KV) the conversion lives in MODEL_CHECKS so the editor can
show what the simulated motor needs.
'''

import os
import re


def _version_defines():
    '''VERSION_MAJOR/MINOR and EEPROM_VERSION from Inc/version.h so the
    defaults below track the firmware instead of being duplicated'''
    d = {'VERSION_MAJOR': 2, 'VERSION_MINOR': 20, 'EEPROM_VERSION': 3}
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', '..', 'Inc', 'version.h')
    try:
        for line in open(path):
            m = re.match(r'\s*#define\s+(\w+)\s+(\d+)', line)
            if m and m.group(1) in d:
                d[m.group(1)] = int(m.group(2))
    except OSError:
        pass
    return d


_VER = _version_defines()

# offset, name, min, max, default, help
PARAMS = [
    (1, 'EEPROM_VERSION', 0, 255, _VER['EEPROM_VERSION'], 'settings layout version'),
    (3, 'VERSION_MAJOR', 0, 255, _VER['VERSION_MAJOR'], 'firmware major version'),
    (4, 'VERSION_MINOR', 0, 255, _VER['VERSION_MINOR'], 'firmware minor version'),
    (5, 'MAX_RAMP', 1, 255, 160, 'duty ramp limit, 0.1%/ms to 25%/ms'),
    (6, 'MIN_DUTY_CYCLE', 0, 50, 1, 'minimum duty, 0.2% to 51%'),
    (7, 'DISABLE_STICK_CALIBRATION', 0, 1, 0, 'ignore throttle range learning'),
    (8, 'ABSOLUTE_VOLTAGE_CUTOFF', 0, 100, 10, 'cutoff voltage in 0.5V steps'),
    (9, 'CURRENT_P', 0, 255, 100, 'current limiter P gain'),
    (10, 'CURRENT_I', 0, 255, 0, 'current limiter I gain'),
    (11, 'CURRENT_D', 0, 255, 100, 'current limiter D gain'),
    (12, 'ACTIVE_BRAKE_POWER', 0, 10, 0, 'active braking strength, percent duty'),
    (17, 'DIR_REVERSED', 0, 1, 0, 'reverse the motor direction'),
    (18, 'BI_DIRECTIONAL', 0, 1, 0, '3D mode: throttle range splits fwd/rev'),
    (19, 'USE_SINE_START', 0, 1, 0, 'sine wave startup'),
    (20, 'COMP_PWM', 0, 1, 1, 'complementary PWM (active freewheeling)'),
    (21, 'VARIABLE_PWM', 0, 1, 1, 'scale PWM frequency with rpm'),
    (22, 'STUCK_ROTOR_PROTECTION', 0, 1, 1, 'cut power when the rotor will not turn'),
    (23, 'ADVANCE_LEVEL', 0, 31, 16, 'commutation timing advance'),
    (24, 'PWM_FREQUENCY', 8, 48, 24, 'PWM carrier, kHz'),
    (25, 'STARTUP_POWER', 50, 150, 100, 'startup duty, percent'),
    (26, 'MOTOR_KV', 0, 255, 55, 'motor Kv, stored as (kv-20)/40'),
    (27, 'MOTOR_POLES', 2, 64, 14, 'magnet poles (not pole pairs)'),
    (28, 'BRAKE_ON_STOP', 0, 1, 0, 'brake when throttle is zero'),
    (29, 'STALL_PROTECTION', 0, 1, 0, 'restart on stall'),
    (30, 'BEEP_VOLUME', 0, 11, 5, 'startup and beacon tone volume'),
    (31, 'TELEMETRY_ON_INTERVAL', 0, 1, 0, 'serial telemetry without a request'),
    (32, 'SERVO_LOW_THRESHOLD', 0, 255, 128, 'servo pulse low end'),
    (33, 'SERVO_HIGH_THRESHOLD', 0, 255, 128, 'servo pulse high end'),
    (34, 'SERVO_NEUTRAL', 0, 255, 128, 'servo neutral position'),
    (35, 'SERVO_DEAD_BAND', 0, 100, 50, 'servo dead band'),
    (36, 'LOW_VOLTAGE_CUTOFF', 0, 2, 0, '0=off 1=per cell 2=absolute'),
    (37, 'LOW_CELL_VOLTAGE', 0, 100, 50, 'per cell cutoff, 2.5V + this/100'),
    (38, 'RC_CAR_REVERSE', 0, 1, 0, 'brake then reverse on reverse throttle'),
    (39, 'USE_HALL_SENSORS', 0, 1, 0, 'sensored commutation'),
    (40, 'SINE_MODE_CHANGEOVER', 0, 255, 5, 'throttle level leaving sine start'),
    (41, 'DRAG_BRAKE_STRENGTH', 0, 10, 10, 'brake strength at zero throttle'),
    (42, 'DRIVING_BRAKE_STRENGTH', 0, 10, 10, 'brake strength while driving'),
    (43, 'TEMPERATURE_LIMIT', 70, 255, 255, 'thermal limit, C (255 = off)'),
    (44, 'CURRENT_LIMIT', 0, 100, 102, 'current limit in 2A steps (>100 = off)'),
    (45, 'SINE_MODE_POWER', 1, 10, 5, 'sine startup power'),
    (46, 'INPUT_SIGNAL_TYPE', 0, 5, 5, '0=auto 1=dshot 2=servo 5=dronecan only'),
    (47, 'AUTO_ADVANCE', 0, 1, 0, 'automatic timing advance'),
    (176, 'CAN_NODE', 0, 127, 0, 'DroneCAN node id (0 = dynamic allocation)'),
    (177, 'ESC_INDEX', 0, 31, 0, 'index into esc.RawCommand'),
    (178, 'REQUIRE_ARMING', 0, 1, 1, 'require an arming message before running'),
    (179, 'TELEM_RATE', 0, 200, 25, 'esc.Status rate, Hz'),
    (180, 'REQUIRE_ZERO_THROTTLE', 0, 1, 0, 'require zero throttle at boot'),
    (181, 'FILTER_HZ', 0, 255, 20, 'DroneCAN input filter cutoff, Hz'),
    (182, 'DEBUG_RATE', 0, 200, 0, 'FlexDebug rate, Hz'),
    (183, 'TERM_ENABLE', 0, 1, 0, 'serial terminal'),
]

PARAMS_BY_NAME = dict((p[1], p) for p in PARAMS)


def _onoff(v):
    return 'on' if v else 'off'


# calculated (engineering) value for a stored byte, matching the
# firmware's own conversion. Only parameters with a non-trivial scaling
# or a meaningful enumeration are listed; the rest show nothing in the
# calculated column (the raw byte is the value). Keep these in step with
# Src/main.c
CALC = {
    'MOTOR_KV': lambda v: '%d Kv' % byte_to_kv(v),
    'MOTOR_POLES': lambda v: '%d poles (%d pole pairs)' % (v, v // 2),
    'MIN_DUTY_CYCLE': lambda v: '%.1f%%' % (v * 0.5),
    'STARTUP_POWER': lambda v: '%d%%' % v,
    'PWM_FREQUENCY': lambda v: '%d kHz' % v,
    'ADVANCE_LEVEL': lambda v: '%.1f deg' % ((v - 10) * 0.9375)
                    if 10 <= v <= 42 else '%.1f deg (old fmt)' % (v * 7.5),
    'ABSOLUTE_VOLTAGE_CUTOFF': lambda v: '%.1f V' % (v * 0.5),
    'LOW_CELL_VOLTAGE': lambda v: '%.2f V/cell' % ((v + 250) / 100.0),
    'LOW_VOLTAGE_CUTOFF': lambda v: ('off', 'per cell', 'absolute')[v]
                    if v < 3 else '?',
    'CURRENT_LIMIT': lambda v: '%d A' % (v * 2) if v <= 100 else 'off',
    'TEMPERATURE_LIMIT': lambda v: '%d C' % v if 70 <= v <= 140 else 'off',
    'TELEM_RATE': lambda v: '%d Hz' % v,
    'DEBUG_RATE': lambda v: '%d Hz' % v,
    'FILTER_HZ': lambda v: '%d Hz' % v,
    'BEEP_VOLUME': lambda v: 'off' if v == 0 else 'level %d' % v,
    'INPUT_SIGNAL_TYPE': lambda v: {0: 'auto', 1: 'dshot', 2: 'servo',
                                    5: 'dronecan'}.get(v, '%d' % v),
    'CAN_NODE': lambda v: 'dynamic' if v == 0 else 'node %d' % v,
    'DIR_REVERSED': _onoff, 'BI_DIRECTIONAL': _onoff, 'USE_SINE_START': _onoff,
    'COMP_PWM': _onoff, 'VARIABLE_PWM': _onoff, 'STUCK_ROTOR_PROTECTION': _onoff,
    'BRAKE_ON_STOP': _onoff, 'STALL_PROTECTION': _onoff, 'AUTO_ADVANCE': _onoff,
    'RC_CAR_REVERSE': _onoff, 'USE_HALL_SENSORS': _onoff,
    'REQUIRE_ARMING': _onoff, 'REQUIRE_ZERO_THROTTLE': _onoff,
    'TERM_ENABLE': _onoff, 'DISABLE_STICK_CALIBRATION': _onoff,
    'TELEMETRY_ON_INTERVAL': _onoff,
}


def calc_value(name, byte):
    '''engineering value string for a stored byte, or "" if none'''
    fn = CALC.get(name)
    if fn is None:
        return ''
    try:
        return fn(int(byte))
    except (IndexError, KeyError, ValueError):
        return '?'


def kv_to_byte(kv):
    '''MOTOR_KV is stored as (kv-20)/40, so only certain Kv values are
    representable; return the byte the firmware would hold'''
    b = int(round((float(kv) - 20.0) / 40.0))
    return max(0, min(255, b))


def byte_to_kv(b):
    return int(b) * 40 + 20


def model_checks(model):
    '''parameters whose value has to agree with the simulated motor,
    as {name: (required_byte, description)}. model is the sim's motor
    dict (kv, poles); the comparison is on the STORED byte so the
    coarse Kv quantisation does not read as a mismatch'''
    out = {}
    if not model:
        return out
    kv = model.get('kv')
    if kv:
        want = kv_to_byte(kv)
        out['MOTOR_KV'] = (
            want,
            'the simulated motor is %.0f Kv (stored as %d = %d Kv).\n'
            'AM32 scales low rpm power protection from MOTOR_KV, so a\n'
            'mismatch clamps duty and the motor sticks at low rpm.'
            % (kv, want, byte_to_kv(want)))
    poles = model.get('poles')
    if poles:
        out['MOTOR_POLES'] = (
            int(poles),
            'the simulated motor has %d poles; a mismatch scales every\n'
            'reported rpm and the commutation timing.' % int(poles))
    return out


def mismatches(image, model):
    '''names of model-critical parameters whose stored value disagrees'''
    bad = []
    for name, (want, _help) in model_checks(model).items():
        off = PARAMS_BY_NAME[name][0]
        if off < len(image) and image[off] != want:
            bad.append(name)
    return bad
