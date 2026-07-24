'''
AM32 eeprom layout and the conversions between stored bytes and the
values the config tools display

Several settings are not stored as the user sees them. A calibration
data set should record what the configurator or the DroneCAN GUI shows,
because that is what a contributor can read back and compare - but the
raw image is the ground truth, so keep both.

Conversions verified against Src/DroneCAN/DroneCAN.c (the param table
and its get/set special cases) and the Offline-Configurator's widget.cpp
display code:

  advance_level  byte 23  degrees = (raw - 10) * 0.9375
                          DroneCAN ADVANCE_LEVEL reports (raw - 10)
  motor_kv       byte 26  rpm/V   = raw * 40 + 20
  current_limit  byte 44  amps    = raw * 2
  cell cutoff    byte 37  volts   = (raw + 250) / 100
  everything else is stored as displayed
'''

# (offset, name, unit, to_display, note)
FIELDS = [
    (1,  'eeprom_version',          '',      None),
    (2,  'bootloader_version',      '',      None),
    (3,  'firmware_major',          '',      None),
    (4,  'firmware_minor',          '',      None),
    (5,  'max_ramp',                '',      None),
    (6,  'minimum_duty_cycle',      '',      None),
    (7,  'disable_stick_calibration', '',    None),
    (8,  'absolute_voltage_cutoff', '',      None),
    (9,  'current_P',               '',      None),
    (10, 'current_I',               '',      None),
    (11, 'current_D',               '',      None),
    (12, 'active_brake_power',      '',      None),
    (17, 'dir_reversed',            '',      None),
    (18, 'bi_direction',            '',      None),
    (19, 'use_sine_start',          '',      None),
    (20, 'comp_pwm',                '',      None),
    (21, 'variable_pwm',            '',      None),
    (22, 'stuck_rotor_protection',  '',      None),
    (23, 'timing_advance',          'deg',   lambda v: round((v - 10) * 0.9375, 4)),
    (23, 'advance_level',           '',      lambda v: v - 10),
    (24, 'pwm_frequency',           'kHz',   None),
    (25, 'startup_power',           '',      None),
    (26, 'motor_kv',                'rpm/V', lambda v: v * 40 + 20),
    (27, 'motor_poles',             '',      None),
    (28, 'brake_on_stop',           '',      None),
    (29, 'stall_protection',        '',      None),
    (30, 'beep_volume',             '',      None),
    (31, 'telemetry_on_interval',   '',      None),
    (36, 'low_voltage_cut_off',     '',      None),
    (37, 'cell_voltage_threshold',  'V',     lambda v: round((v + 250) / 100.0, 2)),
    (38, 'rc_car_reverse',          '',      None),
    (39, 'use_hall_sensors',        '',      None),
    (40, 'sine_mode_changeover',    '',      None),
    (41, 'drag_brake_strength',     '',      None),
    (42, 'driving_brake_strength',  '',      None),
    (43, 'temperature_limit',       'C',     None),
    (44, 'current_limit',           'A',     lambda v: v * 2),
    (45, 'sine_mode_power',         '',      None),
    (46, 'input_type',              '',      None),
    (47, 'auto_advance',            '',      None),
    (176, 'can_node',               '',      None),
    (177, 'can_esc_index',          '',      None),
    (178, 'can_require_arming',     '',      None),
    (179, 'can_telem_rate',         'Hz',    None),
    (180, 'can_require_zero_throttle', '',   None),
    (181, 'can_filter_hz',          'Hz',    None),
    (182, 'can_debug_rate',         'Hz',    None),
    (183, 'can_term_enable',        '',      None),
]

EEPROM_SIZE = 192


def decode(eeprom):
    '''raw eeprom bytes -> {name: {'raw':int, 'value':num, 'unit':str}}'''
    out = {}
    for off, name, unit, conv in FIELDS:
        if off >= len(eeprom):
            continue
        raw = eeprom[off]
        out[name] = {'offset': off, 'raw': raw,
                     'value': conv(raw) if conv else raw, 'unit': unit}
    return out


def as_record(eeprom):
    '''a record suitable for storing with a calibration data set: the
    raw image plus every value as the config tools display it'''
    vals = decode(eeprom)
    return {
        'eeprom_raw_hex': bytes(eeprom[:EEPROM_SIZE]).hex(),
        'settings': {k: v['value'] for k, v in vals.items()},
        'units': {k: v['unit'] for k, v in vals.items() if v['unit']},
        'raw_bytes': {k: v['raw'] for k, v in vals.items()},
    }


def format_text(eeprom):
    lines = []
    vals = decode(eeprom)
    for off, name, unit, conv in FIELDS:
        if name not in vals:
            continue
        v = vals[name]
        shown = '%g' % v['value'] if isinstance(v['value'], float) else v['value']
        suffix = (' %s' % unit) if unit else ''
        raw_note = '' if conv is None else '   (raw %u)' % v['raw']
        lines.append('  %-26s %8s%-6s%s' % (name, shown, suffix, raw_note))
    return '\n'.join(lines)
