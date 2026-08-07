#!/usr/bin/env python3
'''
build a SITL motor model from the numbers on a motor's spec sheet

The calibration flow fits a model from a bench capture, but a user who
just wants to try SITL, or wants a sensible starting point before
capturing, should not have to know inductances and torque constants.
This turns the things a user can read off a motor, an ESC, a battery
and a propeller into the model.json the simulator consumes.

Every value is either taken directly (kv, poles, pack voltage) or
estimated from a documented physical relation (below); a later bench
capture and fit refines them. The estimates were checked against the
committed data sets and land within the range a fit then moves through.

  model_builder.py --size 2216 --poles 14 --kv 920 \\
      --resistance-mohm 115 --idle 0.8@10 --weight 64 \\
      --prop 9x4.5 --source battery --cells 4 -o model.json

Run with -v to print how each number was arrived at.
'''

import argparse
import json
import math
import re
import sys

RHO_AIR = 1.225          # kg/m^3, sea level
TWO_PI = 2.0 * math.pi


def parse_size_code(code):
    '''"2216" -> (stator diameter 22mm, stator height 16mm).

    The convention is two digits of diameter then two of height, both
    in mm; larger motors use a 4+2 split ("2814"), which the same
    "first half is diameter" rule handles. A code already given as
    "22x16" or "22,16" is accepted too.'''
    s = str(code).strip().lower().replace('mm', '')
    m = re.match(r'^(\d+)\s*[x,]\s*(\d+)$', s)
    if m:
        return float(m.group(1)), float(m.group(2))
    if not s.isdigit() or len(s) < 4:
        raise ValueError('motor size %r is not a 4-digit code like 2216 '
                         'or a DxH like 22x16' % code)
    half = len(s) // 2
    return float(s[:half]), float(s[half:])


def prop_dims_m(diameter, pitch, units='in'):
    '''propeller diameter and pitch in metres. Props are quoted in
    inches by convention (a "9x4.5"), but mm is accepted.'''
    scale = 0.0254 if units == 'in' else 0.001
    return diameter * scale, pitch * scale


def parse_prop(text):
    '''"9x4.5" or "9x4.5in" or "240x120mm" -> (diameter, pitch, units)'''
    s = str(text).strip().lower()
    units = 'in'
    if s.endswith('mm'):
        units, s = 'mm', s[:-2]
    elif s.endswith('in') or s.endswith('"'):
        s = s.rstrip('in"')
    m = re.match(r'^([\d.]+)\s*x\s*([\d.]+)$', s)
    if not m:
        raise ValueError('propeller %r is not a DxP like 9x4.5 or '
                         '240x120mm' % text)
    return float(m.group(1)), float(m.group(2)), units


def parse_idle(text):
    '''"0.8@10" -> (0.8 A at 10 V). A bare "0.8" means at the pack
    voltage, decided by the caller.'''
    s = str(text).strip()
    if '@' in s:
        cur, volt = s.split('@', 1)
        return float(cur), float(volt)
    return float(s), None


def prop_torque_coeff(pitch_over_diameter):
    '''propeller torque coefficient C_Q for Q = C_Q * rho * n^2 * D^5,
    n in rev/s. There is no substitute for measured prop data, but C_Q
    rises with pitch, and this linear fit in pitch/diameter reproduces
    the load coefficient of the committed prop-on data set.'''
    return 0.005 + 0.012 * pitch_over_diameter


def build_model(spec, notes=None):
    '''spec is a dict of the user-knowable numbers (see main()); returns
    the model dict. Pass a list as `notes` to collect the derivations.'''
    def note(msg):
        if notes is not None:
            notes.append(msg)

    diam_mm, height_mm = parse_size_code(spec['size'])
    poles = int(spec['poles'])
    kv = float(spec['kv'])
    r_stator = diam_mm * 0.5e-3
    stator_vol = math.pi * r_stator * r_stator * height_mm * 1e-3

    # --- motor electrical ---
    # spec sheets quote line-to-line resistance; two phases conduct in
    # series so the per-phase value the sim wants is half
    r_ll = spec.get('resistance_mohm')
    if r_ll:
        resistance = r_ll * 1e-3 * 0.5
        note('resistance %.4f ohm/phase (%.0f mohm line-to-line / 2)'
             % (resistance, r_ll))
    else:
        resistance = 0.06
        note('resistance %.4f ohm/phase (default, no spec value)'
             % resistance)

    # inductance scales as turns^2 (turns ~ 1/kv) times stator volume;
    # the constant is set from the committed motors
    inductance = 2.2e6 * stator_vol / (kv * kv)
    note('inductance %.2e H (2.2e6 * stator_vol %.2e / kv^2)'
         % (inductance, stator_vol))

    # --- mechanical: inertia ---
    weight_g = spec.get('weight_g')
    if weight_g:
        rotor_mass = 0.55 * weight_g * 1e-3       # bell + magnets
        r_eff = r_stator * 1.15                    # rim just outside stator
        inertia_motor = rotor_mass * r_eff * r_eff
        note('rotor inertia %.2e kg m^2 (0.55 * %.0fg at r %.1fmm)'
             % (inertia_motor, weight_g, r_eff * 1e3))
    else:
        inertia_motor = 0.3 * stator_vol * 1e3 * (r_stator ** 2) * 20
        note('rotor inertia %.2e kg m^2 (size estimate, no weight)'
             % inertia_motor)

    prop = spec.get('prop')
    inertia = inertia_motor
    load_k = 0.0
    if prop:
        d_m, p_m = prop_dims_m(*prop)
        # prop mass estimate ~0.9g per inch of diameter (nylon/GF);
        # a thin rod about its centre, tapered blades ~0.9 of full span
        prop_mass = spec.get('prop_mass_g')
        if prop_mass is None:
            prop_mass = 0.9 * (d_m / 0.0254)
            note('prop mass %.1fg (estimate, 0.9g per inch)' % prop_mass)
        inertia_prop = (1.0 / 12.0) * (prop_mass * 1e-3) * (0.9 * d_m) ** 2
        inertia += inertia_prop
        note('prop inertia %.2e kg m^2 (rod, %.1fg span %.0fmm)'
             % (inertia_prop, prop_mass, d_m * 1e3))
        cq = prop_torque_coeff(p_m / d_m)
        load_k = cq * RHO_AIR * (d_m ** 5) / (4.0 * math.pi ** 2)
        note('load_k_omega2 %.2e (C_Q %.4f, D %.0fmm, pitch %.0fmm)'
             % (load_k, cq, d_m * 1e3, p_m * 1e3))
    else:
        note('no propeller: inertia is the rotor only, no aero load')

    # --- mechanical: drag from the no-load idle current ---
    idle = spec.get('idle')
    kt = 60.0 / (TWO_PI * kv)                       # N m / A, line-to-line
    if idle:
        i_idle, v_idle = idle
        if v_idle is None:
            v_idle = spec.get('voltage') or 10.0
        omega_nl = kv * v_idle * TWO_PI / 60.0 * 0.92
        q_drag = kt * i_idle
        static_friction = 0.15 * q_drag
        damping = 0.85 * q_drag / omega_nl
        note('damping %.2e N m s/rad, static_friction %.2e N m '
             '(idle %.2fA at %.0fV: drag %.2e N m at %.0f rad/s)'
             % (damping, static_friction, i_idle, v_idle, q_drag, omega_nl))
    else:
        damping = 1e-5
        static_friction = 0.002
        note('damping/static_friction defaults (no idle current given)')

    # --- battery / supply ---
    voltage = spec.get('voltage')
    cells = spec.get('cells')
    if not voltage and cells:
        voltage = 3.8 * cells
        note('pack voltage %.1fV (%uS at 3.8V, override with --voltage)'
             % (voltage, cells))
    elif not voltage:
        voltage = 12.0
        note('pack voltage 12.0V (default)')

    source = spec.get('source', 'battery')
    bat_r = spec.get('battery_resistance')
    if bat_r is None:
        if source == 'battery' and cells:
            bat_r = cells * 0.012 + 0.02           # cells + wiring/connectors
        elif source == 'battery':
            bat_r = 0.08
        else:
            bat_r = 0.05                            # bench supply
        note('source resistance %.4f ohm (%s estimate)' % (bat_r, source))

    # a battery sinks regenerated braking energy readily; a bench supply
    # with an output diode barely sinks at all, so the bus pumps up
    if source == 'battery':
        sink_resistance, sink_current_max = 0.35, 0.9
    else:
        sink_resistance, sink_current_max = 10.0, 0.05
    note('sink %s: resistance %.2f ohm, current_max %.2f A'
         % (source, sink_resistance, sink_current_max))

    # --- esc ---
    esc_current = spec.get('esc_current_a')
    if esc_current:
        rds_on = max(0.002, 0.12 / esc_current)
        note('rds_on %.4f ohm (%.0fA ESC)' % (rds_on, esc_current))
    else:
        rds_on = 0.005
        note('rds_on %.4f ohm (default)' % rds_on)

    temp_c = spec.get('temperature_c', 25.0)

    return {
        'motor': {
            'kv': round(kv, 1),
            'poles': poles,
            'resistance': _sig(resistance),
            'inductance': _sig(inductance),
            'inertia': _sig(inertia),
            'damping': _sig(damping),
            'static_friction': _sig(static_friction),
            'load_k_omega2': _sig(load_k),
        },
        'battery': {
            'voltage': round(voltage, 2),
            'resistance': _sig(bat_r),
            'capacitance': 0.002,
            'sink_resistance': sink_resistance,
            'sink_current_max': sink_current_max,
        },
        'esc': {
            'rds_on': _sig(rds_on),
            'diode_vf': 0.7,
            'temperature_c': round(float(temp_c), 1),
            'commutation_transfer': 1.0,
        },
        'sim': {
            'comparator_hysteresis_mv': 0.0,
            'comparator_noise_mv': 3.0,
            'comparator_phase_rc_ns': 800,
            'comparator_neutral_rc_ns': 800,
        },
    }


def _sig(x, digits=3):
    '''round to a few significant figures, keeping the JSON readable'''
    if x == 0:
        return 0.0
    from math import floor, log10
    p = digits - 1 - int(floor(log10(abs(x))))
    return round(x, p)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--size', required=True,
                    help='motor size code, e.g. 2216 (22mm dia, 16mm high)')
    ap.add_argument('--poles', type=int, required=True,
                    help='magnet pole count (e.g. 14)')
    ap.add_argument('--kv', type=float, required=True, help='rpm per volt')
    ap.add_argument('--resistance-mohm', type=float,
                    help='winding resistance line-to-line, milliohms')
    ap.add_argument('--idle', help='no-load current, "0.8@10" = 0.8A at 10V')
    ap.add_argument('--weight', type=float, help='motor weight, grams')
    ap.add_argument('--prop', help='propeller, "9x4.5" (inches) or "240x120mm"')
    ap.add_argument('--prop-mass', type=float, help='propeller mass, grams')
    ap.add_argument('--source', choices=['battery', 'supply'],
                    default='battery', help='power source type')
    ap.add_argument('--cells', type=int, help='battery cell count (LiPo)')
    ap.add_argument('--voltage', type=float, help='pack/supply voltage')
    ap.add_argument('--source-resistance', type=float,
                    help='source internal resistance, ohms')
    ap.add_argument('--esc-current', type=float, help='ESC current rating, A')
    ap.add_argument('--temperature', type=float, default=25.0,
                    help='operating temperature, C')
    ap.add_argument('-o', '--out', help='write the model here (else stdout)')
    ap.add_argument('-v', '--verbose', action='store_true',
                    help='print how each value was derived')
    args = ap.parse_args()

    spec = {
        'size': args.size, 'poles': args.poles, 'kv': args.kv,
        'resistance_mohm': args.resistance_mohm,
        'idle': parse_idle(args.idle) if args.idle else None,
        'weight_g': args.weight,
        'prop': parse_prop(args.prop) if args.prop else None,
        'prop_mass_g': args.prop_mass,
        'source': args.source, 'cells': args.cells, 'voltage': args.voltage,
        'battery_resistance': args.source_resistance,
        'esc_current_a': args.esc_current, 'temperature_c': args.temperature,
    }
    notes = []
    model = build_model(spec, notes)
    if args.verbose:
        for n in notes:
            sys.stderr.write('  %s\n' % n)
    text = json.dumps(model, indent=4)
    if args.out:
        with open(args.out, 'w') as f:
            f.write(text + '\n')
        sys.stderr.write('wrote %s\n' % args.out)
    else:
        print(text)


if __name__ == '__main__':
    main()
