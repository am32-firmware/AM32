#!/usr/bin/env python3
'''
generate a static html report of real-vs-SITL calibration results,
ready to rsync to a web server.

Produces an index page describing each data set with a link to a
per-dataset page carrying all the interactive comparison plots
(chirp, square, steady sweep). plotly.js is written once into the
output directory and shared by all pages, so each page stays small
and everything works offline with no tooling installed.

  esc_report.py --out report
  rsync -av report/ myserver:public_html/am32-sitl/

Edit DATASETS below to add a data set or reword a description.
'''

import argparse
import html
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import esc_plot

DATA = 'Mcu/SITL/data'

DATASETS = [
    {
        'name': 'gt2215',
        'title': 'SEQURE G431 CAN ESC + EMAX GT2215/10 1100KV',
        'description': '''
First calibration target: SEQURE G431 DroneCAN ESC driving an EMAX
GT2215/10 1100KV (14 pole) small quad motor, no prop, 16V bench
supply.
SITL model <code>sequre_gt2215.json</code> v9. Steady state matches
within about 1.5% (top end exact), the chirp accelerating 3dB
bandwidth matches (11.4-12.8Hz across runs vs 11.6 real), and the
square-wave braking curve is monotonic with the largest amplitude
matching. Known residuals: small-signal braking ~15% fast and the
braking bus overshoot exceeds the real supply's 24.2V - both point
at the ideal trapezoid BEMF waveform used by the model.''',
        'plots': [
            ('120s frequency chirp (0.35 throttle, +/-0.15)',
             f'{DATA}/SEQURE_G431/sequre_chirp_120s.jsonl',
             f'{DATA}/SEQURE_G431/sitl_chirp_120s_v9.jsonl'),
            ('square-wave amplitude sweep (full timeline; the two runs '
             'settle at different rates so the time axes diverge)',
             f'{DATA}/SEQURE_G431/square1.jsonl',
             f'{DATA}/SEQURE_G431/sitl_square_v9.jsonl'),
            ('square-wave step responses, aligned at each command edge',
             f'{DATA}/SEQURE_G431/square1.jsonl',
             f'{DATA}/SEQURE_G431/sitl_square_v9.jsonl', 'steps'),
            ('steady-state sweep',
             f'{DATA}/SEQURE_G431/sweep1.jsonl',
             f'{DATA}/SEQURE_G431/sitl_sweep_v9.jsonl'),
        ],
    },
    {
        'name': 'vim1404',
        'title': 'VimDrones L431 dev board ESC + 1404 4300KV',
        'description': '''
Second calibration target: VimDrones ESC development board
(STM32L431, DroneCAN) driving a tiny 1404 4300KV (14 pole) motor,
no prop, 12V bench supply. SITL model <code>vimdrones_1404.json</code>
v2. Steady state matches within 4% (top half ~2%), chirp
accelerating 3dB 9.2Hz vs 8.0-8.8 real, and the square down-curve
reproduces the real shape including the last-point drop. This
combination genuinely desyncs on the bench (chirp troughs); the
model reproduces that marginality at the same operating points.
Known residuals: small-signal braking ~25% fast and large-amplitude
transients run fast at the fitted inertia - the same residual family
as the GT2215, pointing at the shared trapezoid BEMF model.''',
        'plots': [
            ('120s frequency chirp (0.35 throttle, +/-0.15)',
             f'{DATA}/VIMDRONES_L431/vim_chirp_120s_v2.jsonl',
             f'{DATA}/VIMDRONES_L431/sitl_chirp_120s_v2.jsonl'),
            ('square-wave amplitude sweep (full timeline; the two runs '
             'settle at different rates so the time axes diverge)',
             f'{DATA}/VIMDRONES_L431/square2.jsonl',
             f'{DATA}/VIMDRONES_L431/sitl_square_v2.jsonl'),
            ('square-wave step responses, aligned at each command edge',
             f'{DATA}/VIMDRONES_L431/square2.jsonl',
             f'{DATA}/VIMDRONES_L431/sitl_square_v2.jsonl', 'steps'),
            ('steady-state sweep',
             f'{DATA}/VIMDRONES_L431/sweep1.jsonl',
             f'{DATA}/VIMDRONES_L431/sitl_sweep_v2.jsonl'),
        ],
    },
    {
        'name': 'nano2216',
        'title': 'VimDrones Nano ESC + T-Motor AIR 2216 II 920KV, 10x4.5 prop',
        'description': '''
Third calibration target and the first with a propeller: the Edu450
frame drivetrain on 3S (12.6V bench supply, firmware CURRENT_LIMIT
8A protecting the bench supply - mirrored in the SITL eeprom). SITL
model <code>vimdrones_nano_2216.json</code> v1. Steady state within
2% over most of the band including the 8A current-limiter plateau
above 0.85 throttle; chirp accel 3dB 3.31 vs 3.41Hz real. The prop
makes the real dynamics nearly symmetric (aerodynamic drag does the
braking) and completely desync-free, both reproduced. Known
residuals: braking ~15-30% slow (the braking residual inverts sign
with a prop vs the unloaded combos) and the sim misses the
current-limited up-transient stretch at large square amplitudes.''',
        'plots': [
            ('120s frequency chirp (0.35 throttle, +/-0.15)',
             f'{DATA}/VIMDRONES_NANO_2216/nano_chirp_120s.jsonl',
             f'{DATA}/VIMDRONES_NANO_2216/sitl_chirp_120s_v1.jsonl'),
            ('square-wave amplitude sweep (full timeline; the two runs '
             'settle at different rates so the time axes diverge)',
             f'{DATA}/VIMDRONES_NANO_2216/square1.jsonl',
             f'{DATA}/VIMDRONES_NANO_2216/sitl_square_v1.jsonl'),
            ('square-wave step responses, aligned at each command edge',
             f'{DATA}/VIMDRONES_NANO_2216/square1.jsonl',
             f'{DATA}/VIMDRONES_NANO_2216/sitl_square_v1.jsonl', 'steps'),
            ('steady-state sweep (real includes the abort-terminated 1.0 '
             'step attempt; both show the 8A limiter plateau above 0.85)',
             f'{DATA}/VIMDRONES_NANO_2216/sweep2.jsonl',
             f'{DATA}/VIMDRONES_NANO_2216/sitl_sweep_v1.jsonl'),
        ],
    },
]

STYLE = '''
body { font-family: sans-serif; max-width: 1100px; margin: 1em auto;
       padding: 0 1em; color: #222; }
h1 { font-size: 1.5em; } h2 { font-size: 1.2em; margin-top: 1.5em; }
a { color: #1f77b4; }
.desc { max-width: 55em; line-height: 1.45; }
.plotbox { border: 1px solid #ddd; margin: 0.5em 0 2em 0; }
.crumb { font-size: 0.9em; margin-bottom: 1em; }
footer { margin: 2em 0 1em 0; font-size: 0.8em; color: #888; }
'''

FOOT = ('<footer>generated by scripts/esc_report.py (am32-firmware); '
        'real hardware captures and SITL runs from Mcu/SITL/data/'
        '</footer>')


def page(title, body):
    return ('<!DOCTYPE html><html><head><meta charset="utf-8">'
            '<meta name="viewport" content="width=device-width, initial-scale=1">'
            f'<title>{html.escape(title)}</title>'
            f'<style>{STYLE}</style></head><body>{body}{FOOT}</body></html>')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', default='report', help='output directory')
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)

    wrote_js = False
    items = []
    for ds in DATASETS:
        secs = [f'<div class="crumb"><a href="index.html">&larr; all data sets</a></div>'
                f'<h1>{html.escape(ds["title"])}</h1>'
                f'<div class="desc">{ds["description"]}</div>']
        for entry in ds['plots']:
            ptitle, real, sitl = entry[:3]
            kind = entry[3] if len(entry) > 3 else 'timeline'
            missing = [p for p in (real, sitl) if not os.path.exists(p)]
            if missing:
                print('skipping "%s" (%s missing)' % (ptitle, ', '.join(missing)))
                continue
            if kind == 'steps':
                fig = esc_plot.build_steps_figure([real, sitl])
            else:
                fig = esc_plot.build_figure([real, sitl])
            frag = fig.to_html(full_html=False, include_plotlyjs=False,
                               default_height=750)
            secs.append(f'<h2>{html.escape(ptitle)}</h2>'
                        f'<div class="plotbox">{frag}</div>')
        body = '<script src="plotly.min.js"></script>' + ''.join(secs)
        fname = f'{ds["name"]}.html'
        with open(os.path.join(args.out, fname), 'w') as f:
            f.write(page(ds['title'], body))
        print('wrote %s/%s' % (args.out, fname))
        if not wrote_js:
            # let plotly drop its bundled js into the output directory
            fig.write_html(os.path.join(args.out, '_tmp.html'),
                           include_plotlyjs='directory')
            os.remove(os.path.join(args.out, '_tmp.html'))
            wrote_js = True
        items.append(
            f'<h2><a href="{fname}">{html.escape(ds["title"])}</a></h2>'
            f'<div class="desc">{ds["description"]}</div>')

    body = ('<h1>AM32 SITL calibration: real hardware vs simulation</h1>'
            '<div class="desc">Each data set below compares bench captures '
            'of a real AM32 ESC/motor combination against the AM32 SITL '
            'physics model fitted to it. All plots are interactive: drag '
            'to zoom (both graphs stay linked in time), double-click to '
            'reset, click legend entries to toggle traces.</div>'
            + ''.join(items))
    with open(os.path.join(args.out, 'index.html'), 'w') as f:
        f.write(page('AM32 SITL calibration results', body))
    print('wrote %s/index.html' % args.out)


if __name__ == '__main__':
    main()
