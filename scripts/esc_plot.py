#!/usr/bin/env python3
'''
time-series viewer for esc_measure/esc_chirp/esc_square JSONL logs

throttle demand on a top graph, rpm on a bottom graph, with a shared
(linked) time axis so zooming either keeps them in sync for judging
phase lag. Give a second log to overlay it for comparison, aligned at
each log's first throttle command.

  esc_plot.py square1.jsonl
  esc_plot.py square1.jsonl sitl_square.jsonl
  esc_plot.py run.jsonl --save out.png
  esc_plot.py square1.jsonl sitl_square.jsonl --html cmp.html

--html writes a single self-contained interactive page (plotly.js
embedded, no install needed to view) suitable for putting on a web
server.
'''

import argparse
import json
import os


def label(path):
    return os.path.splitext(os.path.basename(path))[0]


def load(path, align):
    rows = [json.loads(l) for l in open(path)]
    cmds = [(r['t'], r['throttle']) for r in rows if r['type'] == 'cmd']
    status = [(r['t'], r['rpm']) for r in rows if r['type'] == 'status']
    curr = [(r['t'], r['curr']) for r in rows
            if r['type'] == 'status' and r['curr'] == r['curr']]
    # align at the first occurrence of the peak throttle command:
    # robust when two logs ran slightly different staircases (extra
    # levels, reordered preconditioning) where first-command
    # alignment shears them apart cumulatively
    t0 = 0.0
    if align and cmds:
        vmax = max(v for _, v in cmds)
        t0 = min(t for t, v in cmds if v == vmax)
    cmds = [(t - t0, v) for t, v in cmds]
    status = [(t - t0, v) for t, v in status]
    curr = [(t - t0, v) for t, v in curr]
    if cmds:
        # extend the last command to the end of the data
        tend = max(status[-1][0] if status else 0, cmds[-1][0])
        cmds = cmds + [(tend, cmds[-1][1])]
    return cmds, status, curr


def build_figure(logs, align=True):
    '''linked throttle/rpm plotly figure for one or two logs; shared by
    the --html mode and esc_report.py'''
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    fig = make_subplots(rows=3, cols=1, shared_xaxes=True,
                        vertical_spacing=0.045,
                        row_heights=[0.22, 0.5, 0.28],
                        subplot_titles=('throttle demand', 'rpm',
                                        'bus current (A)'))
    colors = [('#1f77b4', '#17becf'), ('#d62728', '#ff7f0e')]
    for path, (c_rpm, c_thr) in zip(logs, colors):
        cmds, status, curr = load(path, align)
        fig.add_trace(go.Scatter(
            x=[t for t, _ in cmds], y=[v for _, v in cmds],
            line=dict(shape='hv', width=1.2, color=c_thr),
            name=label(path), legendgroup=path, showlegend=False), row=1, col=1)
        fig.add_trace(go.Scatter(
            x=[t for t, _ in status], y=[v for _, v in status],
            line=dict(width=1, color=c_rpm), mode='lines',
            name=label(path), legendgroup=path), row=2, col=1)
        fig.add_trace(go.Scatter(
            x=[t for t, _ in curr], y=[v for _, v in curr],
            line=dict(width=1, color=c_rpm), mode='lines',
            name=label(path), legendgroup=path, showlegend=False), row=3, col=1)
    fig.update_xaxes(title_text='time (s)', row=3, col=1)
    fig.update_yaxes(rangemode='tozero')
    fig.update_traces(hoverlabel=dict(namelength=-1))
    fig.update_layout(template='plotly_white', height=900,
                      legend=dict(orientation='h', y=1.07),
                      margin=dict(l=60, r=20, t=60, b=50))
    return fig


def step_segments(path):
    '''group throttle transitions by (delta, direction); returns
    {(delta, 'up'|'down'): [(t_edge, lo, hi), ...]}'''
    rows = [json.loads(l) for l in open(path)]
    cmds = [(r['t'], r['throttle']) for r in rows if r['type'] == 'cmd']
    status = [(r['t'], r['rpm']) for r in rows if r['type'] == 'status']
    segs = {}
    for i in range(1, len(cmds)):
        t, thr = cmds[i]
        prev = cmds[i - 1][1]
        if thr == prev or thr == 0 or prev == 0:
            continue
        delta = round(abs(thr - prev), 3)
        key = (delta, 'up' if thr > prev else 'down')
        t_end = cmds[i + 1][0] if i + 1 < len(cmds) else t + 1.0
        segs.setdefault(key, []).append((t, t_end))
    return segs, status


def build_steps_figure(logs, pre=0.05, window=0.6):
    '''per-step overlay grid: one panel per (delta, direction), each
    step response aligned at its command edge, real and SITL logs
    overlaid. This sidesteps the timeline divergence caused by the
    tools' settle-time detection running at different rates'''
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    data = [step_segments(path) for path in logs]
    deltas = sorted({d for segs, _ in data for (d, _) in segs})
    if not deltas:
        raise SystemExit('no throttle steps found')
    rows = len(deltas)
    titles = []
    for d in deltas:
        titles += ['delta %.2f up' % d, 'delta %.2f down' % d]
    fig = make_subplots(rows=rows, cols=2, subplot_titles=titles,
                        vertical_spacing=min(0.025, 0.5 / rows),
                        horizontal_spacing=0.07)
    colors = ['#1f77b4', '#d62728']
    for li, ((segs, status), path) in enumerate(zip(data, logs)):
        color = colors[li % len(colors)]
        shown = False
        for r, d in enumerate(deltas):
            for c, dirn in ((1, 'up'), (2, 'down')):
                for t0, t_end in segs.get((d, dirn), []):
                    pts = [(t - t0, rpm) for t, rpm in status
                           if t0 - pre <= t <= min(t0 + window, t_end)]
                    if len(pts) < 4:
                        continue
                    fig.add_trace(go.Scatter(
                        x=[1000.0 * t for t, _ in pts],
                        y=[v for _, v in pts],
                        mode='lines', line=dict(width=1, color=color),
                        opacity=0.75, name=label(path),
                        legendgroup=path, showlegend=not shown),
                        row=r + 1, col=c)
                    shown = True
    # every panel gets its own axis label: with a tall grid the
    # bottom-row-only convention scrolls out of view
    fig.update_xaxes(title_text='ms since step', title_standoff=2,
                     title_font_size=10)
    fig.update_yaxes(title_text='rpm', title_standoff=2, title_font_size=10,
                     col=1)
    fig.update_traces(hoverlabel=dict(namelength=-1))
    fig.update_layout(template='plotly_white', height=max(900, 330 * rows),
                      legend=dict(orientation='h', y=1.0 + 0.18 / rows),
                      margin=dict(l=60, r=20, t=60, b=50))
    fig.update_annotations(font_size=12)
    return fig


def plot_html(args):
    if args.steps:
        fig = build_steps_figure(args.logs)
    else:
        fig = build_figure(args.logs, args.align)
    fig.write_html(args.html, include_plotlyjs=True)
    print('wrote %s' % args.html)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', nargs='+', help='one or two JSONL logs')
    parser.add_argument('--save', default=None, help='write a png instead of a window')
    parser.add_argument('--html', default=None,
                        help='write a self-contained interactive html page')
    parser.add_argument('--no-align', dest='align', action='store_false',
                        help='do not shift each log to its first throttle command')
    parser.add_argument('--steps', action='store_true',
                        help='per-step overlay grid (html only): each throttle '
                             'step aligned at its command edge, grouped by '
                             'amplitude and direction')
    args = parser.parse_args()
    if len(args.logs) > 2:
        raise SystemExit('at most two logs')

    if args.html:
        plot_html(args)
        return
    if args.steps:
        raise SystemExit('--steps requires --html')

    import matplotlib
    if args.save:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, (ax_thr, ax_rpm, ax_cur) = plt.subplots(
        3, 1, figsize=(13, 9), sharex=True, height_ratios=[1, 2.5, 1.2])
    palettes = [('tab:blue', 'tab:cyan'), ('tab:red', 'tab:orange')]
    for path, (c_rpm, c_thr) in zip(args.logs, palettes):
        cmds, status, curr = load(path, args.align)
        if cmds:
            ax_thr.plot([t for t, _ in cmds], [v for _, v in cmds],
                        drawstyle='steps-post', lw=1.0, color=c_thr, alpha=0.9,
                        label='%s throttle' % label(path))
        ax_rpm.plot([t for t, _ in status], [v for _, v in status],
                    lw=0.7, color=c_rpm, label='%s rpm' % label(path))
        ax_cur.plot([t for t, _ in curr], [v for _, v in curr],
                    lw=0.7, color=c_rpm, label='%s current' % label(path))
    ax_cur.set_xlabel('time (s)')
    ax_thr.set_ylabel('throttle demand')
    ax_rpm.set_ylabel('rpm')
    ax_cur.set_ylabel('bus current (A)')
    for ax in (ax_thr, ax_rpm, ax_cur):
        ax.set_ylim(bottom=0)
        ax.legend(fontsize=8, loc='lower right')
        ax.grid(alpha=0.3)
    fig.tight_layout()
    if args.save:
        fig.savefig(args.save, dpi=120)
        print('saved %s' % args.save)
    else:
        plt.show()


if __name__ == '__main__':
    main()
