#!/usr/bin/env python3
'''
Tone client for the AM32 SITL: subscribes to the tone event stream on
the state port and prints, plays, records or asserts on the beeps the
firmware plays (startup tune, DShot beacons, BlueJay tunes).

Durations are computed from the simulated timestamps, so they are
correct at any --speedup: pitch is unaffected, wall-clock duration
scales with 1/speedup.

Playback needs PySide6 (QtMultimedia) from the GUI venv; the other
modes are stdlib only. On Windows use native Windows Python for
playback, not Cygwin Python (no PySide6 wheels there).

usage:
  sitl_tones.py [--port N] print
  sitl_tones.py [--port N] play [--volume V] [--physics]
  sitl_tones.py [--port N] wav OUT.wav --seconds N [--volume V] [--physics]
  sitl_tones.py [--port N] assert --notes 428.6:0.2,585.4:0.2,923.2:0.2

--physics plays the physics audio stream instead of synthesized
tones: torque ripple and phase current magnitude from the motor
model, so commutation, PWM noise and beeps sound as the motor would.
Pitch scales with --speedup (slow motion sounds lower).
'''

import argparse
import math
import os
import struct
import sys
import time
import wave

try:
    import numpy as np   # optional: playback synthesis runs vectorized
except ImportError:
    np = None            # pure python sample loops still work

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sitl_gui_backend import ToneStream, AudioStream

SAMPLE_RATE = 48000
# perceptual reference: the default beep_volume gives a duty fraction
# of ~0.002, which should be clearly audible
FULL_SCALE_DUTY = 0.005
RAMP_S = 0.003  # attack/release to avoid clicks


def tone_gain(amp, volume):
    '''map a raw PWM duty fraction to a playback gain'''
    if amp <= 0:
        return 0.0
    return volume * min(1.0, math.sqrt(amp / FULL_SCALE_DUTY))


def events_to_notes(events, min_amp=1e-5):
    '''collapse a tone event list into [(freq_hz, duration_s, amplitude)].
    Notes end at the next event; a trailing note without a terminating
    event (silence or change) is dropped. Simulated time going
    backwards (an emulated reset) also terminates the note list
    segment.'''
    notes = []
    for i, ev in enumerate(events[:-1]):
        t_ns, freq, amp = ev[1], ev[2], ev[3]
        if amp < min_amp:
            continue
        t2 = events[i + 1][1]
        if t2 <= t_ns:
            continue
        notes.append((freq, (t2 - t_ns) * 1e-9, amp))
    return notes


def render_wav(events, path, volume=0.5):
    '''render a tone event list to a mono 16 bit WAV, no audio device
    needed'''
    frames = bytearray()
    phase = 0.0
    for freq, dur, amp in events_to_notes(events, min_amp=-1.0):
        gain = tone_gain(amp, volume)
        n = int(dur * SAMPLE_RATE)
        ramp = min(int(RAMP_S * SAMPLE_RATE), n // 2)
        for k in range(n):
            g = gain
            if ramp > 0:
                if k < ramp:
                    g *= k / ramp
                elif n - k < ramp:
                    g *= (n - k) / ramp
            if freq > 0:
                phase += 2 * math.pi * freq / SAMPLE_RATE
            frames += struct.pack('<h', int(32767 * g * math.sin(phase)))
        phase %= 2 * math.pi
    with wave.open(path, 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(SAMPLE_RATE)
        w.writeframes(bytes(frames))
    return len(frames) // 2


def _run_sink_on_thread(build):
    '''run a QAudioSink on its own thread with its own Qt event loop.

    QtMultimedia pull mode services readData from the event loop of the
    thread that owns the sink. On the GUI thread a graph repaint blocks
    that loop and the audio underruns (heard as stutter), so the sink is
    built and pumped here instead, fully decoupled from rendering.

    build() runs on the new thread and returns (sink, dev), or None when
    there is no output device, or raises on error. Returns
    (thread, ok, error).'''
    from PySide6.QtCore import QThread
    import threading
    ready = threading.Event()
    res = {'ok': False, 'err': ''}

    class SinkThread(QThread):
        def run(self):
            try:
                built = build()
            except Exception as ex:
                res['err'] = str(ex)
                ready.set()
                return
            if not built:
                ready.set()          # no device; build() set the message
                return
            self._sink, self._dev = built
            res['ok'] = True
            ready.set()
            self.exec()              # services the audio pull on this thread
            self._sink.stop()

    t = SinkThread()
    t.start()
    ready.wait(3.0)
    return t, res['ok'], res['err']


class ToneSynth(object):
    '''continuous sine synth following a ToneStream, via QtMultimedia.
    Created lazily so everything else works without PySide6; silently
    inactive when there is no audio output device (headless CI). The
    sink runs on its own thread so graph repaints cannot stall it.'''

    def __init__(self, stream, volume=0.5):
        self.stream = stream
        self.volume = volume
        self.active = False
        self.error = ''
        try:
            from PySide6.QtCore import QIODevice
            from PySide6.QtMultimedia import (QAudioFormat, QAudioSink,
                                              QMediaDevices)
        except ImportError as ex:
            self.error = 'PySide6 QtMultimedia not available: %s' % ex
            return

        synth = self

        class SineDevice(QIODevice):
            '''pull-mode source: the sink calls readData for each
            period, we synthesize from the current tone state'''
            def __init__(self):
                super().__init__()
                self.phase = 0.0
                self.gain = 0.0  # ramped towards the target
                self.open(QIODevice.OpenModeFlag.ReadOnly)

            def bytesAvailable(self):
                return 65536 + super().bytesAvailable()

            def readData(self, maxlen):
                n = min(maxlen // 2, 4096)
                if n <= 0:
                    return b''
                freq, amp = synth.stream.current()
                target = tone_gain(amp, synth.volume)
                step = 1.0 / (RAMP_S * SAMPLE_RATE)
                if np is not None:
                    # vectorized: the per-sample python loop is a large
                    # GIL load at 48kHz, felt by the DShot sender thread
                    k = np.arange(1, n + 1)
                    if self.gain < target:
                        g = np.minimum(target, self.gain + step * k)
                    elif self.gain > target:
                        g = np.maximum(target, self.gain - step * k)
                    else:
                        g = np.full(n, self.gain)
                    if freq > 0:
                        ph = self.phase + (2 * math.pi * freq / SAMPLE_RATE) * k
                        self.phase = float(ph[-1]) % (2 * math.pi)
                    else:
                        ph = np.full(n, self.phase)
                    self.gain = float(g[-1])
                    return (32767 * g * np.sin(ph)).astype('<i2').tobytes()
                out = bytearray()
                for _ in range(n):
                    if self.gain < target:
                        self.gain = min(target, self.gain + step)
                    elif self.gain > target:
                        self.gain = max(target, self.gain - step)
                    if freq > 0:
                        self.phase += 2 * math.pi * freq / SAMPLE_RATE
                        if self.phase > 2 * math.pi:
                            self.phase -= 2 * math.pi
                    out += struct.pack('<h',
                                       int(32767 * self.gain * math.sin(self.phase)))
                return bytes(out)

            def writeData(self, data):
                return -1

        def build():
            dev = QMediaDevices.defaultAudioOutput()
            if dev.isNull():
                synth.error = 'no audio output device'
                return None
            fmt = QAudioFormat()
            fmt.setSampleRate(SAMPLE_RATE)
            fmt.setChannelCount(1)
            fmt.setSampleFormat(QAudioFormat.SampleFormat.Int16)
            io = SineDevice()
            sink = QAudioSink(dev, fmt)
            sink.setBufferSize(SAMPLE_RATE // 10)
            sink.start(io)
            return sink, io

        self.thread, self.active, err = _run_sink_on_thread(build)
        if err:
            self.error = err

    def set_volume(self, volume):
        self.volume = volume

    def stop(self):
        if self.active:
            self.thread.quit()      # ends the audio thread's event loop
            self.thread.wait(2000)
            self.active = False


def goertzel(samples, freq_hz, rate=SAMPLE_RATE):
    '''single-bin DFT magnitude, stdlib only (used by the CI tests to
    check spectral content of the physics audio stream)'''
    w = 2 * math.pi * freq_hz / rate
    c = 2 * math.cos(w)
    s1 = s2 = 0.0
    for v in samples:
        s0 = v + c * s1 - s2
        s2, s1 = s1, s0
    return math.sqrt(abs(s1 * s1 + s2 * s2 - c * s1 * s2)) / max(1, len(samples))


class PhysicsAudio(object):
    '''QtMultimedia player for the physics audio stream. The sample
    arrival rate varies with --speedup and pacing jitter, so playback
    uses a simple adaptive resampler: the consume ratio tracks the
    buffer fill, which settles at arrival_rate/48kHz (slow motion
    plays lower pitched, as physics should). A slow AGC keeps quiet
    beeps and full throttle motor noise both audible.'''

    def __init__(self, stream, volume=0.5):
        self.stream = stream
        self.volume = volume
        self.active = False
        self.error = ''
        try:
            from PySide6.QtCore import QIODevice
            from PySide6.QtMultimedia import (QAudioFormat, QAudioSink,
                                              QMediaDevices)
        except ImportError as ex:
            self.error = 'PySide6 QtMultimedia not available: %s' % ex
            return

        player = self

        class PhysDevice(QIODevice):
            def __init__(self):
                super().__init__()
                self.buf = []
                self.pos = 0.0  # fractional read position into buf
                self.env = 1e-3  # AGC envelope
                self.fade = 0.0  # ramp on underrun/recovery, no clicks
                self.open(QIODevice.OpenModeFlag.ReadOnly)

            def bytesAvailable(self):
                return 65536 + super().bytesAvailable()

            def readData(self, maxlen):
                n = min(maxlen // 2, 4096)
                buf = self.buf
                for t0, vals in player.stream.take_batches():
                    buf.extend(vals)
                # keep latency bounded when samples arrive faster than
                # we play them (high speedup free run)
                if len(buf) > SAMPLE_RATE:
                    del buf[:len(buf) - SAMPLE_RATE // 2]
                    self.pos = 0.0
                if n <= 0:
                    return b''
                # consume ratio from buffer fill: equilibrium is
                # fill/target = arrival/output rate
                target = SAMPLE_RATE / 10.0
                ratio = max(0.0, min(4.0, (len(buf) - self.pos) / target))
                if np is not None:
                    return self._read_np(n, ratio)
                out = bytearray()
                for _ in range(n):
                    ipos = int(self.pos)
                    if ipos + 1 < len(buf):
                        frac = self.pos - ipos
                        x = buf[ipos] * (1 - frac) + buf[ipos + 1] * frac
                        self.pos += ratio
                        self.fade = min(1.0, self.fade + 0.001)
                    else:
                        x = 0.0
                        self.fade = max(0.0, self.fade - 0.001)
                    ax = abs(x)
                    if ax > self.env:
                        self.env = ax  # instant attack
                    else:
                        self.env *= 0.99997  # ~1s decay
                    g = player.volume * self.fade * 0.6 / max(self.env, 1e-4)
                    out += struct.pack('<h', int(32767 * math.tanh(x * g)))
                ipos = int(self.pos)
                if ipos > 0:
                    del buf[:ipos]
                    self.pos -= ipos
                return bytes(out)

            def _read_np(self, n, ratio):
                '''vectorized resample + AGC, same shape as the scalar
                loop: the per-sample python version is a large GIL load
                at 48kHz, felt by the DShot sender thread'''
                buf = self.buf
                L = len(buf)
                pos = self.pos + ratio * np.arange(n)
                ipos = pos.astype(int)
                # positions are nondecreasing, so validity is a prefix
                m = int(np.count_nonzero(ipos + 1 < L))
                x = np.zeros(n)
                if m:
                    bufa = np.asarray(buf, dtype=np.float64)
                    ip = ipos[:m]
                    frac = pos[:m] - ip
                    x[:m] = bufa[ip] * (1.0 - frac) + bufa[ip + 1] * frac
                    self.pos = float(pos[m - 1]) + ratio
                # fade ramps up while samples flow, down on underrun
                f = np.empty(n)
                if m:
                    f[:m] = np.minimum(1.0, self.fade + 0.001 * np.arange(1, m + 1))
                top = f[m - 1] if m else self.fade
                if n > m:
                    f[m:] = np.maximum(0.0, top - 0.001 * np.arange(1, n - m + 1))
                self.fade = float(f[-1])
                # peak follower env = max(|x|, env*decay): rescale so the
                # running max becomes a cummax (decay^-k stays ~1 for n<=4096)
                d = 0.99997
                dk = d ** np.arange(n)
                c = np.maximum.accumulate(np.abs(x) / dk)
                env = dk * np.maximum(c, self.env * d)
                self.env = float(env[-1])
                g = player.volume * f * 0.6 / np.maximum(env, 1e-4)
                out = (32767 * np.tanh(x * g)).astype('<i2').tobytes()
                ip0 = int(self.pos)
                if ip0 > 0:
                    del buf[:ip0]
                    self.pos -= ip0
                return out

            def writeData(self, data):
                return -1

        def build():
            dev = QMediaDevices.defaultAudioOutput()
            if dev.isNull():
                player.error = 'no audio output device'
                return None
            fmt = QAudioFormat()
            fmt.setSampleRate(SAMPLE_RATE)
            fmt.setChannelCount(1)
            fmt.setSampleFormat(QAudioFormat.SampleFormat.Int16)
            io = PhysDevice()
            sink = QAudioSink(dev, fmt)
            sink.setBufferSize(SAMPLE_RATE // 5)
            sink.start(io)
            return sink, io

        self.thread, self.active, err = _run_sink_on_thread(build)
        if err:
            self.error = err

    def set_volume(self, volume):
        self.volume = volume

    def stop(self):
        if self.active:
            self.thread.quit()      # ends the audio thread's event loop
            self.thread.wait(2000)
            self.active = False


def parse_expected(spec):
    '''parse 428.6:0.2,585.4:0.2 into [(freq, duration)]'''
    out = []
    for part in spec.split(','):
        f, d = part.split(':')
        out.append((float(f), float(d)))
    return out


def cmd_print(stream, args):
    seen = 0
    try:
        while True:
            with stream.lock:
                events = list(stream.events)[seen:]
                seen += len(events)
            for wall_t, t_ns, freq, amp, source in events:
                print('t=%.4fs freq=%.1fHz amp=%.5f' % (t_ns * 1e-9, freq, amp))
                sys.stdout.flush()
            time.sleep(0.05)
    except KeyboardInterrupt:
        return 0


def cmd_play(stream, args):
    from PySide6.QtCore import QCoreApplication, QTimer
    app = QCoreApplication(sys.argv)
    kind = PhysicsAudio if args.physics else ToneSynth
    synth = kind(stream, volume=args.volume)
    if not synth.active:
        print('cannot play: %s' % synth.error, file=sys.stderr)
        return 1
    print('playing %s from udp %s:%u, ctrl-c to stop' %
          ('physics audio' if args.physics else 'tones',
           stream.addr[0], stream.addr[1]))
    # let KeyboardInterrupt through the Qt event loop
    poke = QTimer()
    poke.timeout.connect(lambda: None)
    poke.start(200)
    try:
        app.exec()
    except KeyboardInterrupt:
        pass
    synth.stop()
    return 0


def render_physics_wav(batches, path):
    '''write physics audio batches as a peak-normalized mono WAV'''
    vals = [v for t0, bvals in batches for v in bvals]
    peak = max((abs(v) for v in vals), default=0.0)
    scale = 0.7 * 32767 / peak if peak > 0 else 0.0
    frames = b''.join(struct.pack('<h', int(max(-32767, min(32767, v * scale))))
                      for v in vals)
    with wave.open(path, 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(SAMPLE_RATE)
        w.writeframes(frames)
    return len(vals)


def cmd_wav(stream, args):
    print('recording %.1fs of %s ...' %
          (args.seconds, 'physics audio' if args.physics else 'tone events'))
    time.sleep(args.seconds)
    if args.physics:
        nsamples = render_physics_wav(stream.take_batches(), args.out)
        print('wrote %s: %.2fs' % (args.out, nsamples / SAMPLE_RATE))
        return 0
    events = stream.take_events()
    nsamples = render_wav(events, args.out, volume=args.volume)
    print('wrote %s: %d events, %.2fs' %
          (args.out, len(events), nsamples / SAMPLE_RATE))
    return 0


def match_notes(notes, expected, freq_tol, dur_tol):
    '''find the expected note sequence as a contiguous run anywhere in
    the captured notes (a late subscriber sees a partial first note,
    and boot tunes repeat when the ESC reboots). Returns the matching
    slice or None'''
    def match(n, e):
        return (abs(n[0] - e[0]) <= e[0] * freq_tol
                and abs(n[1] - e[1]) <= dur_tol)
    for start in range(len(notes) - len(expected) + 1):
        cand = notes[start:start + len(expected)]
        if all(match(n, e) for n, e in zip(cand, expected)):
            return cand
    return None


def cmd_assert(stream, args):
    expected = parse_expected(args.notes)
    deadline = time.time() + args.timeout
    notes, found = [], None
    while time.time() < deadline:
        with stream.lock:
            events = list(stream.events)
        notes = events_to_notes(events)
        found = match_notes(notes, expected, args.freq_tol, args.dur_tol)
        if found:
            break
        time.sleep(0.05)
    if found:
        for (freq, dur, amp), (efreq, edur) in zip(found, expected):
            print('PASS: %.1fHz %.3fs (want %.1fHz %.3fs)' %
                  (freq, dur, efreq, edur))
        return 0
    print('FAIL: expected %s, captured notes:' % args.notes)
    for freq, dur, amp in notes:
        print('  %.1fHz %.3fs amp=%.5f' % (freq, dur, amp))
    return 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=57734,
                    help='SITL state port (default 57734)')
    sub = ap.add_subparsers(dest='mode')
    sub.add_parser('print', help='print tone events as they arrive')
    p = sub.add_parser('play', help='play tones on the default audio output')
    p.add_argument('--volume', type=float, default=0.5)
    p.add_argument('--physics', action='store_true',
                   help='play the physics audio stream (torque/current '
                        'ripple) instead of synthesized tones')
    p = sub.add_parser('wav', help='record events and render a WAV file')
    p.add_argument('out')
    p.add_argument('--seconds', type=float, default=5.0)
    p.add_argument('--volume', type=float, default=0.5)
    p.add_argument('--physics', action='store_true',
                   help='record the physics audio stream instead of '
                        'synthesized tones')
    p = sub.add_parser('assert', help='assert on a played note sequence')
    p.add_argument('--notes', required=True,
                   help='freq_hz:duration_s,... e.g. 428.6:0.2,585.4:0.2')
    p.add_argument('--freq-tol', type=float, default=0.02,
                   help='relative frequency tolerance (default 0.02)')
    p.add_argument('--dur-tol', type=float, default=0.05,
                   help='absolute duration tolerance, s (default 0.05)')
    p.add_argument('--timeout', type=float, default=10.0)
    args = ap.parse_args()
    mode = args.mode or 'print'

    if getattr(args, 'physics', False):
        maxlen = 2048
        if mode == 'wav':
            # hold the whole recording (batches of 64 samples)
            maxlen = int(args.seconds * SAMPLE_RATE / 64) + 64
        stream = AudioStream(args.host, args.port, maxlen=maxlen)
    else:
        stream = ToneStream(args.host, args.port)
    try:
        return {'print': cmd_print, 'play': cmd_play,
                'wav': cmd_wav, 'assert': cmd_assert}[mode](stream, args)
    finally:
        stream.close()


if __name__ == '__main__':
    sys.exit(main())
