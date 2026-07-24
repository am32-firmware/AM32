#!/usr/bin/env python3
'''
headless PWM/DShot input test for the AM32 SITL

streams frames to the SITL UDP input port, handles zero-throttle arming,
then applies throttle and reports BDShot replies and rates.

note that the ESC must have INPUT_SIGNAL_TYPE set away from DRONECAN_IN
(5) or the input interrupts are disabled; use --input-type to set it over
DroneCAN first (1=DSHOT_IN, 2=SERVO_IN, 0=AUTO_IN)

examples:
  dshot_test.py --input-type 1 --type dshot300 --throttle 500
  dshot_test.py --type dshot600 --bidir --throttle 800 --edt
  dshot_test.py --input-type 2 --type pwm --throttle 1500
'''

import argparse
import sys
import time

import sitl_dshot as sd


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=57733)
    ap.add_argument('--type', default='dshot300', choices=sorted(sd.TYPE_NAMES.keys()))
    ap.add_argument('--bidir', action='store_true', help='bidirectional DShot (idle high, inverted CRC)')
    ap.add_argument('--edt', action='store_true', help='enable extended DShot telemetry after arming')
    ap.add_argument('--rate', type=float, default=500, help='frame rate Hz')
    ap.add_argument('--throttle', type=float, default=500,
                    help='DShot value 48..2047, or PWM pulse width in us')
    ap.add_argument('--arm-time', type=float, default=1.6, help='zero throttle arming time')
    ap.add_argument('--duration', type=float, default=10, help='run time at throttle')
    ap.add_argument('--poles', type=int, default=14)
    ap.add_argument('--bad-crc', type=float, default=0, help='fraction of frames sent with corrupted CRC')
    ap.add_argument('--input-type', type=int, default=None,
                    help='first set INPUT_SIGNAL_TYPE over DroneCAN (0=auto 1=dshot 2=servo 5=dronecan), save and restart')
    ap.add_argument('--can-uri', default='mcast:0')
    args = ap.parse_args()

    if args.input_type is not None:
        print('setting INPUT_SIGNAL_TYPE=%d over DroneCAN...' % args.input_type)
        node = sd.set_dronecan_param('INPUT_SIGNAL_TYPE', args.input_type, uri=args.can_uri)
        print('set on node %d, restarted' % node)
        time.sleep(1.0)

    ptype = sd.TYPE_NAMES[args.type]
    port = sd.InputPort(args.host, args.port)
    period = 1.0 / args.rate

    zero = 1000 if ptype == sd.TYPE_PWM else 0

    def send(value, telem=False, corrupt=False):
        if ptype == sd.TYPE_PWM:
            port.send_pwm(int(value))
        else:
            port.send_dshot(int(value), ptype=ptype, telem=telem,
                            bidir=args.bidir, corrupt=corrupt)

    def stream(value, duration, label):
        t0 = time.time()
        next_send = t0
        last_report = t0
        nsent = 0
        erpm_period = None
        edt = {}
        badcrc = 0
        while time.time() - t0 < duration:
            now = time.time()
            # catch-up burst: coarse sleep granularity (VMs, CI runners)
            # must not lower the average frame rate - the firmware's
            # bidirectional auto-detect needs >100 frames before arming
            # completes
            burst = 0
            while now >= next_send and burst < 10:
                next_send += period
                corrupt = args.bad_crc > 0 and (nsent % max(1, int(1 / args.bad_crc))) == 0
                send(value, corrupt=corrupt)
                nsent += 1
                burst += 1
            if now - next_send > 0.25:
                next_send = now  # fell too far behind, resync
            for r in port.get_replies():
                kind, val = sd.decode_reply(r[3], edt_expected=args.edt)
                if kind == 'erpm':
                    erpm_period = val
                elif kind == 'badcrc':
                    badcrc += 1
                else:
                    edt[kind] = val
            if now - last_report >= 1.0:
                last_report = now
                rpm = sd.erpm_period_to_rpm(erpm_period, args.poles) if erpm_period else 0
                extra = ' '.join('%s=%s' % kv for kv in sorted(edt.items()))
                print('%s: sent=%u replies=%u rpm=%.0f badcrc=%u %s'
                      % (label, port.sent_count, port.reply_count, rpm, badcrc, extra))
                sys.stdout.flush()
            time.sleep(0.0005)

    print('arming with zero throttle for %.1fs...' % args.arm_time)
    stream(zero, args.arm_time, 'arm')

    if args.edt and ptype != sd.TYPE_PWM:
        print('enabling extended DShot telemetry (cmd 13)...')
        for _ in range(8):
            send(sd.DSHOT_CMD_EDT_ENABLE, telem=True)
            time.sleep(period)

    print('throttle %g for %.1fs...' % (args.throttle, args.duration))
    stream(args.throttle, args.duration, 'run')

    print('back to zero...')
    stream(zero, 1.0, 'stop')
    port.close()


if __name__ == '__main__':
    main()
