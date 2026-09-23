#!/usr/bin/env python3
"""Exercise startup gating and reset exit with an actual SITL executable.

python3 Mcu/SITL/tests/test_debug_lifecycle.py build/sitl/AM32_SITL_CAN
"""
import argparse
from pathlib import Path
import socket
import struct
import subprocess
import tempfile
import threading
import time


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def test_lifecycle(binary, wait_seconds):
    input_port, state_port = free_port(), free_port()
    while state_port == input_port:
        state_port = free_port()
    lines = []
    stop = threading.Event()
    sender = None
    with tempfile.TemporaryDirectory(prefix='am32-debug-test-') as tmp:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(0.2)
            proc = subprocess.Popen(
                [str(binary), '--eeprom', str(Path(tmp) / 'eeprom.bin'),
                 '--input-type', '1', '--can-uri', 'none',
                 '--input-port', str(input_port), '--state-port', str(state_port),
                 '--wait-for-input', '--exit-on-reset', '--nosleep'],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            reader = threading.Thread(target=lambda: lines.extend(proc.stdout), daemon=True)
            reader.start()
            try:
                deadline = time.monotonic() + 15
                while not any('waiting for PWM/DShot' in line for line in lines):
                    assert proc.poll() is None, ''.join(lines)
                    assert time.monotonic() < deadline, 'Startup wait was not reached'
                    time.sleep(0.05)
                # Neither malformed packets nor a static signal level should boot.
                for packet in (b'bad', struct.pack('<HBBHH', 0x4453, 5, 4, 0, 0)):
                    sock.sendto(packet, ('127.0.0.1', input_port))
                time.sleep(wait_seconds)
                assert proc.poll() is None, 'Exited before the GUI connected: ' + ''.join(lines)
                assert not any('booting firmware' in line for line in lines), ''.join(lines)
                # EEPROM queries stay available while physics/firmware are held.
                sock.sendto(struct.pack('<HBB', 0x5353, 5, 0), ('127.0.0.1', state_port))
                reply = sock.recv(4096)
                assert reply[:2] == struct.pack('<H', 0x5358), reply[:8]

                def send_zero():
                    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as signal:
                        packet = struct.pack('<HBBHH', 0x4453, 2, 4, 0, 0)
                        while not stop.wait(0.002):
                            signal.sendto(packet, ('127.0.0.1', input_port))

                sender = threading.Thread(target=send_zero, daemon=True)
                sender.start()
                deadline = time.monotonic() + 15
                got_samples = False
                while time.monotonic() < deadline:
                    sock.sendto(struct.pack('<HBBI', 0x5353, 0, 1, 1000000),
                                ('127.0.0.1', state_port))
                    try:
                        reply = sock.recv(4096)
                        if reply[:2] == struct.pack('<H', 0x5354):
                            got_samples = True
                            break
                    except socket.timeout:
                        pass
                    assert proc.poll() is None, ''.join(lines)
                assert got_samples, 'No physics samples after input arrived: ' + ''.join(lines)
                assert any('booting firmware' in line for line in lines), ''.join(lines)
                # A real simulated reset ends this run, without a successor keeping ports.
                sock.sendto(struct.pack('<HBB', 0x5353, 9, 0), ('127.0.0.1', state_port))
                assert proc.wait(timeout=10) == 0
                reader.join(timeout=2)
                assert any('--exit-on-reset: run ended' in line for line in lines), ''.join(lines)
                for port in (input_port, state_port):
                    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as check:
                        check.bind(('127.0.0.1', port))
                print('PASS: delayed input, packet filtering, EEPROM access, physics startup, reset exit and port release')
            finally:
                stop.set()
                if sender:
                    sender.join(timeout=2)
                if proc.poll() is None:
                    proc.kill()
                    proc.wait(timeout=10)
                reader.join(timeout=2)
                proc.stdout.close()


def test_diagnostic_log(binary):
    """Keep telemetry alive beyond the Windows debugger's unread pipe capacity."""
    input_port, state_port = free_port(), free_port()
    while state_port == input_port:
        state_port = free_port()
    with tempfile.TemporaryDirectory(prefix='am32-log-test-') as tmp:
        log = Path(tmp) / 'debug log.txt'
        log.write_text('Previous run\n')
        proc = subprocess.Popen(
            [str(binary), '--eeprom', str(Path(tmp) / 'eeprom.bin'),
             '--input-type', '1', '--can-uri', 'none',
             '--input-port', str(input_port), '--state-port', str(state_port),
             '--exit-on-reset', '--nosleep', '--verbose', '--log-file', str(log)],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        # Deliberately do not drain stdout/stderr while the simulator runs.
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.setblocking(False)
                deadline = time.monotonic() + 90
                next_subscribe = 0
                last_sample_ns = 0
                last_update = time.monotonic()
                while last_sample_ns < 25_000_000_000:
                    now = time.monotonic()
                    assert proc.poll() is None, log.read_text()
                    assert now < deadline, 'Physics did not reach 25 seconds'
                    assert now - last_update < 10, 'Telemetry stopped: ' + log.read_text()
                    sock.sendto(struct.pack('<HBBHH', 0x4453, 2, 4, 0, 0),
                                ('127.0.0.1', input_port))
                    if now >= next_subscribe:
                        sock.sendto(struct.pack('<HBBI', 0x5353, 0, 1, 1_000_000),
                                    ('127.0.0.1', state_port))
                        next_subscribe = now + 0.5
                    while True:
                        try:
                            reply = sock.recv(4096)
                        except (BlockingIOError, ConnectionResetError):
                            # Windows reports ICMP port-unreachable while
                            # the child is still starting and binding UDP.
                            break
                        if len(reply) >= 12 and reply[:2] == b'\x54\x53' and reply[3]:
                            stamp = struct.unpack_from('<Q', reply, 4)[0]
                            if stamp > last_sample_ns:
                                last_sample_ns = stamp
                                last_update = now
                    time.sleep(0.002)
                sock.sendto(struct.pack('<HBB', 0x5353, 9, 0), ('127.0.0.1', state_port))
                assert proc.wait(timeout=10) == 0
            diagnostics = log.read_text()
            assert diagnostics.startswith('Previous run\n'), 'Existing diagnostics were truncated'
            assert len(diagnostics) > 4096, 'Did not exercise the Windows pipe capacity'
            assert '--exit-on-reset: run ended' in diagnostics, diagnostics
            assert not proc.stdout.read(), 'Diagnostics leaked to the debugger pipe'
            print('PASS: 25 seconds of live telemetry with unread stderr, appended diagnostics and reset logging')
        finally:
            if proc.poll() is None:
                proc.kill()
                proc.wait(timeout=10)
            proc.stdout.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('binary', type=lambda p: Path(p).resolve())
    parser.add_argument('--wait-seconds', type=float, default=3)
    args = parser.parse_args()
    test_lifecycle(args.binary, args.wait_seconds)
    test_diagnostic_log(args.binary)
