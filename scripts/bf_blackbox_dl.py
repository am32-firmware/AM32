#!/usr/bin/env python3
'''
download Betaflight blackbox logs from an FC's SD card over USB

Betaflight cannot serve SD card logs over MSP, but it can reboot into
USB Mass Storage mode, where the card appears as an ordinary block
device. This drives that: reboot to MSC, wait for the device, mount it,
copy the logs off, unmount and reboot back to firmware.

Mounting uses udisksctl so no root is needed (the FC enumerates as a
removable device). --format rewrites the card as FAT32, which is worth
knowing about because Betaflight refuses cards whose filesystem it
cannot parse ("FS: Fatal - bad FAT header") - that needs root.

usage:
  bf_blackbox_dl.py --port /dev/ttyACM0 --out logs/
  bf_blackbox_dl.py --port /dev/ttyACM0 --msc-only     # just enter MSC
'''

import argparse
import glob
import os
import shutil
import subprocess
import sys
import time

import serial

import msp

MSP_REBOOT = 68
MSP_REBOOT_FIRMWARE = 0
MSP_REBOOT_MSC = 2


def fc_ports():
    return set(glob.glob('/dev/serial/by-id/*'))


WINDOWS = sys.platform.startswith('win')
MACOS = sys.platform == 'darwin'


def storage_volumes():
    '''set of currently attached removable storage volumes.

    Windows and macOS mount removable media automatically, so a volume
    is a path we can read directly. On Linux nothing is mounted yet, so
    this returns block device names and mount() does the rest'''
    if WINDOWS:
        import string
        import ctypes
        mask = ctypes.windll.kernel32.GetLogicalDrives()
        return {'%s:\\' % c for i, c in enumerate(string.ascii_uppercase)
                if mask & (1 << i)}
    if MACOS:
        return set(glob.glob('/Volumes/*'))
    out = set()
    for d in glob.glob('/sys/block/sd*'):
        try:
            with open(os.path.join(d, 'removable')) as f:
                if f.read().strip() != '1':
                    continue
        except OSError:
            continue
        out.add(os.path.basename(d))
    return out


def enter_msc(port, timeout=25.0):
    '''reboot the FC into mass storage mode, return the new volume
    (a drive letter on Windows, a mount point on macOS, a block device
    on Linux)'''
    before = storage_volumes()
    p = msp.MspPort(port)
    try:
        p.send(MSP_REBOOT, bytes([MSP_REBOOT_MSC]))
        time.sleep(1.0)
    finally:
        try:
            p.close()
        except Exception:
            pass
    print('rebooting the FC into USB mass storage mode...')
    deadline = time.time() + timeout
    while time.time() < deadline:
        new = storage_volumes() - before
        if new:
            vol = sorted(new)[0]
            time.sleep(1.5)          # let the volume settle
            return vol if (WINDOWS or MACOS) else '/dev/' + vol
        time.sleep(0.5)
    return None


def leave_msc(port_glob, timeout=30.0):
    '''MSC mode only exits on a power cycle or a USB reset; ask the user
    if the FC does not come back on its own'''
    deadline = time.time() + timeout
    while time.time() < deadline:
        hits = [p for p in glob.glob(port_glob) if 'Betaflight' in p]
        if hits:
            return hits[0]
        time.sleep(1.0)
    return None


def mount(dev):
    '''make the card readable, returning (path, handle-to-unmount).
    Windows and macOS have already mounted it for us'''
    if WINDOWS or MACOS:
        return dev, None
    # the partition node and its udisks object appear a moment after the
    # disk itself, so retry rather than failing on the first attempt
    deadline = time.time() + 20
    target = dev
    while time.time() < deadline:
        target = dev + '1' if os.path.exists(dev + '1') else dev
        r = subprocess.run(['udisksctl', 'mount', '-b', target],
                           capture_output=True, text=True)
        out = (r.stdout + r.stderr).strip()
        if 'Mounted' in out and ' at ' in out:
            # udisksctl prints: Mounted /dev/sda1 at `/run/media/u/NAME'.
            path = out.rsplit(' at ', 1)[1].strip().rstrip('.')
            return path.strip('`\'"'), target
        if 'AlreadyMounted' in out:
            r2 = subprocess.run(['findmnt', '-n', '-o', 'TARGET', target],
                                capture_output=True, text=True)
            if r2.stdout.strip():
                return r2.stdout.strip().splitlines()[0], target
        time.sleep(1.0)
    print('mount failed: %s' % out)
    return None, target


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=None, help='FC serial port (auto-detected if omitted)')
    ap.add_argument('--out', default='blackbox_logs',
                    help='directory to copy the logs into')
    ap.add_argument('--msc-only', action='store_true',
                    help='enter MSC mode and stop, leaving the card mounted')
    ap.add_argument('--format', action='store_true',
                    help='format the card as FAT32 first (needs root, DESTROYS DATA)')
    args = ap.parse_args()

    args.port = msp.find_fc_port(args.port)
    dev = enter_msc(args.port)
    if dev is None:
        raise SystemExit('no mass storage device appeared - does this FC '
                         'build have USE_USB_MSC, and is a card inserted?')
    print('mass storage device: %s' % dev)

    if args.format:
        if WINDOWS or MACOS:
            raise SystemExit('--format is Linux only; format the card with '
                             'the OS disk utility (FAT32) instead')
        part = dev + '1'
        print('formatting %s as FAT32 (all data lost)...' % dev)
        subprocess.run(['sudo', 'parted', '-s', dev, 'mklabel', 'msdos',
                        'mkpart', 'primary', 'fat32', '1MiB', '100%'],
                       check=True)
        time.sleep(2)
        subprocess.run(['sudo', 'mkfs.vfat', '-F', '32', '-n', 'BLACKBOX',
                        part if os.path.exists(part) else dev], check=True)
        print('formatted; power cycle the FC so Betaflight re-reads the card')
        return

    mnt, part = mount(dev)
    if mnt is None:
        raise SystemExit('could not mount %s' % dev)
    print('mounted at %s' % mnt)

    if args.msc_only:
        print('left mounted; unmount with: udisksctl unmount -b %s' % part)
        return

    # Betaflight names SD card logs LOG00001.BFL; the dataflash path and
    # some tools use .BBL, so accept both
    logs = []
    for pat in ('*.BFL', '*.bfl', '*.BBL', '*.bbl'):
        logs += glob.glob(os.path.join(mnt, '**', pat), recursive=True)
    logs = sorted(set(logs))
    os.makedirs(args.out, exist_ok=True)
    if not logs:
        print('no blackbox logs found on the card')
    for f in logs:
        dst = os.path.join(args.out, os.path.basename(f))
        shutil.copy2(f, dst)
        print('copied %s (%u bytes)' % (dst, os.path.getsize(dst)))

    if part is not None:
        subprocess.run(['udisksctl', 'unmount', '-b', part],
                       capture_output=True, text=True)
    print('unmounted. Power cycle the FC to leave mass storage mode '
          '(MSC does not exit on its own).')


if __name__ == '__main__':
    main()
