#!/usr/bin/env python3
'''
launch the AM32 SITL binary for the SITL control GUI

Lets the GUI run the simulator itself instead of the user starting it
separately: it launches the binary on the ports the GUI already drives
(UDP PWM/DShot input, the state stream, and the CAN multicast bus) and
streams its output. A bundled binary ships with the packaged build, but
the user can point at their own, and can still run the simulator apart
from the GUI and just leave the launcher stopped.
'''

import os
import subprocess
import sys
import threading
from collections import deque


def _resource_dir():
    '''where bundled data lives: the PyInstaller unpack dir when frozen,
    otherwise the repo root (this file is at Mcu/SITL/)'''
    if getattr(sys, 'frozen', False):
        return getattr(sys, '_MEIPASS', os.path.dirname(sys.executable))
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.dirname(os.path.dirname(here))


def _exe(name):
    return name + '.exe' if sys.platform.startswith('win') else name


def bundled_sitl():
    '''the SITL firmware binary shipped with the build, if any'''
    base = _resource_dir()
    for cand in (os.path.join(base, 'sitl', _exe('AM32_SITL_CAN')),
                 os.path.join(base, _exe('AM32_SITL_CAN'))):
        if os.path.isfile(cand):
            return cand
    import glob
    for pat in ('obj/AM32_AM32_SITL_CAN_*.elf', 'obj/AM32_SITL_CAN*'):
        hits = sorted(glob.glob(os.path.join(base, pat)))
        if hits:
            return hits[0]
    return None


def bundled_eeprom():
    '''a default eeprom image to seed the simulated ESC.

    The packaged build ships one generated at build time. From a
    checkout there is none to ship, so generate it from the parameter
    file into build/ - the simulator opens its eeprom read/write, and
    handing it a file in the source tree corrupts the tree.
    '''
    base = _resource_dir()
    packaged = os.path.join(base, 'sitl', 'default_eeprom.bin')
    if os.path.isfile(packaged):
        return packaged
    here = os.path.dirname(os.path.abspath(__file__))
    params = os.path.join(here, 'data', 'VIMDRONES_NANO_2216', 'sitl.param')
    if not os.path.isfile(params):
        return None
    out = os.path.join(base, 'build', 'sitl_gui', 'default_eeprom.bin')
    if (not os.path.isfile(out) or
            os.path.getmtime(out) < os.path.getmtime(params)):
        import sitl_params
        os.makedirs(os.path.dirname(out), exist_ok=True)
        sitl_params.write_eeprom(params, out)
    return out


# On Windows the SITL emulates an MCU reset by re-exec (execv has no true
# equivalent), so a no-signal reboot spawns a fresh PID and the original
# Popen handle goes dead - the launcher would then think the sim stopped
# and grey out Stop while a new SITL keeps running. Putting the SITL in a
# Job Object fixes both tracking and teardown: a re-exec'd child inherits
# its parent's job, so the job's live-process count still reflects the sim
# and terminating the job kills whichever PID is current.
def _win_job_start(pid):
    import ctypes
    from ctypes import wintypes
    ULONG_PTR = ctypes.c_size_t

    class BASIC_LIMIT(ctypes.Structure):
        _fields_ = [('PerProcessUserTimeLimit', wintypes.LARGE_INTEGER),
                    ('PerJobUserTimeLimit', wintypes.LARGE_INTEGER),
                    ('LimitFlags', wintypes.DWORD),
                    ('MinimumWorkingSetSize', ctypes.c_size_t),
                    ('MaximumWorkingSetSize', ctypes.c_size_t),
                    ('ActiveProcessLimit', wintypes.DWORD),
                    ('Affinity', ULONG_PTR),
                    ('PriorityClass', wintypes.DWORD),
                    ('SchedulingClass', wintypes.DWORD)]

    class IO_COUNTERS(ctypes.Structure):
        _fields_ = [('ReadOperationCount', ctypes.c_ulonglong),
                    ('WriteOperationCount', ctypes.c_ulonglong),
                    ('OtherOperationCount', ctypes.c_ulonglong),
                    ('ReadTransferCount', ctypes.c_ulonglong),
                    ('WriteTransferCount', ctypes.c_ulonglong),
                    ('OtherTransferCount', ctypes.c_ulonglong)]

    class EXTENDED_LIMIT(ctypes.Structure):
        _fields_ = [('BasicLimitInformation', BASIC_LIMIT),
                    ('IoInfo', IO_COUNTERS),
                    ('ProcessMemoryLimit', ctypes.c_size_t),
                    ('JobMemoryLimit', ctypes.c_size_t),
                    ('PeakProcessMemoryUsed', ctypes.c_size_t),
                    ('PeakJobMemoryUsed', ctypes.c_size_t)]

    KILL_ON_JOB_CLOSE = 0x2000
    EXTENDED_LIMIT_CLASS = 9
    PROCESS_TERMINATE = 0x0001
    PROCESS_SET_QUOTA = 0x0100
    k32 = ctypes.WinDLL('kernel32', use_last_error=True)
    k32.CreateJobObjectW.restype = wintypes.HANDLE
    k32.OpenProcess.restype = wintypes.HANDLE
    hjob = k32.CreateJobObjectW(None, None)
    if not hjob:
        return None
    info = EXTENDED_LIMIT()
    # kill the whole tree if the GUI exits without stopping it
    info.BasicLimitInformation.LimitFlags = KILL_ON_JOB_CLOSE
    hproc = k32.OpenProcess(PROCESS_SET_QUOTA | PROCESS_TERMINATE, False, pid)
    if (not k32.SetInformationJobObject(hjob, EXTENDED_LIMIT_CLASS,
                                        ctypes.byref(info), ctypes.sizeof(info))
            or not hproc or not k32.AssignProcessToJobObject(hjob, hproc)):
        if hproc:
            k32.CloseHandle(hproc)
        k32.CloseHandle(hjob)
        return None
    k32.CloseHandle(hproc)
    return (k32, hjob)


def _win_job_active(job):
    import ctypes
    from ctypes import wintypes
    k32, hjob = job

    class ACCOUNTING(ctypes.Structure):
        _fields_ = [('TotalUserTime', wintypes.LARGE_INTEGER),
                    ('TotalKernelTime', wintypes.LARGE_INTEGER),
                    ('ThisPeriodTotalUserTime', wintypes.LARGE_INTEGER),
                    ('ThisPeriodTotalKernelTime', wintypes.LARGE_INTEGER),
                    ('TotalPageFaultCount', wintypes.DWORD),
                    ('TotalProcesses', wintypes.DWORD),
                    ('ActiveProcesses', wintypes.DWORD),
                    ('TotalTerminatedProcesses', wintypes.DWORD)]

    ACCOUNTING_CLASS = 1
    info = ACCOUNTING()
    if not k32.QueryInformationJobObject(hjob, ACCOUNTING_CLASS,
                                         ctypes.byref(info),
                                         ctypes.sizeof(info), None):
        return False
    return info.ActiveProcesses > 0


def _win_job_kill(job):
    k32, hjob = job
    try:
        k32.TerminateJobObject(hjob, 1)
    finally:
        k32.CloseHandle(hjob)


class SimRunner(object):
    '''owns the SITL child process and its output.

    start() launches it with every input the GUI uses enabled; the
    stdout is streamed to the callback and kept in a ring buffer so a
    late-attached view can show recent output.
    '''

    def __init__(self, line_cb=None):
        self.line_cb = line_cb or (lambda s: None)
        self.proc = None
        self._requested_stop = None
        self.job = None      # Windows Job Object handle, tracks re-execs
        self.output = deque(maxlen=1000)
        self._reader = None

    def start(self, binary, eeprom, model=None, host='127.0.0.1',
              input_port=57733, state_port=57734, can_uri='mcast:0',
              bootloader=None, verbose=False, input_type=None,
              high_accuracy=False):
        self.stop()
        if not binary or not os.path.isfile(binary):
            raise RuntimeError('no simulator binary - build the SITL or pick '
                               'one with Browse')
        if not eeprom or not os.path.isfile(eeprom):
            raise RuntimeError('no eeprom file for the simulator')
        cmd = [binary,
               '--input-port', str(input_port),
               '--state-port', str(state_port),
               '--can-uri', can_uri,
               '--eeprom', eeprom]
        if model and os.path.isfile(model):
            cmd += ['--config', model]
        if bootloader and os.path.isfile(bootloader):
            cmd += ['--bootloader', bootloader]
        # force the firmware's input mode (0 auto, 1 dshot, 5 dronecan) so
        # the pane the user drives actually reaches the ESC - a
        # DroneCAN-configured eeprom otherwise ignores DShot input
        if input_type is not None:
            cmd += ['--input-type', str(input_type)]
        # --nosleep busy-waits to hold the sim ratio steady; without it the
        # sim paces with sleep and frees the core for the UI, but on a
        # CPU-constrained machine the pacing jitters and the motor desyncs.
        # Offer it as a high-accuracy option
        if high_accuracy:
            cmd.append('--nosleep')
        if verbose:
            cmd.append('--verbose')
        # a binary unpacked from a onefile bundle can lose its exec bit
        if not sys.platform.startswith('win'):
            try:
                os.chmod(binary, os.stat(binary).st_mode | 0o111)
            except OSError:
                pass
        self._log('launching: %s' % ' '.join(
            os.path.basename(c) if i == 0 else c
            for i, c in enumerate(cmd)))
        self.proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1)
        self._requested_stop = None
        if sys.platform.startswith('win'):
            try:
                self.job = _win_job_start(self.proc.pid)
            except Exception:
                self.job = None
        self._reader = threading.Thread(target=self._pump, daemon=True)
        self._reader.start()

    def stop(self):
        if self.job is not None:
            # kills the current SITL PID even after a reset re-exec
            try:
                _win_job_kill(self.job)
            except Exception:
                pass
            self.job = None
        if self.proc is not None:
            self._requested_stop = self.proc
            try:
                self.proc.terminate()
                self.proc.wait(timeout=5)
            except Exception:
                try:
                    self.proc.kill()
                except Exception:
                    pass
            self.proc = None

    def is_running(self):
        if self.job is not None:
            # the tracked PID dies on every emulated reset; the job stays
            # alive as long as some re-exec'd SITL is still in it
            try:
                return _win_job_active(self.job)
            except Exception:
                pass
        return self.proc is not None and self.proc.poll() is None

    def recent(self):
        return list(self.output)

    def _pump(self):
        proc = self.proc
        try:
            for line in proc.stdout:
                self._log(line.rstrip('\n'))
        except Exception:
            pass
        try:
            status = proc.wait()
        except Exception:
            return
        # Keep an unexpected early exit visible in the launcher's log
        # panel; otherwise a process that dies between output lines only
        # changes the status label to "stopped" with no explanation.
        if (self.proc is proc and self._requested_stop is not proc
                and status != 0):
            self._log('simulator exited with status %d' % status)

    def _log(self, line):
        self.output.append(line)
        try:
            self.line_cb(line)
        except Exception:
            pass
