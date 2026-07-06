"""Shared test fixtures.

``host_perf_elf`` compiles the real firmware header (Inc/hwci_perf.h) with the
host C compiler into a small ELF carrying DWARF for ``hwci_perf_s`` and the
``hwci_perf`` symbol. The struct is naturally aligned with members <= 4 bytes,
so its layout is identical on the host and on the Cortex-M0 target.
"""
import shutil
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
HEADER_DIR = REPO_ROOT / "Inc"
_CC = shutil.which("cc") or shutil.which("gcc")

_PROBE_C = """\
#define HWCI_PERF 1
#include "hwci_perf.h"
volatile hwci_perf_t hwci_perf = {
  .magic = HWCI_PERF_MAGIC, .version = HWCI_PERF_VERSION,
  .size = (uint16_t)sizeof(hwci_perf_t) };
void hwci_perf_apply_cmd(void) {}
int main(void){ return (int)hwci_perf.size; }
"""


# Frozen copy of the v1 struct (pre zero-cross jitter block), so the host's
# backward-compat decode path is tested against a real compiled v1 ELF - the
# A side of an A/B bench session runs firmware with exactly this layout.
_PROBE_V1_C = """\
#include <stdint.h>
typedef struct hwci_perf_s {
    uint32_t magic; uint16_t version; uint16_t size;
    uint16_t ctrl_exec_us_last; uint16_t ctrl_exec_us_max;
    uint16_t ctrl_period_us_last; uint16_t ctrl_period_us_max;
    uint16_t ctrl_period_us_min;
    uint16_t main_loop_us_last; uint16_t main_loop_us_max;
    uint16_t input; uint16_t duty_cycle; uint16_t e_rpm;
    uint16_t voltage_cv; int16_t current_ca; int16_t temperature_c;
    uint8_t bemf_timeout_state; uint8_t armed; uint8_t running;
    uint8_t _pad0; uint16_t _pad1;
    uint32_t loop_iters; uint32_t zero_cross_count;
    uint32_t commutation_interval; uint32_t commutation_interval_max;
    uint32_t update_count; volatile uint32_t host_cmd;
} hwci_perf_t;
volatile hwci_perf_t hwci_perf = {
  .magic = 0x31435748u, .version = 1, .size = (uint16_t)sizeof(hwci_perf_t) };
int main(void){ return (int)hwci_perf.size; }
"""


# Frozen copy of the v2 struct (zero-cross jitter block, pre v3 confirm-reject
# counter), so the host keeps decoding v2 firmware: an A/B bench session
# flashes vintages with exactly this layout.
_PROBE_V2_C = """\
#include <stdint.h>
typedef struct hwci_perf_s {
    uint32_t magic; uint16_t version; uint16_t size;
    uint16_t ctrl_exec_us_last; uint16_t ctrl_exec_us_max;
    uint16_t ctrl_period_us_last; uint16_t ctrl_period_us_max;
    uint16_t ctrl_period_us_min;
    uint16_t main_loop_us_last; uint16_t main_loop_us_max;
    uint16_t input; uint16_t duty_cycle; uint16_t e_rpm;
    uint16_t voltage_cv; int16_t current_ca; int16_t temperature_c;
    uint8_t bemf_timeout_state; uint8_t armed; uint8_t running;
    uint8_t _pad0; uint16_t _pad1;
    uint32_t loop_iters; uint32_t zero_cross_count;
    uint32_t commutation_interval; uint32_t commutation_interval_max;
    uint32_t update_count; volatile uint32_t host_cmd;
    uint32_t zc_count; uint32_t zc_jitter_sum; uint32_t zc_interval_sum;
    uint16_t zc_jitter_max; uint16_t _pad2;
} hwci_perf_t;
volatile hwci_perf_t hwci_perf = {
  .magic = 0x31435748u, .version = 2, .size = (uint16_t)sizeof(hwci_perf_t) };
int main(void){ return (int)hwci_perf.size; }
"""


def _compile_probe(tmp_path_factory, name: str, source: str,
                   include_dir=None) -> str:
    pytest.importorskip("elftools")
    if _CC is None:
        pytest.skip("no host C compiler available")
    d = tmp_path_factory.mktemp(name)
    src = d / "probe.c"
    src.write_text(source)
    out = d / "probe.elf"
    cmd = [_CC, "-g", "-O0"]
    if include_dir is not None:
        cmd.append(f"-I{include_dir}")
    subprocess.run(cmd + [str(src), "-o", str(out)],
                   check=True, capture_output=True)
    return str(out)


@pytest.fixture(scope="session")
def host_perf_elf(tmp_path_factory):
    if not (HEADER_DIR / "hwci_perf.h").exists():
        pytest.skip("Inc/hwci_perf.h not found")
    return _compile_probe(tmp_path_factory, "perf_elf", _PROBE_C, HEADER_DIR)


@pytest.fixture(scope="session")
def host_perf_elf_v1(tmp_path_factory):
    return _compile_probe(tmp_path_factory, "perf_elf_v1", _PROBE_V1_C)


@pytest.fixture(scope="session")
def host_perf_elf_v2(tmp_path_factory):
    return _compile_probe(tmp_path_factory, "perf_elf_v2", _PROBE_V2_C)


# Frozen copy of the v3 struct (confirm-reject counter, pre v4 phase
# histogram), for the decode-compat regression like the v1/v2 probes.
_PROBE_V3_C = _PROBE_V2_C.replace(
    "uint16_t zc_jitter_max; uint16_t _pad2;",
    "uint16_t zc_jitter_max; uint16_t _pad2;\n    uint32_t zc_confirm_reject;"
).replace(".version = 2,", ".version = 3,")


@pytest.fixture(scope="session")
def host_perf_elf_v3(tmp_path_factory):
    return _compile_probe(tmp_path_factory, "perf_elf_v3", _PROBE_V3_C)


# EEprom settings probe: compiles the REAL Inc/eeprom.h on the host so the
# DWARF union-walk cross-check in hwci.settings is tested against the actual
# firmware layout. eeprom.h starts with `#include "main.h"` (an MCU-specific
# header), so the header is copied next to a stub main.h that only provides
# <stdint.h> - all EEprom_t needs. Members are uint8_t, so host and Cortex-M0
# layouts are identical.
_PROBE_EEPROM_C = """\
#include "eeprom.h"
EEprom_t eepromBuffer;
void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len) {(void)data;(void)add;(void)out_buff_len;}
void save_flash_nolib(uint8_t* data, int length, uint32_t add) {(void)data;(void)length;(void)add;}
int main(void){ return (int)sizeof(EEprom_t); }
"""


@pytest.fixture(scope="session")
def host_eeprom_elf(tmp_path_factory):
    if not (HEADER_DIR / "eeprom.h").exists():
        pytest.skip("Inc/eeprom.h not found")
    pytest.importorskip("elftools")
    if _CC is None:
        pytest.skip("no host C compiler available")
    d = tmp_path_factory.mktemp("eeprom_elf")
    (d / "main.h").write_text("#include <stdint.h>\n")
    (d / "eeprom.h").write_text((HEADER_DIR / "eeprom.h").read_text())
    src = d / "probe.c"
    src.write_text(_PROBE_EEPROM_C)
    out = d / "probe.elf"
    subprocess.run([_CC, "-g", "-O0", f"-I{d}", str(src), "-o", str(out)],
                   check=True, capture_output=True)
    return str(out)
