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


@pytest.fixture(scope="session")
def host_perf_elf(tmp_path_factory):
    pytest.importorskip("elftools")
    if _CC is None:
        pytest.skip("no host C compiler available")
    if not (HEADER_DIR / "hwci_perf.h").exists():
        pytest.skip("Inc/hwci_perf.h not found")
    d = tmp_path_factory.mktemp("perf_elf")
    src = d / "probe.c"
    src.write_text(_PROBE_C)
    out = d / "probe.elf"
    subprocess.run(
        [_CC, "-g", "-O0", f"-I{HEADER_DIR}", str(src), "-o", str(out)],
        check=True, capture_output=True)
    return str(out)
