"""AM32 EEPROM settings blob: encode/apply/diff, range refusal, address
resolution, and the DWARF cross-check against a host-compiled Inc/eeprom.h."""
import pytest

from hwci import settings as st
from hwci.debugger.base import MockDebugger


def test_default_blob_round_trips_known_fields():
    s = st.Settings(st.default_blob())
    assert s.get("advance_level") == 26
    assert s.get("pwm_frequency") == 24
    assert s.get("variable_pwm") == 1
    assert s.get("auto_advance") == 0
    assert s.get("max_ramp") == 160
    assert s.get("startup_power") == 100


def test_apply_returns_mutated_copy_and_diff_names_fields():
    base = st.Settings(st.default_blob())
    trial = base.apply({"advance_level": 30, "auto_advance": 1})
    # original untouched
    assert base.get("advance_level") == 26
    assert trial.get("advance_level") == 30
    d = base.diff(trial)
    assert ("advance_level", 26, 30) in d
    assert ("auto_advance", 0, 1) in d
    assert len(d) == 2
    # only the tuned bytes changed - identity/calibration preserved
    assert trial.to_bytes()[1] == base.to_bytes()[1]
    assert trial.to_bytes()[3:5] == base.to_bytes()[3:5]


def test_bin_round_trip(tmp_path):
    s = st.Settings(st.default_blob()).apply({"pwm_frequency": 48})
    p = s.to_bin(tmp_path / "settings.bin")
    assert p.read_bytes() == s.to_bytes()
    assert st.Settings.from_bin(p) == s


@pytest.mark.parametrize("name,value", [
    ("advance_level", 9),      # firmware treats <10 as legacy format
    ("advance_level", 43),     # firmware falls back to 16
    ("pwm_frequency", 7),      # firmware keeps compile-time ARR
    ("pwm_frequency", 145),
    ("startup_power", 49),
    ("startup_power", 151),
    ("variable_pwm", 3),
    ("auto_advance", 2),
    ("max_ramp", 0),
    ("max_ramp", 256),
])
def test_out_of_range_values_are_refused_not_clamped(name, value):
    base = st.Settings(st.default_blob())
    with pytest.raises(st.SettingsError, match="range"):
        base.apply({name: value})


def test_unknown_name_requires_explicit_offset():
    base = st.Settings(st.default_blob())
    with pytest.raises(st.SettingsError, match="unknown setting"):
        base.apply({"warp_drive": 1})
    # explicit offset addressing (forward-compat with newer firmware fields)
    out = base.apply({"warp_drive": 7}, offsets={"warp_drive": 60})
    assert out.to_bytes()[60] == 7


def test_explicit_offset_refuses_identity_bytes():
    with pytest.raises(st.SettingsError, match="read-only"):
        st.resolve_field("hack", offset=3)


def test_explicit_offset_must_match_known_field():
    with pytest.raises(st.SettingsError, match="contradicts"):
        st.resolve_field("advance_level", offset=24)


def test_wrong_blob_size_rejected():
    with pytest.raises(st.SettingsError, match="192"):
        st.Settings(b"\x00" * 191)


def test_from_device_reads_page(tmp_path):
    dbg = MockDebugger(base=st.DEFAULT_EEPROM_ADDRESS, size=1024)
    dbg.poke(st.DEFAULT_EEPROM_ADDRESS, st.default_blob())
    s = st.Settings.from_device(dbg, st.DEFAULT_EEPROM_ADDRESS)
    assert s.get("advance_level") == 26


# --------------------------------------------------------------------------
# live eeprom_address resolution
# --------------------------------------------------------------------------
def test_resolve_eeprom_address_sanity_checks(monkeypatch):
    from hwci import elf as elfmod
    monkeypatch.setattr(
        st.elf, "find_symbol",
        lambda path, name: elfmod.Symbol(name=name, address=0x20000010, size=4))
    dbg = MockDebugger(base=0x20000000, size=0x100)
    dbg.poke(0x20000010, (0x08007C00).to_bytes(4, "little"))
    assert st.resolve_eeprom_address(dbg, "fake.elf") == 0x08007C00
    dbg.poke(0x20000010, (0x08001234).to_bytes(4, "little"))
    with pytest.raises(st.SettingsError, match="refusing"):
        st.resolve_eeprom_address(dbg, "fake.elf")


# --------------------------------------------------------------------------
# DWARF union-walk cross-check vs the real firmware header
# --------------------------------------------------------------------------
def test_union_layout_flattens_anonymous_struct(host_eeprom_elf):
    from hwci import elf
    members = {m.name: m for m in elf.union_layout(host_eeprom_elf, "EEprom_u")}
    # direct members of the anonymous first struct
    assert members["advance_level"].offset == 23
    assert members["pwm_frequency"].offset == 24
    assert members["auto_advance"].offset == 47
    # nested NAMED structs flatten with dotted names
    assert members["version.major"].offset == 3
    assert members["version.minor"].offset == 4
    assert members["servo.dead_band"].offset == 35
    assert members["can.esc_index"].offset == 177
    # union's second member overlaps at offset 0
    assert members["buffer"].offset == 0


def test_eeprom_fields_match_firmware_header(host_eeprom_elf):
    st.check_eeprom_layout(host_eeprom_elf)


def test_layout_check_fails_hard_on_missing_union(host_perf_elf):
    # DWARF is present but has no EEprom_u: exactly the drift the check must
    # catch loudly (renamed union, stripped types).
    with pytest.raises(st.SettingsError, match="cross-check"):
        st.check_eeprom_layout(host_perf_elf)


def test_layout_check_soft_skips_without_dwarf(monkeypatch):
    # Stripped debug info (no DWARF at all) only warns - the canonical
    # offsets still apply - mirroring PerfReader._check_layout's policy.
    from hwci import elf

    def no_dwarf(path, name):
        raise elf.ElfError("no DWARF info")

    monkeypatch.setattr(st.elf, "union_layout", no_dwarf)
    with pytest.warns(UserWarning, match="cross-check skipped"):
        st.check_eeprom_layout("whatever.elf")
