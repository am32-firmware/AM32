"""Locate the ``hwci_perf`` struct inside a built firmware ELF.

The host never hard-codes a RAM address. It reads the address (and, when DWARF
is present, the full member layout) straight from the ELF that was flashed, so
the harness stays correct across firmware revisions and across the four
identical STM32F051 dies on the ARK 4IN1 (they share one image).

Requires ``pyelftools``.
"""
from __future__ import annotations

from dataclasses import dataclass

try:
    from elftools.elf.elffile import ELFFile
    _HAVE_ELFTOOLS = True
except ImportError:  # pragma: no cover - exercised only without the dep
    _HAVE_ELFTOOLS = False


class ElfError(RuntimeError):
    pass


class StructNotFoundError(ElfError):
    """DWARF is present but the requested struct DIE is not.

    Distinct from "no DWARF at all": callers that soft-skip the layout
    cross-check when debug info is stripped must still HARD-FAIL here - a
    renamed/dropped struct is exactly the drift the check exists to catch."""


def _require_elftools() -> None:
    if not _HAVE_ELFTOOLS:
        raise ElfError(
            "pyelftools is required to read firmware symbols; "
            "pip install pyelftools")


@dataclass
class Symbol:
    name: str
    address: int
    size: int


def find_symbol(elf_path: str, name: str) -> Symbol:
    """Return address+size of ``name`` from the ELF symbol table."""
    _require_elftools()
    with open(elf_path, "rb") as fh:
        elf = ELFFile(fh)
        symtab = elf.get_section_by_name(".symtab")
        if symtab is None:
            raise ElfError(f"{elf_path}: no .symtab (build with debug symbols)")
        matches = [s for s in symtab.iter_symbols() if s.name == name]
        if not matches:
            raise ElfError(
                f"symbol {name!r} not found in {elf_path}; "
                "is the firmware built with HWCI_PERF=1?")
        sym = matches[0]
        return Symbol(name=name,
                      address=sym["st_value"],
                      size=sym["st_size"])


# Map (DWARF encoding, byte_size) -> struct code, matching perf.FIELDS codes.
_ENCODING_SIGNED = {5, 6, 0xd}  # DW_ATE_signed / signed_char / (vendor)


@dataclass
class Member:
    name: str
    offset: int
    size: int
    signed: bool


def struct_layout(elf_path: str, type_name: str) -> list[Member]:
    """Read the member layout of ``struct <type_name>`` from DWARF debug info.

    Returns members in offset order. Raises :class:`ElfError` if the ELF has no
    DWARF (firmware must be compiled with -g, which AM32 does by default).
    """
    _require_elftools()
    with open(elf_path, "rb") as fh:
        elf = ELFFile(fh)
        if not elf.has_dwarf_info():
            raise ElfError(f"{elf_path}: no DWARF info")
        dwarf = elf.get_dwarf_info()
        for cu in dwarf.iter_CUs():
            for die in cu.iter_DIEs():
                if die.tag != "DW_TAG_structure_type":
                    continue
                name_attr = die.attributes.get("DW_AT_name")
                if name_attr is None:
                    continue
                if name_attr.value.decode("utf-8", "replace") != type_name:
                    continue
                return _members_of(die, cu)
    raise StructNotFoundError(
        f"struct {type_name!r} not found in DWARF of {elf_path}")


def _members_of(struct_die, cu) -> list[Member]:
    members: list[Member] = []
    for child in struct_die.iter_children():
        if child.tag != "DW_TAG_member":
            continue
        name = child.attributes["DW_AT_name"].value.decode("utf-8", "replace")
        offset = child.attributes.get("DW_AT_data_member_location")
        offset_val = offset.value if offset is not None else 0
        if isinstance(offset_val, list):  # location expression form
            # DW_OP_plus_uconst <n> => [0x23, n]
            offset_val = offset_val[1] if len(offset_val) > 1 else 0
        size, signed = _base_type(child, cu)
        members.append(Member(name=name, offset=offset_val,
                              size=size, signed=signed))
    members.sort(key=lambda m: m.offset)
    return members


def _base_type(member_die, cu) -> tuple[int, bool]:
    type_ref = member_die.attributes.get("DW_AT_type")
    if type_ref is None:
        return 0, False
    die = cu.get_DIE_from_refaddr(type_ref.value + cu.cu_offset)
    # Walk through typedef/const/volatile qualifiers to the base type.
    while die.tag in ("DW_TAG_typedef", "DW_TAG_const_type",
                      "DW_TAG_volatile_type"):
        nxt = die.attributes.get("DW_AT_type")
        if nxt is None:
            return 0, False
        die = cu.get_DIE_from_refaddr(nxt.value + cu.cu_offset)
    size_attr = die.attributes.get("DW_AT_byte_size")
    size = size_attr.value if size_attr is not None else 0
    enc_attr = die.attributes.get("DW_AT_encoding")
    signed = bool(enc_attr and enc_attr.value in _ENCODING_SIGNED)
    return size, signed
