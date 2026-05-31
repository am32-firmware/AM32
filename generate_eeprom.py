#!/usr/bin/env python3
"""
Keep AM32's Inc/eeprom.h and Inc/eeprom.json in sync.

eeprom.h is the hand-edited source of truth for the EEPROM layout. eeprom.json
mirrors that layout and adds the metadata (ranges, units, enums, version
gating) that configurators such as the AM32 Configurator and QGroundControl use
to build version-aware UIs.

By default this script *checks* that the committed eeprom.h matches what the
schema describes and writes nothing (used by the eeprom-schema CI job and
'make eeprom-check'). --write regenerates eeprom.h from the schema instead.

Usage:
    python generate_eeprom.py                      # check eeprom.h matches eeprom.json (no writes)
    python generate_eeprom.py --check              # same, explicit
    python generate_eeprom.py --write              # regenerate eeprom.h from the schema
    python generate_eeprom.py --url --check        # check against the hosted schema at am32.ca/eeprom
    python generate_eeprom.py path/to/schema.json  # check against a specific schema file
"""

import difflib
import json
import os
import sys
import urllib.request
from collections import OrderedDict

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SCHEMA_URL = "https://am32.ca/eeprom"
DEFAULT_SCHEMA = os.path.join(SCRIPT_DIR, "Inc", "eeprom.json")
OUTPUT_FILE = os.path.join(SCRIPT_DIR, "Inc", "eeprom.h")

# Byte width of each schema type. Array-ish types (reserved, bluejay) carry
# their length in `size`; scalar multi-byte types must declare a matching `size`.
TYPE_WIDTH = {
    'int8': 1, 'uint8': 1, 'bool': 1, 'enum': 1, 'number': 1,
    'reserved': 1, 'bluejay': 1, 'uint16': 2, 'int16': 2, 'uint32': 4,
}


def load_schema_from_url(url: str) -> dict:
    """Fetch schema from URL."""
    with urllib.request.urlopen(url) as response:
        return json.loads(response.read().decode('utf-8'))


def load_schema_from_file(path: str) -> dict:
    """Load schema from local file."""
    with open(path, 'r') as f:
        return json.load(f)


def load_schema(source: str) -> dict:
    """Load schema from a local file path or an http(s) URL."""
    if source.startswith('http://') or source.startswith('https://'):
        print(f"Fetching schema from {source}...", file=sys.stderr)
        return load_schema_from_url(source)
    return load_schema_from_file(source)


def get_c_type(field: dict) -> str:
    """Map schema type to C type."""
    field_type = field.get('type', 'uint8')
    size = field.get('size', 1)
    if field_type == 'reserved':
        return 'char' if size > 1 else 'uint8_t'
    elif field_type in ('bool', 'uint8', 'enum'):
        return 'uint8_t'
    elif field_type == 'int8':
        return 'int8_t'
    elif field_type == 'uint16':
        return 'uint16_t'
    elif field_type == 'number':
        return 'uint8_t'  # Numbers are stored as uint8 in EEPROM
    elif field_type == 'bluejay':
        return 'uint8_t'  # Array type, handled separately
    return 'uint8_t'


def validate_layout(schema: dict) -> None:
    """Validate that the EEPROM layout is internally consistent.

    Raises ValueError on the first problem found. This guards layout
    *correctness* (offsets, sizes, struct contiguity) independently of whether
    eeprom.h and eeprom.json agree with each other.
    """
    fields = schema['fields']
    buffer_size = max(
        (v.get('bufferSize', v.get('size', 192))
         for v in schema.get('eepromVersions', {}).values()),
        default=192,
    )

    ordered = sorted(fields.items(), key=lambda kv: kv[1]['offset'])

    # Fields must form a gapless, non-overlapping cover starting at offset 0:
    # the C struct packs members with no implicit gaps, so a gap or overlap
    # means the declared offsets won't match the real struct layout.
    prev_end = 0
    prev_name = None
    for name, f in ordered:
        offset = f['offset']
        size = f.get('size', 1)
        ftype = f.get('type', 'uint8')

        if offset > prev_end:
            raise ValueError(
                f"gap before '{name}': bytes {prev_end}..{offset - 1} are "
                f"unmapped (cover them with a reserved field)")
        if offset < prev_end:
            raise ValueError(
                f"'{name}' at offset {offset} overlaps '{prev_name}' which "
                f"ends at {prev_end}")
        if offset + size > buffer_size:
            raise ValueError(
                f"'{name}' [{offset}, {offset + size}) exceeds the "
                f"{buffer_size}-byte buffer")

        width = TYPE_WIDTH.get(ftype)
        if width is None:
            raise ValueError(f"'{name}' has unknown type '{ftype}'")
        if width > 1 and size != width:
            raise ValueError(
                f"'{name}' is type '{ftype}' (width {width}) but declares "
                f"size {size}")

        prev_end = offset + size
        prev_name = name

    # Nested structs (cName "prefix.member") are emitted as one contiguous
    # block, so no field outside the struct may fall within its offset span.
    groups = OrderedDict()
    for name, f in ordered:
        cname = f.get('cName', name)
        if '.' in cname:
            groups.setdefault(cname.split('.', 1)[0], []).append(f)
    for prefix, members in groups.items():
        start = min(m['offset'] for m in members)
        end = max(m['offset'] + m.get('size', 1) for m in members)
        for name, f in ordered:
            cname = f.get('cName', name)
            if '.' not in cname and start <= f['offset'] < end:
                raise ValueError(
                    f"nested struct '{prefix}' (offsets {start}..{end - 1}) is "
                    f"interleaved with non-member '{cname}' at {f['offset']}")


def generate_header(schema: dict) -> str:
    """Generate the C header file content."""
    fields = schema['fields']

    # Sort fields by offset
    sorted_fields = sorted(
        [(name, f) for name, f in fields.items()],
        key=lambda x: x[1]['offset']
    )

    lines = []
    lines.append("#include \"main.h\"")
    lines.append("")
    lines.append("#pragma once")
    lines.append("")
    lines.append("#ifndef EEPROM_H_")
    lines.append("#define EEPROM_H_")
    lines.append("")
    lines.append("typedef union EEprom_u {")
    lines.append("    struct {")

    # Pre-scan: group nested-struct members (cName "prefix.member") by prefix so
    # each struct can be emitted as one block when its first member is reached.
    nested_structs = OrderedDict()

    for name, field in sorted_fields:
        c_name = field.get('cName', None)
        if c_name is None:
            # Convert camelCase to snake_case for C
            c_name = ''.join(['_' + c.lower() if c.isupper() else c for c in name]).lstrip('_')

        if '.' in c_name:
            prefix, local_name = c_name.split('.', 1)
            nested_structs.setdefault(prefix, []).append((name, field, local_name))

    # Process all fields in offset order, handling nested structs
    processed_structs = set()

    for name, field in sorted_fields:
        offset = field['offset']
        size = field.get('size', 1)
        c_name_raw = field.get('cName', None)

        if c_name_raw is None:
            c_name_raw = ''.join(['_' + c.lower() if c.isupper() else c for c in name]).lstrip('_')

        # Check if this starts a nested struct
        if '.' in c_name_raw:
            prefix = c_name_raw.split('.')[0]
            if prefix not in processed_structs:
                processed_structs.add(prefix)
                # Output the nested struct
                struct_fields = nested_structs[prefix]
                lines.append(f"        struct {{")
                for _, sf, local_name in struct_fields:
                    sf_size = sf.get('size', 1)
                    sf_offset = sf['offset']
                    c_type = get_c_type(sf)
                    if sf_size > 1:
                        lines.append(f"            {c_type} {local_name}[{sf_size}]; // {sf_offset}")
                    else:
                        lines.append(f"            {c_type} {local_name}; // {sf_offset}")
                lines.append(f"        }} {prefix};")
        else:
            # Regular flat field
            c_type = get_c_type(field)

            # Handle array types
            if size > 1:
                lines.append(f"        {c_type} {c_name_raw}[{size}]; // {offset}")
            else:
                lines.append(f"        {c_type} {c_name_raw}; // {offset}")

    # Get max EEPROM buffer size from versions
    max_size = max(v.get('bufferSize', v.get('size', 192)) for v in schema.get('eepromVersions', {}).values())

    lines.append("    };")
    lines.append(f"    uint8_t buffer[{max_size}];")
    lines.append("} EEprom_t;")
    lines.append("")
    lines.append("extern EEprom_t eepromBuffer;")
    lines.append("")
    lines.append("void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len);")
    lines.append("void save_flash_nolib(uint8_t* data, int length, uint32_t add);")
    lines.append("")
    lines.append("#endif /* EEPROM_H_ */")
    lines.append("")

    return '\n'.join(lines)


def check_in_sync(generated: str) -> int:
    """Compare freshly generated header text against the committed eeprom.h.

    Returns a process exit code: 0 when they match, 1 otherwise. Writes nothing.
    """
    if not os.path.exists(OUTPUT_FILE):
        print(f"error: {OUTPUT_FILE} does not exist; run with --write to create it",
              file=sys.stderr)
        return 1

    with open(OUTPUT_FILE) as f:
        committed = f.read()

    if committed == generated:
        print(f"{os.path.basename(OUTPUT_FILE)} is in sync with eeprom.json")
        return 0

    diff = difflib.unified_diff(
        committed.splitlines(keepends=True),
        generated.splitlines(keepends=True),
        fromfile="eeprom.h (committed)",
        tofile="eeprom.json (what the schema describes)",
    )
    sys.stderr.writelines(diff)
    print(
        "\nerror: eeprom.h and eeprom.json have diverged.\n"
        "  - If you edited eeprom.h, update eeprom.json to match "
        "(Claude can do this from the diff above).\n"
        "  - To regenerate eeprom.h from the schema instead, run: "
        "python generate_eeprom.py --write",
        file=sys.stderr,
    )
    return 1


def main():
    # Parse arguments
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    flags = [a for a in sys.argv[1:] if a.startswith('--')]

    # Determine schema source
    if '--url' in flags:
        source = SCHEMA_URL
    elif args:
        source = args[0]
    else:
        source = DEFAULT_SCHEMA

    schema = load_schema(source)

    try:
        validate_layout(schema)
    except ValueError as e:
        print(f"error: invalid EEPROM layout: {e}", file=sys.stderr)
        sys.exit(1)

    header_content = generate_header(schema)

    # --write regenerates the header; default is a non-destructive sync check.
    if '--write' in flags:
        with open(OUTPUT_FILE, 'w') as f:
            f.write(header_content)
        print(f"Wrote {OUTPUT_FILE}")
        return

    sys.exit(check_in_sync(header_content))


if __name__ == '__main__':
    main()
