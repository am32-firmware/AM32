#!/usr/bin/env python3
"""
Generate AM32 eeprom.h from the unified JSON schema.

This script generates the C header file that defines the EEprom_t union
used by the AM32 firmware. It ensures the struct layout matches the schema.

Usage:
    python generate_eeprom.py                        # Use local Inc/eeprom.json
    python generate_eeprom.py --url                  # Fetch from https://am32.ca/eeprom
    python generate_eeprom.py path/to/schema.json    # Use specified file
"""

import json
import os
import sys
import urllib.request
from datetime import datetime
from collections import OrderedDict

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SCHEMA_URL = "https://am32.ca/eeprom"
DEFAULT_SCHEMA = os.path.join(SCRIPT_DIR, "Inc", "eeprom.json")
OUTPUT_FILE = os.path.join(SCRIPT_DIR, "Inc", "eeprom.h")


def load_schema_from_url(url: str) -> dict:
    """Fetch schema from URL."""
    with urllib.request.urlopen(url) as response:
        return json.loads(response.read().decode('utf-8'))


def load_schema_from_file(path: str) -> dict:
    """Load schema from local file."""
    with open(path, 'r') as f:
        return json.load(f)


def load_schema(source: str = None) -> dict:
    """Load schema from file or URL (defaults to hosted schema)."""
    if source is None:
        print(f"Fetching schema from {SCHEMA_URL}...", file=sys.stderr)
        return load_schema_from_url(SCHEMA_URL)
    elif source.startswith('http://') or source.startswith('https://'):
        return load_schema_from_url(source)
    else:
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
    elif field_type == 'rtttl':
        return 'uint8_t'  # Array type, handled separately
    return 'uint8_t'


def generate_header(schema: dict) -> str:
    """Generate the C header file content."""
    fields = schema['fields']

    # Sort fields by offset
    sorted_fields = sorted(
        [(name, f) for name, f in fields.items()],
        key=lambda x: x[1]['offset']
    )

    lines = []
    lines.append(f"// AUTO-GENERATED from eeprom.json - DO NOT EDIT")
    lines.append(f"// Generated: {datetime.now().isoformat()}")
    lines.append(f"// Schema version: {schema.get('version', 'unknown')}")
    lines.append("")
    lines.append("#include \"main.h\"")
    lines.append("")
    lines.append("#pragma once")
    lines.append("")
    lines.append("typedef union EEprom_u {")
    lines.append("    struct {")

    current_offset = 0

    # Track nested structs: {prefix: [(field_name, field_data, local_name), ...]}
    nested_structs = OrderedDict()
    flat_fields = []

    for name, field in sorted_fields:
        c_name = field.get('cName', None)
        if c_name is None:
            # Convert camelCase to snake_case for C
            c_name = ''.join(['_' + c.lower() if c.isupper() else c for c in name]).lstrip('_')

        # Check if this is a nested struct field
        if '.' in c_name:
            prefix, local_name = c_name.split('.', 1)
            if prefix not in nested_structs:
                nested_structs[prefix] = []
            nested_structs[prefix].append((name, field, local_name))
        else:
            flat_fields.append((name, field, c_name))

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
                # Update current_offset to end of struct
                last_field = struct_fields[-1][1]
                current_offset = last_field['offset'] + last_field.get('size', 1)
        else:
            # Regular flat field
            c_type = get_c_type(field)

            # Handle array types
            if size > 1:
                lines.append(f"        {c_type} {c_name_raw}[{size}]; // {offset}")
            else:
                lines.append(f"        {c_type} {c_name_raw}; // {offset}")

            current_offset = offset + size

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

    return '\n'.join(lines)


def main():
    # Parse arguments
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    flags = [a for a in sys.argv[1:] if a.startswith('--')]

    # Determine source
    if '--url' in flags:
        source = SCHEMA_URL
    elif args:
        source = args[0]
    else:
        source = DEFAULT_SCHEMA

    schema = load_schema(source)

    header_content = generate_header(schema)
    with open(OUTPUT_FILE, 'w') as f:
        f.write(header_content)
    print(f"Generated {OUTPUT_FILE}")


if __name__ == '__main__':
    main()
