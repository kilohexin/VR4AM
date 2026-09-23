"""Build a byte preserving, semantically checked control config; never connect."""
import argparse
import copy
import re
from pathlib import Path

import yaml


def build_control_bytes(original: bytes) -> bytes:
    before = yaml.safe_load(original)
    if not isinstance(before, dict) or not isinstance(before.get('real_robot'), dict):
        raise ValueError('real_robot_config_required')
    if before['real_robot'].get('mode') != 'readonly':
        raise ValueError('real_robot_readonly_required')
    text = original.decode('latin-1')
    pattern = r'(?m)^  mode: readonly(?=\r?$)'
    if len(re.findall(pattern, text)) != 1:
        raise ValueError('unique_readonly_line_required')
    candidate = re.sub(pattern, '  mode: control', text).encode('latin-1')
    expected = copy.deepcopy(before)
    expected['real_robot']['mode'] = 'control'
    if yaml.safe_load(candidate) != expected:
        raise ValueError('config_switch_changes_other_fields')
    return candidate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--original', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    with args.output.open('xb') as output:
        output.write(build_control_bytes(args.original.read_bytes()))


if __name__ == '__main__':
    main()
