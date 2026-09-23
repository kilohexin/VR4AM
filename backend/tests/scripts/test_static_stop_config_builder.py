import pytest

from scripts.build_static_stop_control_config import build_control_bytes


def test_preserves_crlf_and_all_other_bytes():
    before = b'backend: lebai\r\nreal_robot:\r\n  mode: readonly\r\n  ip: 192.0.2.1\r\n'
    after = build_control_bytes(before)
    assert after == before.replace(b'  mode: readonly', b'  mode: control')


def test_unrelated_readonly_line_cannot_mask_control_robot():
    before = (b'backend: lebai\nreal_robot:\n  mode: control\n'
              b'other:\n  mode: readonly\n')
    with pytest.raises(ValueError, match='real_robot_readonly_required'):
        build_control_bytes(before)


def test_readonly_line_in_other_section_is_rejected():
    before = (b'backend: lebai\nreal_robot: {mode: readonly}\n'
              b'other:\n  mode: readonly\n')
    with pytest.raises(ValueError, match='config_switch_changes_other_fields'):
        build_control_bytes(before)
