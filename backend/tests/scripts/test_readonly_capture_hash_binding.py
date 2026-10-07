"""The readonly child must pin its destination/config before any connection."""
import hashlib
import json

import pytest

from app.commissioning import readonly_state as module
from tests.scripts.test_readonly_state import Reader, setup


async def test_hash_mismatch_blocks_connection_even_with_valid_readonly_mode(setup):
    config, output, _ = setup
    async def forbidden(_):
        pytest.fail('must not connect with changed endpoint/config')
    with pytest.raises(ValueError, match='config_hash'):
        await module.collect_state(config, output, forbidden, expected_config_sha256='0' * 64)
    assert not output.exists()


async def test_matching_hash_keeps_original_allowlist_and_readonly_result(setup):
    config, output, now = setup
    reader = Reader(now)
    async def connect(_):
        return reader
    baseline = hashlib.sha256(config.read_bytes()).hexdigest()
    assert await module.collect_state(config, output, connect, expected_config_sha256=baseline) == 0
    assert set(reader.calls) == set(module.READ_METHODS)
    assert json.loads(output.read_text())['hardware_stop_verified'] is False


@pytest.mark.parametrize('invalid', [True, '', 'all', 'F' * 64, 123])
async def test_invalid_config_hash_rejected_before_connect(setup, invalid):
    config, output, _ = setup
    async def forbidden(_):
        pytest.fail('must not connect')
    with pytest.raises(ValueError, match='config_hash'):
        await module.collect_state(config, output, forbidden, expected_config_sha256=invalid)
    assert not output.exists()


def test_cli_preview_hash_mismatch_is_offline_and_does_not_collect(setup, monkeypatch):
    config, _, _ = setup
    def forbidden(*_args, **_kwargs):
        pytest.fail('preview must not collect')
    monkeypatch.setattr(module, 'collect_state', forbidden)
    with pytest.raises(ValueError, match='config_hash'):
        module.main(['--config', str(config), '--expected-config-sha256', '0' * 64])
