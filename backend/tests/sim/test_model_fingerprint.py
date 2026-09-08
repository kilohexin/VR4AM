import hashlib
import importlib.util
from pathlib import Path

import pytest

from app.sim import lm3_model


@pytest.mark.parametrize("newline", [b"\n", b"\r\n"])
def test_model_fingerprint_ignores_only_platform_line_endings(tmp_path, newline):
    canonical = b'{\n  "length": 0.3\n}\n'
    path = tmp_path / "model.json"
    path.write_bytes(canonical.replace(b"\n", newline))
    assert lm3_model.model_config_sha256(path) == hashlib.sha256(canonical).hexdigest()


def test_model_fingerprint_detects_changed_parameter(tmp_path):
    path = tmp_path / "model.json"
    path.write_bytes(b'{\n  "length": 0.3\n}\n')
    original = lm3_model.model_config_sha256(path)
    path.write_bytes(b'{\r\n  "length": 0.4\r\n}\r\n')
    assert lm3_model.model_config_sha256(path) != original


def test_fixture_generator_uses_canonical_model_fingerprint(tmp_path, monkeypatch):
    script = Path(__file__).resolve().parents[3] / "scripts/generate_reachability_fixture.py"
    spec = importlib.util.spec_from_file_location("fixture_generator", script)
    generator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(generator)
    path = tmp_path / "model.json"
    canonical = b'{\n  "length": 0.3\n}\n'
    path.write_bytes(canonical.replace(b"\n", b"\r\n"))
    monkeypatch.setattr(generator, "MODEL_CONFIG", path)
    fixture = generator.build_fixture(sample_count=0)
    assert fixture["model_sha256"] == hashlib.sha256(canonical).hexdigest()
