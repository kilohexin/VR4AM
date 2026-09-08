import json
import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
BACKEND = ROOT / "backend"
OUTPUT = ROOT / "schemas" / "fixtures" / "sim-reachability-v2.json"
if str(BACKEND) not in sys.path:
    sys.path.insert(0, str(BACKEND))

from app.sim.kinematics import forward_pose
from app.sim.lm3_model import MODEL_CONFIG, LM3Model, model_config_sha256


def build_fixture(
    *,
    seed: int = 42,
    sample_count: int = 1000,
) -> dict[str, object]:
    rng = np.random.default_rng(seed)
    model = LM3Model()
    home = np.asarray(model.home_q)
    rows: list[dict[str, object]] = []
    for _ in range(sample_count):
        q = home + rng.uniform(-0.35, 0.35, 6)
        nearby_seed = q + rng.uniform(-0.05, 0.05, 6)
        rows.append(
            {
                "target": forward_pose(q, model).model_dump(mode="json"),
                "seed": nearby_seed.tolist(),
            }
        )
    return {
        "version": 2,
        "generator_seed": seed,
        "sample_count": sample_count,
        "model_sha256": model_config_sha256(MODEL_CONFIG),
        "rows": rows,
    }


def main() -> int:
    OUTPUT.write_text(
        json.dumps(build_fixture(), separators=(",", ":")),
        encoding="utf-8",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
