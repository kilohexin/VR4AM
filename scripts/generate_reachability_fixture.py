import json
from pathlib import Path

import numpy as np

from app.sim.kinematics import forward_pose
from app.sim.lm3_model import LM3Model


rng = np.random.default_rng(42)
model = LM3Model()
home = np.asarray(model.home_q)
rows = []
for _ in range(1000):
    q = home + rng.uniform(-0.35, 0.35, 6)
    seed = q + rng.uniform(-0.05, 0.05, 6)
    rows.append({"target": forward_pose(q, model).model_dump(mode="json"), "seed": seed.tolist()})
Path("schemas/fixtures/sim-reachability-v1.json").write_text(
    json.dumps(rows, separators=(",", ":")), encoding="utf-8"
)
