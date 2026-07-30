import asyncio

from app.acceptance.fake_lebai import run_fake_lebai_scenarios


def test_fake_lebai_scenarios_cover_every_required_axis_and_action() -> None:
    results = asyncio.run(run_fake_lebai_scenarios())
    by_name = {result.name: result for result in results}
    assert set(by_name) == {
        "translation",
        "rotation",
        "gripper_home_stop",
        "faults",
    }
    assert by_name["translation"].metrics["axes"] == [
        "+x",
        "-x",
        "+y",
        "-y",
        "+z",
        "-z",
    ]
    assert by_name["rotation"].metrics["axes"] == [
        "+roll",
        "-roll",
        "+pitch",
        "-pitch",
        "+yaw",
        "-yaw",
    ]
    assert all(result.passed for result in results)
