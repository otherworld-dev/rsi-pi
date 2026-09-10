"""cycles_per_step > 1 must SPREAD a delta over its cycles, not burst it.

Found on the KR 16-2 (2026-09-10): move_joint_trajectory(..., cycles_per_step=25)
sent each 0.2 deg delta in one 4 ms cycle then held for 24 - a 50 deg/s
hammer blow every 100 ms, loudly audible. The same shape silently shrinks
under the rate limiter. Each waypoint's delta now goes out as n equal parts
on n consecutive cycles: same distance, same timing, smooth.
"""
import types
from unittest.mock import MagicMock

import pytest

from RSIPI.motion_api import MotionAPI


def _client(published):
    client = MagicMock()
    client.rsi_mode = "relative"
    client.receive_variables = {"RKorr": {"X": 0.0, "Y": 0.0}}
    client._oneshot_active = types.SimpleNamespace(value=False)
    client.publish_corrections.side_effect = lambda c: published.append(dict(c["RKorr"])) or len(published)
    client.wait_correction_applied.return_value = True
    return client


def test_delta_is_spread_evenly_over_the_cycles():
    published = []
    motion = MotionAPI(_client(published))
    motion.execute_trajectory([{"X": 1.0, "Y": -0.4}], space="cartesian",
                              cycles_per_step=4, points="delta")
    assert len(published) == 4
    for p in published:
        assert p["X"] == pytest.approx(0.25) and p["Y"] == pytest.approx(-0.1)
    assert sum(p["X"] for p in published) == pytest.approx(1.0)


def test_cycles_per_step_one_sends_the_delta_once():
    published = []
    motion = MotionAPI(_client(published))
    motion.execute_trajectory([{"X": 1.0, "Y": 0.0}], space="cartesian",
                              cycles_per_step=1, points="delta")
    assert published == [{"X": 1.0, "Y": 0.0}]


def test_world_points_take_the_same_path():
    """move_cartesian_trajectory goes world -> deltas -> the same spreading."""
    published = []
    client = _client(published)
    client.send_variables = {"RIst": {"X": 100.0, "Y": 0.0, "Z": 0.0, "A": 0.0, "B": 0.0, "C": 0.0}}
    motion = MotionAPI(client)
    motion.execute_trajectory([{"X": 100.5}, {"X": 101.0}], space="cartesian",
                              cycles_per_step=5, points="world")
    assert len(published) == 10
    assert all(p["X"] == pytest.approx(0.1) for p in published)
    assert sum(p["X"] for p in published) == pytest.approx(1.0)
