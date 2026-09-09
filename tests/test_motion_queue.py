"""The trajectory queue paces legs in robot cycles, like execute_trajectory.

queue_*() used to accept only rate= (seconds per waypoint) and hand it to
execute_trajectory(), which then warned that rate= is deprecated on every
leg. cycles_per_step is now first-class on the queue; rate= is resolved at
queue time and stays as a deprecated alias.
"""
import logging
from unittest.mock import MagicMock

import pytest

from RSIPI.motion_api import MotionAPI
from RSIPI.safety_manager import SafetyManager


class _StubClient:
    cycle_time = 0.004

    def __init__(self):
        self.send_variables = {}
        self.receive_variables = {"RKorr": {"X": 0.0, "Y": 0.0, "Z": 0.0},
                                  "AKorr": {"A1": 0.0}}
        self.safety_manager = SafetyManager()


P0 = {"X": 0.0, "Y": 0.0, "Z": 0.0}
P1 = {"X": 10.0, "Y": 0.0, "Z": 0.0}


@pytest.fixture
def motion():
    return MotionAPI(_StubClient())


class TestQueuePacing:
    def test_cycles_per_step_is_stored_and_reported(self, motion):
        motion.queue_cartesian_trajectory(P0, P1, steps=10, cycles_per_step=5)
        item = motion.get_queue()[0]
        assert item["space"] == "cartesian"
        assert item["steps"] == 10
        assert item["cycles_per_step"] == 5
        assert item["rate"] == pytest.approx(0.02)

    def test_rate_is_a_deprecated_alias(self, motion, caplog):
        with caplog.at_level(logging.WARNING):
            motion.queue_cartesian_trajectory(P0, P1, steps=10, rate=0.04)
        assert motion.get_queue()[0]["cycles_per_step"] == 10
        assert any("deprecated" in r.message for r in caplog.records)

    def test_defaults_match_the_old_rates(self, motion):
        # 0.012 s and 0.4 s at a 4 ms cycle.
        motion.queue_cartesian_trajectory(P0, P1, steps=5)
        motion.queue_joint_trajectory({"A1": 0.0}, {"A1": 3.0}, steps=5)
        assert [i["cycles_per_step"] for i in motion.get_queue()] == [3, 100]

    @pytest.mark.parametrize("kwargs", [{"cycles_per_step": 0}, {"rate": 0.0}, {"rate": -1}])
    def test_rejects_non_positive_pacing(self, motion, kwargs):
        with pytest.raises(ValueError):
            motion.queue_cartesian_trajectory(P0, P1, steps=5, **kwargs)
        assert motion.get_queue() == []


class TestExecuteQueued:
    def test_executor_receives_cycles_per_step_and_queue_is_cleared(self, motion):
        motion.queue_cartesian_trajectory(P0, P1, steps=4, cycles_per_step=7)
        motion.queue_joint_trajectory({"A1": 0.0}, {"A1": 1.0}, steps=3, cycles_per_step=25)
        motion.execute_trajectory = MagicMock()

        motion.execute_queued_trajectories()

        calls = motion.execute_trajectory.call_args_list
        assert [c.kwargs["cycles_per_step"] for c in calls] == [7, 25]
        assert [c.args[1] for c in calls] == ["cartesian", "joint"]
        assert motion.get_queue() == []
