import math

from g1_cmd.g1_move import (
    G1MoveNode,
    LowSpeedPulseController,
    apply_command_timeout,
    apply_planar_deadband,
)


class FakeTime:
    def __init__(self, seconds):
        self.seconds = float(seconds)

    def __sub__(self, other):
        return type("FakeDuration", (), {"nanoseconds": int((self.seconds - other.seconds) * 1e9)})()


class FakeLogger:
    def warn(self, _msg):
        return None


def make_yaw_node():
    node = object.__new__(G1MoveNode)
    node.last_raw_yaw = 0.0
    node.last_yaw_time = FakeTime(0.0)
    node.continuous_small_yaw_time = 0.0
    node.last_small_yaw_sign = 0
    node.compensation_active = False
    node.accumulated_yaw_error = 0.0
    node.compensation_enabled = True
    node.compensation_duration = 0.5
    node.compensation_factor = 1.5
    node.yaw_deadband = 0.3
    node.min_yaw_command = 0.35
    node.stationary_yaw_immediate_threshold = 0.1
    node.max_vyaw = 1.0
    node.get_logger = lambda: FakeLogger()
    node.reset_compensation_state = G1MoveNode.reset_compensation_state.__get__(
        node, G1MoveNode
    )
    return node


def test_zero_velocity_stays_zero():
    assert apply_planar_deadband(0.0, 0.0, 0.15, 0.3) == (0.0, 0.0)


def test_velocity_above_deadband_is_unchanged():
    assert apply_planar_deadband(0.2, 0.0, 0.15, 0.3) == (0.2, 0.0)


def test_small_forward_velocity_is_promoted_to_minimum_command():
    assert apply_planar_deadband(0.12, 0.0, 0.15, 0.3) == (0.3, 0.0)


def test_small_diagonal_velocity_preserves_direction_when_scaled():
    vx, vy = apply_planar_deadband(0.06, 0.08, 0.15, 0.3)
    assert math.isclose(vx, 0.18, rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(vy, 0.24, rel_tol=1e-6, abs_tol=1e-6)


def test_small_velocity_uses_minimum_pulse_then_drops_to_zero():
    controller = LowSpeedPulseController(
        linear_deadband=0.15,
        min_linear_command=0.3,
        pulse_duration=0.45,
    )

    assert controller.update(0.12, 0.0, 0.0) == (0.3, 0.0)
    assert controller.update(0.12, 0.0, 0.20) == (0.3, 0.0)
    assert controller.update(0.12, 0.0, 0.46) == (0.0, 0.0)


def test_zero_velocity_resets_small_velocity_pulse():
    controller = LowSpeedPulseController(
        linear_deadband=0.15,
        min_linear_command=0.3,
        pulse_duration=0.45,
    )

    controller.update(0.12, 0.0, 0.0)
    assert controller.update(0.0, 0.0, 0.50) == (0.0, 0.0)
    assert controller.update(0.12, 0.0, 0.60) == (0.3, 0.0)


def test_large_velocity_passthrough_resets_small_velocity_pulse():
    controller = LowSpeedPulseController(
        linear_deadband=0.15,
        min_linear_command=0.3,
        pulse_duration=0.45,
    )

    controller.update(0.12, 0.0, 0.0)
    assert controller.update(0.20, 0.0, 0.10) == (0.20, 0.0)
    assert controller.update(0.12, 0.0, 0.20) == (0.3, 0.0)


def test_small_diagonal_velocity_preserves_direction_during_pulse():
    controller = LowSpeedPulseController(
        linear_deadband=0.15,
        min_linear_command=0.3,
        pulse_duration=0.45,
    )

    vx, vy = controller.update(0.06, 0.08, 0.0)
    assert math.isclose(vx, 0.18, rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(vy, 0.24, rel_tol=1e-6, abs_tol=1e-6)


def test_command_timeout_preserves_fresh_velocity():
    assert apply_command_timeout(0.2, 0.1, 0.3, command_age=0.05, timeout=0.25) == (
        0.2,
        0.1,
        0.3,
    )


def test_command_timeout_zeros_stale_velocity():
    assert apply_command_timeout(0.2, 0.1, 0.3, command_age=0.30, timeout=0.25) == (
        0.0,
        0.0,
        0.0,
    )


def test_small_stationary_yaw_is_promoted_immediately():
    node = make_yaw_node()

    assert node.smart_yaw_deadband(0.2, FakeTime(0.0), linear_motion=False) == 0.35


def test_tiny_stationary_yaw_still_waits_for_delayed_compensation():
    node = make_yaw_node()

    assert node.smart_yaw_deadband(0.05, FakeTime(0.0), linear_motion=False) == 0.0
