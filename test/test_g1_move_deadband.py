import math

from g1_cmd.g1_move import (
    LowSpeedPulseController,
    apply_command_timeout,
    apply_planar_deadband,
)


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
