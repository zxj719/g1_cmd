"""
ROS2 cmd_vel -> Unitree G1 LocoClient bridge.

The ROS2 node and the Unitree SDK worker run in separate processes to avoid
CycloneDDS conflicts. Commands are exchanged through shared memory.
"""

import multiprocessing as mp
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

YELLOW = "\033[93m"
RED = "\033[91m"
GREEN = "\033[92m"
BLUE = "\033[94m"
RESET = "\033[0m"

# Shared memory layout: [vx, vy, vyaw, mode, alive]
IDX_VX = 0
IDX_VY = 1
IDX_VYAW = 2
IDX_MODE = 3
IDX_ALIVE = 4
SHM_SIZE = 5

DEFAULT_CHANNEL_NAME = "enP8p1s0"
DEFAULT_CONTROL_HZ = 50.0
DEFAULT_LOG_PERIOD = 0.5


def apply_planar_deadband(vx, vy, linear_deadband, min_linear_command):
    """Scale small planar commands up to the minimum actionable magnitude."""
    planar_speed = math.hypot(vx, vy)
    if (
        planar_speed <= 0.0
        or linear_deadband <= 0.0
        or min_linear_command <= 0.0
        or planar_speed >= linear_deadband
    ):
        return vx, vy

    scale = min_linear_command / planar_speed
    return vx * scale, vy * scale


def apply_command_timeout(vx, vy, vyaw, command_age, timeout):
    """Drop stale velocity commands instead of replaying the last twist forever."""
    if timeout > 0.0 and command_age > timeout:
        return 0.0, 0.0, 0.0

    return vx, vy, vyaw


class LowSpeedPulseController:
    """Convert a tiny linear command into one bounded startup pulse."""

    def __init__(self, linear_deadband, min_linear_command, pulse_duration):
        self.linear_deadband = float(linear_deadband)
        self.min_linear_command = float(min_linear_command)
        self.pulse_duration = float(pulse_duration)
        self._pulse_end_time = None
        self._pulse_vx = 0.0
        self._pulse_vy = 0.0
        self._latched = False

    def reset(self):
        self._pulse_end_time = None
        self._pulse_vx = 0.0
        self._pulse_vy = 0.0
        self._latched = False

    def update(self, vx, vy, now_sec):
        planar_speed = math.hypot(vx, vy)
        if (
            planar_speed <= 0.0
            or self.linear_deadband <= 0.0
            or self.min_linear_command <= 0.0
            or self.pulse_duration <= 0.0
        ):
            self.reset()
            return vx, vy

        if planar_speed >= self.linear_deadband:
            self.reset()
            return vx, vy

        if self._latched and (vx * self._pulse_vx + vy * self._pulse_vy) <= 0.0:
            self.reset()

        if not self._latched:
            scale = self.min_linear_command / planar_speed
            self._pulse_vx = vx * scale
            self._pulse_vy = vy * scale
            self._pulse_end_time = float(now_sec) + self.pulse_duration
            self._latched = True
            return self._pulse_vx, self._pulse_vy

        if float(now_sec) < self._pulse_end_time:
            return self._pulse_vx, self._pulse_vy

        return 0.0, 0.0


class G1MoveNode(Node):
    """Subscribe to cmd_vel and mirror the latest command into shared memory."""

    def __init__(self, shared_arr):
        super().__init__("g1_move")
        self.shared_arr = shared_arr

        self.declare_parameter("channel_name", DEFAULT_CHANNEL_NAME)
        self.declare_parameter("compensation_enabled", True)
        self.declare_parameter("compensation_duration", 0.5)
        self.declare_parameter("compensation_factor", 1.5)
        self.declare_parameter("linear_deadband", 0.15)
        self.declare_parameter("min_linear_command", 0.3)
        self.declare_parameter("linear_pulse_enabled", True)
        self.declare_parameter("linear_pulse_duration", 0.45)
        self.declare_parameter("cmd_vel_timeout", 0.25)
        self.declare_parameter("yaw_deadband", 0.3)
        self.declare_parameter("min_yaw_command", 0.35)
        self.declare_parameter("stationary_yaw_immediate_threshold", 0.1)

        self.compensation_enabled = bool(
            self.get_parameter("compensation_enabled").value
        )
        self.compensation_duration = float(
            self.get_parameter("compensation_duration").value
        )
        self.compensation_factor = float(
            self.get_parameter("compensation_factor").value
        )

        self.control_hz = DEFAULT_CONTROL_HZ
        self.log_period = DEFAULT_LOG_PERIOD

        self.linear_deadband = float(self.get_parameter("linear_deadband").value)
        self.min_linear_command = float(
            self.get_parameter("min_linear_command").value
        )
        self.linear_pulse_enabled = bool(
            self.get_parameter("linear_pulse_enabled").value
        )
        self.linear_pulse_duration = float(
            self.get_parameter("linear_pulse_duration").value
        )
        self.cmd_vel_timeout = float(self.get_parameter("cmd_vel_timeout").value)
        self.yaw_deadband = float(self.get_parameter("yaw_deadband").value)
        self.min_yaw_command = float(self.get_parameter("min_yaw_command").value)
        self.stationary_yaw_immediate_threshold = float(
            self.get_parameter("stationary_yaw_immediate_threshold").value
        )
        self.max_vx = 0.4
        self.max_vy = 0.2
        self.max_vyaw = 1.0
        self.low_speed_pulse_controller = LowSpeedPulseController(
            self.linear_deadband,
            self.min_linear_command,
            self.linear_pulse_duration,
        )

        self.last_raw_yaw = 0.0
        self.last_yaw_time = self.get_clock().now()
        self.continuous_small_yaw_time = 0.0
        self.last_small_yaw_sign = 0
        self.compensation_active = False
        self.accumulated_yaw_error = 0.0

        self.latest_twist = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.heartbeat = 0
        self.last_motion_state = False
        self.last_log_time = self.get_clock().now()

        self.command_cb_group = MutuallyExclusiveCallbackGroup()
        self.velocity_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer_cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            Twist,
            "cmd_vel",
            self.velocity_callback,
            10,
            callback_group=self.velocity_cb_group,
        )
        self.create_subscription(
            String,
            "cmd_control_command",
            self.command_callback,
            10,
            callback_group=self.command_cb_group,
        )

        # Keep feeding the low-level controller faster than the Nav2 command rate.
        self.create_timer(
            1.0 / self.control_hz,
            self.control_loop,
            callback_group=self.timer_cb_group,
        )

        self.get_logger().info(
            f"g1_move ready: control_hz={self.control_hz:.1f}, "
            f"linear_deadband={self.linear_deadband:.2f}, "
            f"linear_pulse_enabled={self.linear_pulse_enabled}, "
            f"linear_pulse_duration={self.linear_pulse_duration:.2f}, "
            f"cmd_vel_timeout={self.cmd_vel_timeout:.2f}, "
            f"yaw_deadband={self.yaw_deadband:.2f}, "
            f"stationary_yaw_immediate_threshold="
            f"{self.stationary_yaw_immediate_threshold:.2f}"
        )

    def command_callback(self, msg):
        mode_map = {"velocity_control": 0, "stand": 1, "squat": 2}
        if msg.data in mode_map:
            self.shared_arr[IDX_MODE] = mode_map[msg.data]
            self.reset_compensation_state()
            self.low_speed_pulse_controller.reset()

    def velocity_callback(self, msg):
        self.latest_twist = msg
        self.last_cmd_time = self.get_clock().now()

    def reset_compensation_state(self):
        self.continuous_small_yaw_time = 0.0
        self.compensation_active = False
        self.accumulated_yaw_error = 0.0
        self.last_small_yaw_sign = 0

    def smart_yaw_deadband(self, raw_yaw, current_time, linear_motion):
        self.last_raw_yaw = raw_yaw

        if raw_yaw == 0.0 or abs(raw_yaw) >= self.yaw_deadband:
            self.reset_compensation_state()
            return raw_yaw

        # G1 often ignores tiny yaw commands while it is already translating.
        # Promote them to the minimum actionable turn rate immediately so the
        # robot can keep arcing instead of alternating between straight and spin.
        if linear_motion:
            self.reset_compensation_state()
            return self.min_yaw_command * (1 if raw_yaw > 0.0 else -1)

        current_sign = 1 if raw_yaw > 0 else -1

        # Navigation often finishes large heading changes with in-place yaw
        # commands around 0.1~0.2 rad/s. Treat those as deliberate turns and
        # promote them immediately instead of inserting a 0.5 s stop.
        if abs(raw_yaw) >= self.stationary_yaw_immediate_threshold:
            self.reset_compensation_state()
            return self.min_yaw_command * current_sign

        if current_sign == self.last_small_yaw_sign:
            time_diff = (current_time - self.last_yaw_time).nanoseconds / 1e9
            self.continuous_small_yaw_time += time_diff
            self.accumulated_yaw_error += raw_yaw * time_diff

            if (
                self.continuous_small_yaw_time > self.compensation_duration
                and self.compensation_enabled
                and not self.compensation_active
            ):
                self.compensation_active = True
                self.get_logger().warn(
                    f"{RED}activating yaw compensation after "
                    f"{self.continuous_small_yaw_time:.1f}s of small yaw commands{RESET}"
                )
        else:
            self.continuous_small_yaw_time = 0.0
            self.accumulated_yaw_error = 0.0

        self.last_yaw_time = current_time
        self.last_small_yaw_sign = current_sign

        if not self.compensation_active:
            return 0.0

        compensated_yaw = self.min_yaw_command * current_sign
        if abs(self.accumulated_yaw_error) > 0.2:
            compensated_yaw *= self.compensation_factor
            compensated_yaw = min(abs(compensated_yaw), self.max_vyaw * 0.5)
            compensated_yaw *= current_sign

        return compensated_yaw

    def control_loop(self):
        current_time = self.get_clock().now()
        command_age = (current_time - self.last_cmd_time).nanoseconds / 1e9

        raw_vx = max(min(self.latest_twist.linear.x, self.max_vx), -self.max_vx)
        raw_vy = max(min(self.latest_twist.linear.y, self.max_vy), -self.max_vy)
        raw_vyaw = self.latest_twist.angular.z
        raw_vx, raw_vy, raw_vyaw = apply_command_timeout(
            raw_vx,
            raw_vy,
            raw_vyaw,
            command_age,
            self.cmd_vel_timeout,
        )
        now_sec = current_time.nanoseconds / 1e9
        if self.linear_pulse_enabled:
            vx, vy = self.low_speed_pulse_controller.update(raw_vx, raw_vy, now_sec)
        else:
            vx, vy = apply_planar_deadband(
                raw_vx,
                raw_vy,
                self.linear_deadband,
                self.min_linear_command,
            )
        vyaw = raw_vyaw

        linear_motion = abs(vx) > 0.03 or abs(vy) > 0.03

        if vyaw != 0.0:
            vyaw = self.smart_yaw_deadband(vyaw, current_time, linear_motion)
        else:
            self.reset_compensation_state()

        vyaw = max(min(vyaw, self.max_vyaw), -self.max_vyaw)

        self.shared_arr[IDX_VX] = vx
        self.shared_arr[IDX_VY] = vy
        self.shared_arr[IDX_VYAW] = vyaw
        self.heartbeat += 1
        self.shared_arr[IDX_ALIVE] = self.heartbeat

        is_moving = abs(vx) > 1e-3 or abs(vy) > 1e-3 or abs(vyaw) > 1e-3
        elapsed = (current_time - self.last_log_time).nanoseconds / 1e9
        if is_moving != self.last_motion_state or (is_moving and elapsed >= self.log_period):
            self.last_motion_state = is_moving
            self.last_log_time = current_time
            if is_moving:
                color = YELLOW if abs(vyaw) >= self.yaw_deadband else BLUE
                self.get_logger().info(
                    f"{color}send vx={vx:.3f}, vy={vy:.3f}, yaw={vyaw:.3f} "
                    f"(raw={self.last_raw_yaw:.3f}){RESET}"
                )
            else:
                self.get_logger().info("send vx=0.000, vy=0.000, yaw=0.000")


def main():
    from g1_cmd.g1_sdk_worker import sdk_process

    ctx = mp.get_context("spawn")
    shared_arr = ctx.Array("d", SHM_SIZE)
    shared_arr[IDX_MODE] = 0

    channel_name = DEFAULT_CHANNEL_NAME

    old_ld = os.environ.pop("LD_LIBRARY_PATH", "")
    old_cdds = os.environ.pop("CYCLONEDDS_HOME", "")
    print(f"{YELLOW}[main] starting SDK worker with a clean DDS environment{RESET}")

    sdk_proc = ctx.Process(
        target=sdk_process,
        args=(shared_arr, channel_name, DEFAULT_CONTROL_HZ),
        daemon=True,
    )
    sdk_proc.start()

    os.environ["LD_LIBRARY_PATH"] = old_ld
    if old_cdds:
        os.environ["CYCLONEDDS_HOME"] = old_cdds

    time.sleep(2.0)

    rclpy.init()
    node = G1MoveNode(shared_arr)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        shared_arr[IDX_VX] = 0.0
        shared_arr[IDX_VY] = 0.0
        shared_arr[IDX_VYAW] = 0.0
        time.sleep(0.2)

        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        sdk_proc.terminate()
        sdk_proc.join(timeout=3)


if __name__ == "__main__":
    main()
