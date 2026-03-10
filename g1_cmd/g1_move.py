"""
g1_move — ROS2 cmd_vel -> Unitree G1 LocoClient 速度桥接

由于 unitree_sdk2py 和 ROS2 都使用 CycloneDDS，无法在同一进程中共存。
因此拆分为两个进程:
  - SDK 进程: 运行 LocoClient，从共享内存读取速度指令并发送 (g1_sdk_worker.py)
  - ROS2 进程: 订阅 cmd_vel，将速度写入共享内存 (本文件)
两个进程通过 multiprocessing 共享数组通信，零拷贝、无网络开销。
SDK 子进程使用 spawn 方式启动，并清除 LD_LIBRARY_PATH 以避免
ROS2 安装的 CycloneDDS 库与 unitree_sdk2py 内置的 CycloneDDS 冲突。
"""

import os
import time
import multiprocessing as mp

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from std_msgs.msg import String

YELLOW = '\033[93m'
RED = '\033[91m'
GREEN = '\033[92m'
BLUE = '\033[94m'
RESET = '\033[0m'

# 共享内存布局: [vx, vy, vyaw, mode, alive]
# mode: 0=velocity_control, 1=stand, 2=squat
# alive: ROS2 进程心跳 (递增计数)
IDX_VX = 0
IDX_VY = 1
IDX_VYAW = 2
IDX_MODE = 3
IDX_ALIVE = 4
SHM_SIZE = 5


# ===================================================================
# ROS2 节点: 订阅 cmd_vel，写入共享内存
# ===================================================================
class G1MoveNode(Node):
    """ROS2 节点，订阅 cmd_vel 并通过共享内存传递给 SDK 进程。"""

    def __init__(self, shared_arr):
        super().__init__('g1_move')
        self.shared_arr = shared_arr

        # -------- 参数 --------
        self.declare_parameter('channel_name', 'enP8p1s0')
        self.declare_parameter('compensation_enabled', True)
        self.declare_parameter('compensation_duration', 0.5)
        self.declare_parameter('compensation_factor', 1.5)

        self.compensation_enabled = self.get_parameter('compensation_enabled').value
        self.compensation_duration = self.get_parameter('compensation_duration').value
        self.compensation_factor = self.get_parameter('compensation_factor').value

        # -------- 控制参数 --------
        self.yaw_deadband = 0.3
        self.min_yaw_command = 0.31
        self.max_vx = 0.4
        self.max_vy = 0.2
        self.max_vyaw = 1.0

        # -------- 死区补偿状态 --------
        self.last_raw_yaw = 0.0
        self.last_yaw_time = self.get_clock().now()
        self.continuous_small_yaw_time = 0.0
        self.last_small_yaw_sign = 0
        self.compensation_active = False
        self.accumulated_yaw_error = 0.0

        # -------- 回调组 --------
        self.command_cb_group = MutuallyExclusiveCallbackGroup()
        self.velocity_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer_cb_group = ReentrantCallbackGroup()

        # -------- 状态 --------
        self.latest_twist = Twist()
        self.heartbeat = 0

        # -------- 订阅 --------
        self.create_subscription(
            Twist, 'cmd_vel', self.velocity_callback, 10,
            callback_group=self.velocity_cb_group)

        self.create_subscription(
            String, 'cmd_control_command', self.command_callback, 10,
            callback_group=self.command_cb_group)

        # -------- 定时器 10Hz --------
        self.create_timer(0.1, self.control_loop,
                          callback_group=self.timer_cb_group)

        self.get_logger().info(f'g1_move ROS2 节点初始化完成')
        self.get_logger().info(
            f'{GREEN}角速度死区: {self.yaw_deadband}, '
            f'最小输出: {self.min_yaw_command}{RESET}')

    # ================= 回调 =================

    def command_callback(self, msg):
        mode_map = {'velocity_control': 0, 'stand': 1, 'squat': 2}
        if msg.data in mode_map:
            self.shared_arr[IDX_MODE] = mode_map[msg.data]
            self.reset_compensation_state()

    def velocity_callback(self, msg):
        self.latest_twist = msg

    def reset_compensation_state(self):
        self.continuous_small_yaw_time = 0.0
        self.compensation_active = False
        self.accumulated_yaw_error = 0.0
        self.last_small_yaw_sign = 0

    # ================= 智能死区处理 =================

    def smart_yaw_deadband(self, raw_yaw, current_time):
        self.last_raw_yaw = raw_yaw

        if raw_yaw == 0.0 or abs(raw_yaw) >= self.yaw_deadband:
            self.reset_compensation_state()
            return raw_yaw

        current_sign = 1 if raw_yaw > 0 else -1

        if current_sign == self.last_small_yaw_sign:
            time_diff = (current_time - self.last_yaw_time).nanoseconds / 1e9
            self.continuous_small_yaw_time += time_diff
            self.accumulated_yaw_error += raw_yaw * time_diff

            if (self.continuous_small_yaw_time > self.compensation_duration
                    and self.compensation_enabled
                    and not self.compensation_active):
                self.compensation_active = True
                self.get_logger().warn(
                    f'{RED}激活角速度补偿! '
                    f'持续{self.continuous_small_yaw_time:.1f}s'
                    f'收到小角速度{RESET}')
        else:
            self.continuous_small_yaw_time = 0.0
            self.accumulated_yaw_error = 0.0

        self.last_yaw_time = current_time
        self.last_small_yaw_sign = current_sign

        if self.compensation_active:
            compensated_yaw = self.min_yaw_command * current_sign
            if abs(self.accumulated_yaw_error) > 0.2:
                compensated_yaw *= self.compensation_factor
                compensated_yaw = min(abs(compensated_yaw),
                                      self.max_vyaw * 0.5) * current_sign
            self.get_logger().info(
                f'{BLUE}补偿: {raw_yaw:.3f} -> {compensated_yaw:.3f} '
                f'(累积误差: {self.accumulated_yaw_error:.3f}){RESET}')
            return compensated_yaw

        return 0.0

    # ================= 控制循环 =================

    def control_loop(self):
        current_time = self.get_clock().now()

        vx = self.latest_twist.linear.x
        vy = self.latest_twist.linear.y
        vyaw = self.latest_twist.angular.z

        # 限幅
        vx = max(min(vx, self.max_vx), -self.max_vx)
        vy = max(min(vy, self.max_vy), -self.max_vy)

        # 智能死区处理
        if vyaw != 0:
            vyaw = self.smart_yaw_deadband(vyaw, current_time)
        else:
            self.reset_compensation_state()

        vyaw = max(min(vyaw, self.max_vyaw), -self.max_vyaw)

        # 写入共享内存
        self.shared_arr[IDX_VX] = vx
        self.shared_arr[IDX_VY] = vy
        self.shared_arr[IDX_VYAW] = vyaw
        self.heartbeat += 1
        self.shared_arr[IDX_ALIVE] = self.heartbeat

        # 日志
        if vyaw != 0:
            color = YELLOW if abs(vyaw) >= self.yaw_deadband else BLUE
            self.get_logger().info(
                f'{color}发送: vx={vx:.3f}, vy={vy:.3f}, '
                f'yaw={vyaw:.3f} (原始: {self.last_raw_yaw:.3f}){RESET}')
        elif vx != 0 or vy != 0:
            self.get_logger().info(
                f'{GREEN}发送: vx={vx:.3f}, vy={vy:.3f}, '
                f'yaw={vyaw:.3f}{RESET}')


# ===================================================================
# main
# ===================================================================
def main():
    # 延迟导入 SDK worker (独立模块，不依赖 ROS2)
    from g1_cmd.g1_sdk_worker import sdk_process

    # 使用 spawn 启动方式，确保子进程获得干净的环境
    ctx = mp.get_context('spawn')

    # 共享内存数组 (双精度浮点)
    shared_arr = ctx.Array('d', SHM_SIZE)
    shared_arr[IDX_MODE] = 0  # velocity_control

    channel_name = "enP8p1s0"

    # 清除 LD_LIBRARY_PATH 和 CYCLONEDDS_HOME，避免 ROS2/系统 CycloneDDS 与 SDK 内置的冲突
    # spawn 子进程会继承此刻的环境变量
    old_ld = os.environ.pop('LD_LIBRARY_PATH', '')
    old_cdds = os.environ.pop('CYCLONEDDS_HOME', '')
    print(f'{YELLOW}[main] 已清除 LD_LIBRARY_PATH 和 CYCLONEDDS_HOME，启动 SDK 子进程...{RESET}')

    # 启动 SDK 进程 (干净环境，无 ROS2 CycloneDDS 冲突)
    sdk_proc = ctx.Process(
        target=sdk_process,
        args=(shared_arr, channel_name),
        daemon=True)
    sdk_proc.start()

    # 恢复环境变量，ROS2 需要
    os.environ['LD_LIBRARY_PATH'] = old_ld
    if old_cdds:
        os.environ['CYCLONEDDS_HOME'] = old_cdds

    # 等待 SDK 初始化
    time.sleep(2.0)

    # ROS2 进程
    rclpy.init()
    node = G1MoveNode(shared_arr)

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 发零速
        shared_arr[IDX_VX] = 0.0
        shared_arr[IDX_VY] = 0.0
        shared_arr[IDX_VYAW] = 0.0
        time.sleep(0.2)

        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        sdk_proc.terminate()
        sdk_proc.join(timeout=3)


if __name__ == '__main__':
    main()
