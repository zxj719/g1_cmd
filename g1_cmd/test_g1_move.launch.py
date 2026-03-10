#!/usr/bin/env python3
"""
g1_move 测试 launch：启动 g1_move 节点，并发布一段测试 cmd_vel 序列。

用法：
  python3 test_g1_move.launch.py

测试流程：
  1. 启动 g1_move 节点 (订阅 cmd_vel -> unitree_sdk2)
  2. 等待 3 秒让节点初始化
  3. 前进 0.2m/s 持续 3 秒
  4. 停止 1 秒
  5. 左转 0.5rad/s 持续 2 秒
  6. 停止 1 秒
  7. 后退 0.15m/s 持续 2 秒
  8. 停止，结束
"""

import time
import subprocess
import sys
import signal

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelTestPublisher(Node):
    """发布测试速度指令序列。"""

    def __init__(self):
        super().__init__('cmd_vel_test_publisher')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('cmd_vel 测试发布器就绪')

    def send(self, vx=0.0, vy=0.0, vyaw=0.0, duration=1.0, hz=10):
        """以 hz 频率持续发布 duration 秒。"""
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = vyaw

        count = int(duration * hz)
        period = 1.0 / hz
        self.get_logger().info(
            f'发布: vx={vx:.2f} vy={vy:.2f} vyaw={vyaw:.2f} '
            f'持续 {duration:.1f}s')

        for _ in range(count):
            self.pub.publish(twist)
            time.sleep(period)

    def stop(self, duration=1.0):
        """发布零速度。"""
        self.send(0.0, 0.0, 0.0, duration)


def main():
    rclpy.init()
    node = CmdVelTestPublisher()

    try:
        node.get_logger().info('等待 g1_move 节点初始化 (3s)...')
        time.sleep(3.0)

        # 测试 1: 前进
        node.get_logger().info('=== 测试 1: 前进 ===')
        node.send(vx=0.2, duration=3.0)
        node.stop(1.0)

        # 测试 2: 左转
        node.get_logger().info('=== 测试 2: 左转 ===')
        node.send(vyaw=0.5, duration=2.0)
        node.stop(1.0)

        # 测试 3: 后退
        node.get_logger().info('=== 测试 3: 后退 ===')
        node.send(vx=-0.15, duration=2.0)
        node.stop(1.0)

        # 测试 4: 侧移 (如果 G1 支持)
        node.get_logger().info('=== 测试 4: 左侧移 ===')
        node.send(vy=0.1, duration=2.0)
        node.stop(1.0)

        # 测试 5: 死区内角速度 (测试智能补偿)
        node.get_logger().info('=== 测试 5: 死区内角速度 0.2 (测试补偿) ===')
        node.send(vyaw=0.2, duration=3.0)
        node.stop(1.0)

        node.get_logger().info('=== 全部测试完成 ===')

    except KeyboardInterrupt:
        node.get_logger().info('测试被用户中断')
        node.stop(0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
