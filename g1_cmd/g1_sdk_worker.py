"""
SDK 子进程: 纯 unitree_sdk2py，不导入任何 ROS2 模块。
由 g1_move.py 通过 multiprocessing spawn 方式启动。
"""

import time

YELLOW = '\033[93m'
RED = '\033[91m'
GREEN = '\033[92m'
RESET = '\033[0m'

# 共享内存布局
IDX_VX = 0
IDX_VY = 1
IDX_VYAW = 2
IDX_MODE = 3
IDX_ALIVE = 4


def sdk_process(shared_arr, channel_name, control_hz=10):
    """独立进程运行 LocoClient，从共享内存读取指令。"""
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

    print(f'{YELLOW}[SDK] 初始化通道: {channel_name}{RESET}')
    ChannelFactoryInitialize(0, channel_name)

    client = LocoClient()
    client.SetTimeout(10.0)
    client.Init()
    print(f'{GREEN}[SDK] LocoClient 初始化成功{RESET}')

    period = 1.0 / control_hz
    last_alive = -1
    no_heartbeat_count = 0

    while True:
        try:
            vx = shared_arr[IDX_VX]
            vy = shared_arr[IDX_VY]
            vyaw = shared_arr[IDX_VYAW]
            mode = int(shared_arr[IDX_MODE])
            alive = int(shared_arr[IDX_ALIVE])

            # 心跳检测: 如果 ROS2 进程不再更新，发零速
            if alive == last_alive:
                no_heartbeat_count += 1
                if no_heartbeat_count > control_hz * 2:  # 2 秒无心跳
                    vx, vy, vyaw = 0.0, 0.0, 0.0
            else:
                no_heartbeat_count = 0
                last_alive = alive

            if mode == 0:  # velocity_control
                client.Move(vx, vy, vyaw)
            elif mode == 1:  # stand
                client.Squat2StandUp()
            elif mode == 2:  # squat
                client.StandUp2Squat()

            time.sleep(period)

        except KeyboardInterrupt:
            print(f'{YELLOW}[SDK] 停止{RESET}')
            client.Move(0.0, 0.0, 0.0)
            break
        except Exception as e:
            print(f'{RED}[SDK] 错误: {e}{RESET}')
            time.sleep(1.0)
