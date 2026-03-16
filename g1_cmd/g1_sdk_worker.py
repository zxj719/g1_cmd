"""Unitree SDK worker process for g1_move."""

import time

YELLOW = "\033[93m"
RED = "\033[91m"
GREEN = "\033[92m"
RESET = "\033[0m"

IDX_VX = 0
IDX_VY = 1
IDX_VYAW = 2
IDX_MODE = 3
IDX_ALIVE = 4


def sdk_process(shared_arr, channel_name, control_hz=50):
    """Run the Unitree LocoClient loop from a clean, ROS-free process."""

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

    print(f"{YELLOW}[SDK] initializing channel: {channel_name}{RESET}")
    ChannelFactoryInitialize(0, channel_name)

    client = LocoClient()
    client.SetTimeout(10.0)
    client.Init()
    print(f"{GREEN}[SDK] LocoClient initialized{RESET}")

    period = 1.0 / max(float(control_hz), 1.0)
    next_tick = time.perf_counter()
    last_alive = -1
    no_heartbeat_count = 0

    while True:
        try:
            vx = shared_arr[IDX_VX]
            vy = shared_arr[IDX_VY]
            vyaw = shared_arr[IDX_VYAW]
            mode = int(shared_arr[IDX_MODE])
            alive = int(shared_arr[IDX_ALIVE])

            if alive == last_alive:
                no_heartbeat_count += 1
                if no_heartbeat_count > control_hz * 2:
                    vx, vy, vyaw = 0.0, 0.0, 0.0
            else:
                no_heartbeat_count = 0
                last_alive = alive

            if mode == 0:
                client.Move(vx, vy, vyaw)
            elif mode == 1:
                client.Squat2StandUp()
            elif mode == 2:
                client.StandUp2Squat()

            next_tick += period
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                next_tick = time.perf_counter()

        except KeyboardInterrupt:
            print(f"{YELLOW}[SDK] stopping{RESET}")
            client.Move(0.0, 0.0, 0.0)
            break
        except Exception as exc:
            print(f"{RED}[SDK] error: {exc}{RESET}")
            next_tick = time.perf_counter()
            time.sleep(1.0)
