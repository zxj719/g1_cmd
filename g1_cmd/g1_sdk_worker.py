"""Unitree SDK worker process for g1_move."""

import time

YELLOW = "\033[93m"
RED = "\033[91m"
GREEN = "\033[92m"
RESET = "\033[0m"

DEFAULT_RPC_TIMEOUT = 1.0
DEFAULT_INIT_RETRY_DELAY = 0.5

IDX_VX = 0
IDX_VY = 1
IDX_VYAW = 2
IDX_MODE = 3
IDX_ALIVE = 4


def _sleep_until(next_tick):
    sleep_time = next_tick - time.perf_counter()
    if sleep_time > 0.0:
        time.sleep(sleep_time)
        return next_tick

    return time.perf_counter()


def _create_client(channel_name, rpc_timeout, init_retry_delay):
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

    print(f"{YELLOW}[SDK] initializing channel: {channel_name}{RESET}")
    ChannelFactoryInitialize(0, channel_name)

    while True:
        client = LocoClient()
        client.SetTimeout(rpc_timeout)

        try:
            client.Init()
            print(f"{GREEN}[SDK] LocoClient initialized{RESET}")
            return client
        except Exception as exc:
            print(f"{RED}[SDK] init error: {exc}{RESET}")
            time.sleep(init_retry_delay)


def sdk_process(
    shared_arr,
    channel_name,
    control_hz=50,
    rpc_timeout=DEFAULT_RPC_TIMEOUT,
    init_retry_delay=DEFAULT_INIT_RETRY_DELAY,
):
    """Run the Unitree LocoClient loop from a clean, ROS-free process."""
    client = _create_client(channel_name, rpc_timeout, init_retry_delay)

    period = 1.0 / max(float(control_hz), 1.0)
    next_tick = time.perf_counter()
    last_alive = 0
    no_heartbeat_count = 0
    has_seen_heartbeat = False

    while True:
        try:
            vx = shared_arr[IDX_VX]
            vy = shared_arr[IDX_VY]
            vyaw = shared_arr[IDX_VYAW]
            mode = int(shared_arr[IDX_MODE])
            alive = int(shared_arr[IDX_ALIVE])

            if not has_seen_heartbeat:
                if alive <= 0:
                    next_tick += period
                    next_tick = _sleep_until(next_tick)
                    continue

                has_seen_heartbeat = True
                no_heartbeat_count = 0
                last_alive = alive
            elif alive == last_alive:
                no_heartbeat_count += 1
                if no_heartbeat_count > control_hz * 2:
                    vx, vy, vyaw = 0.0, 0.0, 0.0
            else:
                no_heartbeat_count = 0
                last_alive = alive

            if mode == 0:
                is_moving = any(abs(value) > 1e-6 for value in (vx, vy, vyaw))
                client.Move(vx, vy, vyaw, continous_move=is_moving)
            elif mode == 1:
                client.Squat2StandUp()
            elif mode == 2:
                client.StandUp2Squat()

            next_tick += period
            next_tick = _sleep_until(next_tick)

        except KeyboardInterrupt:
            print(f"{YELLOW}[SDK] stopping{RESET}")
            client.Move(0.0, 0.0, 0.0)
            break
        except Exception as exc:
            print(f"{RED}[SDK] error: {exc}{RESET}")
            next_tick = time.perf_counter()
            time.sleep(1.0)
