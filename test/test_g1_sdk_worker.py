import sys
import types

import pytest

from g1_cmd import g1_sdk_worker


def install_sdk_stubs(monkeypatch, client_cls, channel_calls):
    unitree_sdk2py = types.ModuleType("unitree_sdk2py")
    unitree_sdk2py.__path__ = []

    core = types.ModuleType("unitree_sdk2py.core")
    core.__path__ = []
    channel = types.ModuleType("unitree_sdk2py.core.channel")

    def fake_channel_factory_initialize(domain_id, interface_name):
        channel_calls.append((domain_id, interface_name))

    channel.ChannelFactoryInitialize = fake_channel_factory_initialize

    g1 = types.ModuleType("unitree_sdk2py.g1")
    g1.__path__ = []
    loco = types.ModuleType("unitree_sdk2py.g1.loco")
    loco.__path__ = []
    g1_loco_client = types.ModuleType("unitree_sdk2py.g1.loco.g1_loco_client")
    g1_loco_client.LocoClient = client_cls

    monkeypatch.setitem(sys.modules, "unitree_sdk2py", unitree_sdk2py)
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.core", core)
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.core.channel", channel)
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.g1", g1)
    monkeypatch.setitem(sys.modules, "unitree_sdk2py.g1.loco", loco)
    monkeypatch.setitem(
        sys.modules,
        "unitree_sdk2py.g1.loco.g1_loco_client",
        g1_loco_client,
    )


def test_waits_for_first_heartbeat_before_move(monkeypatch):
    class FakeClient:
        moves = []

        def SetTimeout(self, timeout):
            self.timeout = timeout

        def Init(self):
            return None

        def Move(self, vx, vy, vyaw, continous_move=False):
            self.__class__.moves.append((vx, vy, vyaw))

        def Squat2StandUp(self):
            raise AssertionError("stand command should not run before heartbeat")

        def StandUp2Squat(self):
            raise AssertionError("squat command should not run before heartbeat")

    channel_calls = []
    install_sdk_stubs(monkeypatch, FakeClient, channel_calls)

    monkeypatch.setattr(g1_sdk_worker.time, "perf_counter", lambda: 0.0)

    sleep_calls = []

    def stop_after_first_sleep(_seconds):
        sleep_calls.append(_seconds)
        raise SystemExit()

    monkeypatch.setattr(g1_sdk_worker.time, "sleep", stop_after_first_sleep)

    shared_arr = [0.0, 0.0, 0.0, 0.0, 0.0]

    with pytest.raises(SystemExit):
        g1_sdk_worker.sdk_process(shared_arr, "test-iface", control_hz=50)

    assert channel_calls == [(0, "test-iface")]
    assert sleep_calls, "worker should idle while waiting for the first heartbeat"
    assert FakeClient.moves == []


def test_retries_client_init_with_short_timeout(monkeypatch):
    class FakeClient:
        init_attempts = 0
        timeouts = []
        moves = []

        def SetTimeout(self, timeout):
            self.__class__.timeouts.append(timeout)

        def Init(self):
            self.__class__.init_attempts += 1
            if self.__class__.init_attempts == 1:
                raise RuntimeError("sport service not ready")

        def Move(self, vx, vy, vyaw, continous_move=False):
            self.__class__.moves.append((vx, vy, vyaw))

        def Squat2StandUp(self):
            return None

        def StandUp2Squat(self):
            return None

    channel_calls = []
    install_sdk_stubs(monkeypatch, FakeClient, channel_calls)

    monkeypatch.setattr(g1_sdk_worker.time, "perf_counter", lambda: 0.0)

    sleep_calls = []

    def sleep_with_retry_then_stop(_seconds):
        sleep_calls.append(_seconds)
        if len(sleep_calls) >= 2:
            raise SystemExit()

    monkeypatch.setattr(g1_sdk_worker.time, "sleep", sleep_with_retry_then_stop)

    shared_arr = [0.1, 0.0, 0.0, 0.0, 1.0]

    with pytest.raises(SystemExit):
        g1_sdk_worker.sdk_process(shared_arr, "test-iface", control_hz=50)

    assert channel_calls == [(0, "test-iface")]
    assert FakeClient.init_attempts == 2
    assert FakeClient.timeouts == [1.0, 1.0]
    assert FakeClient.moves == [(0.1, 0.0, 0.0)]


def test_uses_continuous_move_for_velocity_control(monkeypatch):
    class FakeClient:
        moves = []

        def SetTimeout(self, timeout):
            self.timeout = timeout

        def Init(self):
            return None

        def Move(self, vx, vy, vyaw, continous_move=False):
            self.__class__.moves.append((vx, vy, vyaw, continous_move))

        def Squat2StandUp(self):
            return None

        def StandUp2Squat(self):
            return None

    channel_calls = []
    install_sdk_stubs(monkeypatch, FakeClient, channel_calls)

    monkeypatch.setattr(g1_sdk_worker.time, "perf_counter", lambda: 0.0)

    sleep_calls = []

    def stop_after_move(_seconds):
        sleep_calls.append(_seconds)
        if FakeClient.moves:
            raise SystemExit()

    monkeypatch.setattr(g1_sdk_worker.time, "sleep", stop_after_move)

    shared_arr = [0.1, 0.0, 0.0, 0.0, 1.0]

    with pytest.raises(SystemExit):
        g1_sdk_worker.sdk_process(shared_arr, "test-iface", control_hz=50)

    assert channel_calls == [(0, "test-iface")]
    assert sleep_calls
    assert FakeClient.moves == [(0.1, 0.0, 0.0, True)]
