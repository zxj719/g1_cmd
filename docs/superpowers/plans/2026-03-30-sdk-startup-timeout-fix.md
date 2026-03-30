# G1 SDK Startup Timeout Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent `g1_move` startup from stalling on the first `LocoClient.Move()` call before the Unitree sport service is ready.

**Architecture:** Keep the existing ROS 2 process plus SDK worker process split and shared-memory bridge. Change the SDK worker so it stays idle until it sees a ROS heartbeat, uses short RPC timeouts, and retries initialization instead of letting a single blocked startup call freeze the control loop.

**Tech Stack:** Python, pytest, ROS 2 `ament_python`, `unitree_sdk2py`

---

### Task 1: Lock In Worker Startup Behavior

**Files:**
- Create: `test/test_g1_sdk_worker.py`
- Modify: `g1_cmd/g1_sdk_worker.py`

- [ ] **Step 1: Write the failing tests**

```python
def test_waits_for_first_heartbeat_before_move():
    ...

def test_retries_client_init_with_short_timeout():
    ...
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest /home/unitree/ros2_ws/src/g1_cmd/test/test_g1_sdk_worker.py -q`
Expected: FAIL because the worker currently calls `Move()` immediately and has no retry wrapper for client initialization.

- [ ] **Step 3: Write the minimal implementation**

```python
if not has_seen_heartbeat:
    sleep_until_next_tick()
    continue

client = create_client_with_retries(...)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest /home/unitree/ros2_ws/src/g1_cmd/test/test_g1_sdk_worker.py -q`
Expected: PASS

- [ ] **Step 5: Run package-level regression checks**

Run: `pytest /home/unitree/ros2_ws/src/g1_cmd/test/test_g1_sdk_worker.py /home/unitree/ros2_ws/src/g1_cmd/test/test_pep257.py -q`
Expected: The new worker tests pass; if `ament_*` plugins are unavailable outside colcon, report that limitation instead of guessing.
