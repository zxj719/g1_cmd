# 2026-04-10 G1 低层通信延迟排查报告

## 概要

本次会话围绕 `g1_move` 与 `lightning run_slam_online` 并行运行时，机器人“终端持续打印速度，但机器人不动或明显延迟后才动”的现象展开排查。

本次最终结论分成两部分：

- 低速不动的主要根因是线速度死区与起步延迟叠加，不是单纯 `Start()` 缺失。
- “停止后继续沿用 last 速度”会放大现场风险，因此已新增 `cmd_vel` 超时清零机制，不再无限沿用旧速度。

## 测量口径

### 主判据

本次最终用于判断“机器人是否真的开始运动”的主判据是 Unitree 原始 DDS 状态：

- DDS topic: `rt/odommodestate`
- 消息类型: `unitree_sdk2py.idl.unitree_go.msg.dds_.SportModeState_`

原因：

- 它直接反映机器人底层运动状态，延迟和解释链最短。
- `/lightning/odometry` 约为 `10 Hz`，存在 SLAM 自身滞后。
- 在高 CPU 占用场景下，`/lightning/odometry` 出现过“先报微小速度/位移，但 raw 状态仍未确认起步”的情况，因此不适合作为最终 pass/fail 判据。

### 辅助判据

`/lightning/odometry` 仅作为辅证，用于观察：

- SLAM 链路是否正常工作
- 上层观测与底层 raw 状态是否一致
- 是否存在明显的额外感知滞后

### 说明

这里不是使用 ROS 话题 `/sportmodestate` 作为主判据。实测真正稳定有数据的是 Unitree DDS `rt/odommodestate`。`/lightning/odometry` 只用于交叉验证。

## 测试环境

- 机器人状态：站立，前方约 `0.5 m` 可通行区域
- ROS 2：Humble
- `g1_move` 包路径：`/home/unitree/ros2_ws/src/g1_cmd`
- Lightning 启动命令实际可用配置：
  - `DISPLAY=:0 ros2 run lightning run_slam_online --config /home/unitree/hoslam_lightning_lm/config/default_livox.yaml`
- 说明：用户提供的 `default__livox.yaml` 在本机不存在，本次统一使用 `default_livox.yaml`

## 排查过程

### 1. 基线确认

- `run_slam_online` 需要 `DISPLAY=:0` 才能在该环境稳定启动。
- 默认 RMW 为 `rmw_fastrtps_cpp`，当前现象不能直接归因为“ROS 2 与 Unitree SDK 共用 CycloneDDS”。
- `lightning` 正常时 `/lightning/odometry` 存在，频率约 `10 Hz`。

### 2. 低速死区确认

基于 raw 状态对照，确认低速死区存在：

- `0.08 m/s`：不动
- `0.12 m/s`：不动
- `0.20 m/s`：可动
- `0.30 m/s`：可动

### 3. 起步延迟确认

在 raw 状态和 Lightning odom 双观察下，多次测到首次起步约为 `0.5 s` 量级。这个延迟在 raw 状态里也存在，因此不是单纯 Lightning 侧滞后。

### 4. `Start()` 因素排除

做过“手动 `LocoClient.Start()` 后再测 `0.12 m/s`”对照，结果仍然 `RAW_MOTION none`。因此本次没有证据表明 `Start()` 是当前低速不起步问题的主因。

### 5. 小速度整段抬高的问题

最初把小速度直接整段拉到 `0.3 m/s`，虽然能让机器人更容易起步，但会导致：

- 低速命令持续期间始终维持较大速度
- 用户感知为“晚动但一旦动就走多了”

这与现场需求“先动起来再停止，不要傻等 1s”不完全一致。

### 6. 起步脉冲实验

围绕“同样是很小的平均期望速度，是否可以通过短促起步脉冲更早触发步态”做了对照：

- `0.30 m/s x 0.25 s`：起不来
- `0.30 m/s x 0.45 s`：可以起步

因此最终采用“低速命令先打一段有限时长的起步脉冲，然后归零等待下次触发”的策略，而不是整段时间抬速。

### 7. 高 CPU 占用对照

用户补充了“CPU 高占用时，速度持续发布，但可能几秒后机器人才开始动”的线索。本次做了受控 CPU 压力对照，压测方式为 4 个 Python busy-loop 进程同时占核。

对照结果：

- `50 Hz` 控制循环，`0.20 m/s`，raw-only 判据：`RAW_MOTION 0.502 s`
- `20 Hz` 控制循环，`0.20 m/s`，raw-only 判据：`RAW_MOTION 0.522 s`

结论：

- 本次没有复现“几秒后才动”的极端长延迟。
- 在本次受控实验里，`50 Hz` 与 `20 Hz` 的起步时延接近，没有证据表明 `50 Hz` 本身就是主因。
- 因此目前不能把“取消停止延迟会不安全”继续归因到 `50 Hz` 过快。

## 代码修改

### 1. 低速起步脉冲

文件：`g1_cmd/g1_move.py`

- 新增 `LowSpeedPulseController`
- 当线速度低于死区但非零时：
  - 先按方向放大到最小可行动速度
  - 仅持续 `linear_pulse_duration`
  - 超时后归零
- 当前默认参数：
  - `linear_deadband = 0.15`
  - `min_linear_command = 0.3`
  - `linear_pulse_enabled = True`
  - `linear_pulse_duration = 0.45`

### 2. stale `cmd_vel` 自动清零

文件：`g1_cmd/g1_move.py`

- 新增 `apply_command_timeout()`
- 新增参数 `cmd_vel_timeout`
- 当前默认值：`0.25 s`
- 当 `cmd_vel` 长时间未更新时：
  - 不再继续复用旧速度
  - 直接输出 `0, 0, 0`

这一步的目的，是让现场可以去掉“无限沿用 last 速度”的兜底方式，而不增加失控风险。

### 3. 单测补充

文件：`test/test_g1_move_deadband.py`

新增覆盖：

- 小速度起步脉冲的持续与结束
- 零速度对脉冲状态的复位
- 大速度穿透时对脉冲状态的复位
- 对角线方向放大后方向保持
- stale `cmd_vel` 超时自动清零

## 实机结果

### 低速修复后

场景：`0.12 m/s` 连续发布

结果：

- `RAW_MOTION 0.649 s`
- 最终 raw 位移约 `0.01 m`
- `g1_move` 日志表现为：
  - 先发送一次 `vx=0.300`
  - 随后很快归零

说明：

- 机器人能被触发起步
- 不再像“整段抬速”那样一旦动起来就走过多

### stale command 回归

场景：只发送一次 `cmd_vel linear.x=0.2`

结果：

- `g1_move` 先打印一次 `send vx=0.200`
- 大约 `0.24 s` 后自动打印 `send vx=0.000`

说明：

- `cmd_vel_timeout=0.25 s` 已实际生效
- 已不再需要通过“长期沿用 last 速度”来保证底层最终停不下来

## 验证结果

### 单测

执行：

```bash
PYTHONPATH=/home/unitree/ros2_ws/src/g1_cmd:$PYTHONPATH \
python3 -m pytest -q \
  /home/unitree/ros2_ws/src/g1_cmd/test/test_g1_move_deadband.py \
  /home/unitree/ros2_ws/src/g1_cmd/test/test_g1_sdk_worker.py
```

结果：

- `13 passed`

### 构建

执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select g1_cmd --event-handlers console_direct+
```

结果：

- 构建成功

## 版本管理

- 分支：`fix/g1-sdk-worker-latency`
- 本次提交：`0f460f7 Improve low-speed start and stale cmd handling`

## 当前残余问题

### 1. 极端高 CPU 下“几秒后才动”未完全复现

本次受控压测没有重现用户描述的“几秒级”启动延迟，因此这部分还不能宣称根治。下一步建议在 SDK worker 增加更底层的时序日志：

- `Move()` 调用开始/结束时间
- 控制循环 overrun 次数
- 共享内存 heartbeat 与 SDK 实际发送时间差

这样才能把问题继续向“调度问题 / SDK 排队问题 / 机器人步态机状态问题”收敛。

### 2. `Ctrl-C` 退出仍有 `rcl_shutdown already called`

当前退出路径仍会抛：

- `rclpy._rclpy_pybind11.RCLError: failed to shutdown: rcl_shutdown already called`

这不影响本次结论，但属于待清理问题。

## 建议的下一步

1. 保留当前“低速起步脉冲 + stale command 超时清零”策略继续实机观察。
2. 如果现场仍出现“高 CPU 时几秒才动”，优先在 SDK worker 加时序日志，不要先凭感觉继续改频率。
3. 如果后续能稳定复现，再针对复现场景单独做 `50 Hz / 20 Hz / 10 Hz` 和 CPU 绑核对照，进一步确认是否存在调度饥饿。
