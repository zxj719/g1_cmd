# 2026-04-13 G1 原地小角速度旋转卡顿排查报告

## 概要

本次会话聚焦的问题是：

- 小角度转弯时动作自然
- 但当导航进入“原地转向”阶段，尤其是总转角较大时，会出现明显卡顿，甚至持续发布速度后机器人仍短暂停住

本次最终结论：

- 根因不在 `50 Hz` 发布频率本身，而在 `g1_move` 对“原地小角速度”采用了延时补偿路径
- 修复前，纯原地 `yaw=0.20` 会先被压成 `0.0`，等待约 `0.5 s` 后才补偿到最小可行动角速度
- 修复后，纯原地且 `abs(yaw) >= 0.10` 的命令会立即提升到 `min_yaw_command`
- 实机回归后，纯原地 `yaw=0.20` 的 raw 起转延迟已从约 `0.75 s` 回落到约 `0.22 s`

## 运动判据

### 主判据

本次是否“真正开始转动”的主判据仍使用 Unitree 原始 DDS 状态：

- topic: `rt/odommodestate`
- 类型: `unitree_sdk2py.idl.unitree_go.msg.dds_.SportModeState_`
- 关键字段: `yaw_speed`

原因：

- 它离底层步态执行最近
- 不会把 SLAM 自身的观测滞后混进“机器人有没有开始动”的判断

### 辅助判据

`/lightning/odometry` 这轮仍只作为辅证，不作为最终 pass/fail 判据。

因此，针对“测试是用 sportmodestate 还是 lightning odom 读机器人运动”这个问题，本 session 的答案是：

- 主判据：`rt/odommodestate`
- 辅助判据：`/lightning/odometry`

## 根因定位

### 静态代码证据

`g1_move` 原先的角速度逻辑同时满足以下条件：

- `yaw_deadband = 0.3`
- `min_yaw_command = 0.35`
- 纯原地转时，如果 `abs(raw_yaw) < 0.3`，会先输出 `0.0`
- 只有同方向小角速度持续约 `0.5 s` 后，才会启动补偿并输出 `0.35`

而导航配置里允许输出更小的角速度：

- `min_vel_theta = 0.10`
- `min_speed_theta = 0.10`

这就导致：

- 边走边转时，`yaw=0.20` 会立刻被放大，动作自然
- 纯原地转时，`yaw=0.20` 会先被抑制，表现成明显停顿

### 修复思路

只改一个变量，不动其它补偿机制：

- 新增参数 `stationary_yaw_immediate_threshold`
- 默认值设为 `0.10`
- 当 `linear_motion == False` 且 `abs(raw_yaw) >= 0.10` 时，直接输出 `min_yaw_command`
- 更小的纯原地角速度仍保留原来的延时补偿路径

这样可以把导航常见的“原地小角速度转向”从延时分支里拿出来，同时不把所有极小角速度都无条件放大。

## 代码改动

文件：

- `g1_cmd/g1_move.py`
- `test/test_g1_move_deadband.py`

本次补充了两个关键单测：

- `test_small_stationary_yaw_is_promoted_immediately`
- `test_tiny_stationary_yaw_still_waits_for_delayed_compensation`

## 实机测试

### 测试前提

- 每轮测试前先清理 `lightning`、`g1_move` 与残留压测进程
- 启动：
  - `DISPLAY=:0 ros2 run lightning run_slam_online --config /home/unitree/hoslam_lightning_lm/config/default_livox.yaml`
  - `ros2 run g1_cmd g1_move`
- 机器人处于站立状态

### 修复前复现

此前已在实机复现出纯原地小角速度的特有停顿：

- `moving_arc_small_yaw` (`vx=0.20`, `yaw=0.20`): `RAW_MOTION 0.225 s`
- `spin_small_yaw` (`vx=0.00`, `yaw=0.20`): `RAW_MOTION 0.750 s`
- `spin_large_yaw` (`vx=0.00`, `yaw=0.40`): `RAW_MOTION 0.253 s`

并且 `g1_move` 日志会出现：

- `activating yaw compensation after 0.5s of small yaw commands`

这说明问题不是“小角速度本身起不来”，而是“小角速度在纯原地模式下被额外延后了 0.5 s”。

### 修复后回归

本轮重新编译安装 `g1_cmd` 后，在实机上用 raw DDS 重新测得：

- `spin_small_yaw` (`vx=0.00`, `yaw=0.20`): `first_motion = 0.224 s`
- `spin_large_yaw` (`vx=0.00`, `yaw=0.40`): `first_motion = 0.243 s`
- `moving_arc_small_yaw` (`vx=0.20`, `yaw=0.20`): `first_motion = 0.221 s`

修复后，三组延迟已经重新收敛到同一量级，不再存在“只有纯原地小角速度会慢半拍”的特征。

### 高 CPU 对照

为了隔离“CPU 高占用是否仍会把原地小角速度拖成几秒级起转”，另外做了一轮受控压测：

- 6 个 `python3 -c 'while True: pass'` busy-loop 进程占核
- 只复测最关键用例：`spin_small_yaw` (`vx=0.00`, `yaw=0.20`)

结果：

- `stress_spin_small_yaw`: `first_motion = 0.298 s`

本轮压测没有复现“持续发布几秒后才开始转”的极端异常。

## 日志证据

修复后，`g1_move` 对纯原地 `raw=0.20` 的处理日志为：

- `send vx=0.000, vy=0.000, yaw=0.350 (raw=0.200)`

并且本轮回归里不再出现：

- `activating yaw compensation after 0.5s of small yaw commands`

这说明修复后的代码路径确实绕开了旧的 `0.5 s` 延时补偿分支。

## 验证结果

### 单测

执行：

```bash
source /opt/ros/humble/setup.bash
source /home/unitree/ros2_ws/install/setup.bash
PYTHONPATH=/home/unitree/ros2_ws/src/g1_cmd:$PYTHONPATH \
python3 -m pytest -q \
  /home/unitree/ros2_ws/src/g1_cmd/test/test_g1_move_deadband.py \
  /home/unitree/ros2_ws/src/g1_cmd/test/test_g1_sdk_worker.py
```

结果：

- `15 passed`

### 构建

执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select g1_cmd --event-handlers console_direct+
```

结果：

- 构建成功

## 结论

这次“导航大角度转向时卡顿”的主要根因已经收敛为：

- 纯原地小角速度命令进入了 `g1_move` 的延时补偿路径

本次修复后，实机表现说明：

- 纯原地 `yaw=0.20` 已能像其它对照组一样快速起转
- 当前没有证据表明 `50 Hz` 自身是这次角速度卡顿的主因
- 当前也没有证据表明修复后在高 CPU 下又退回到“几秒后才动”

## 残余风险

- 本轮没有直接跑完整 Nav2 目标点转向流程，而是用更可控的 `cmd_vel` 原地转向用例隔离变量
- 极小纯原地角速度 `< 0.10` 仍保留延时补偿逻辑，这是有意保留的保护带
- 如果后续现场仍报告“导航目标点前的最终对正还会卡”，建议下一轮直接录制 Nav2 `cmd_vel` 与 raw `yaw_speed` 的同时间线日志，再继续排除控制器参数因素
