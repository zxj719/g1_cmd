# 2026-04-11 G1 Cmd 最近几次 Session 汇总

## 时间范围

- 汇总窗口：2026-03-30 至 2026-04-10
- 关注范围：`g1_move` 启动、低速起步、命令陈旧复用与低层时延判断口径

## 总结

`g1_cmd` 最近几次 session 的结论已经比较清楚：

1. 现场“速度在打、机器人不动或明显延迟后才动”的主因，不是单纯 `Start()` 调用缺失
2. 真正更稳定的根因组合是：
   - 低速线速度落在步态死区
   - 起步存在约 `0.5 s` 量级延迟
   - 旧 `cmd_vel` 若长期沿用，会放大现场风险
3. 因此修复方向最终落在：
   - 低速起步脉冲
   - stale `cmd_vel` 超时清零
   - 用 raw DDS 运动状态而不是只看 SLAM odom 来判定“是否真的起步”

## Session 1：SDK Startup Timeout 假设作为背景

日期主线：

- 2026-03-30

当时的主问题是：

- `g1_move` 首次 `LocoClient.Move()` 可能卡住，导致启动阶段异常迟滞

这轮工作留下了一份实现计划，核心思路是：

- SDK worker 在收到 ROS heartbeat 前保持 idle
- 初始化使用短超时与重试，而不是一次阻塞卡死

这条线后来变成背景假设，而不是 4 月 10 日现场问题的主结论。

参考文档：

- [2026-03-30-sdk-startup-timeout-fix.md](/home/unitree/ros2_ws/src/g1_cmd/docs/superpowers/plans/2026-03-30-sdk-startup-timeout-fix.md)

## Session 2：低速不起步与 stale command 风险定位

日期主线：

- 2026-04-10

本轮先把“机器人是否真正开始运动”的判据重新定义为 raw DDS 状态：

- 主判据：`rt/odommodestate`
- 辅助判据：`/lightning/odometry`

原因是：

- raw DDS 直接反映底层运动状态，解释链最短
- `/lightning/odometry` 更新频率和上层 SLAM 链路都会引入额外滞后

随后得到的结论是：

- `0.08 m/s`：不起步
- `0.12 m/s`：不起步
- `0.20 m/s`：可起步
- `0.30 m/s`：可起步
- 起步延迟大致在 `0.5 s` 量级

因此，“低速不动”更像是死区和起步延迟叠加，而不是简单缺一个 `Start()`。

## Session 3：最终实现收敛

最终落到代码里的修复有两项：

### 1. 低速起步脉冲

思路：

- 当命令线速度非零但低于死区时，不把整个命令持续抬高
- 只打一段有限时长的起步脉冲
- 过了脉冲窗口就回零，等待下一次触发

这样能兼顾两件事：

- 让机器人更早进入有效步态
- 避免“整段抬速导致一旦动起来就走过多”

### 2. stale `cmd_vel` 自动清零

思路：

- 给 `g1_move` 增加 `cmd_vel_timeout`
- 长时间没有收到新命令时，不再无限沿用 last command
- 自动回到 `0, 0, 0`

这条改动是现场安全性上的关键补丁，因为它直接减少了“上层停了，但底层还沿用旧速度”的风险。

相关提交：

- `0f460f7 Improve low-speed start and stale cmd handling`

详细报告：

- [2026-04-10-g1-low-level-latency-debug.md](/home/unitree/ros2_ws/src/g1_cmd/docs/report/2026-04-10-g1-low-level-latency-debug.md)

## 当前验证结论

根据 2026-04-10 那轮记录：

- `test_g1_move_deadband.py` 和 `test_g1_sdk_worker.py` 共 `13 passed`
- `colcon build --packages-select g1_cmd` 构建通过
- 真机观察到：
  - 小速度命令可以通过起步脉冲被触发
  - 单次 `cmd_vel` 发布后，约 `0.25 s` 左右自动清零

## 还没有完全闭环的问题

剩余问题主要集中在“极端高 CPU 占用时的几秒级长延迟”：

- 本轮受控压测没有稳定复现“几秒后才动”的极端现象
- 因此目前还不能宣称这个问题已经根治
- 下一步仍建议在 SDK worker 内增加更底层的时序日志，直接看：
  - ROS 收到命令时间
  - worker 消费共享内存时间
  - DDS `Move` 发送时间
  - raw 状态首次确认起步时间

## 建议的下一步

1. 若现场继续出现“几秒后才动”，优先补 SDK worker 时序日志，而不是继续盲调频率
2. 继续沿 raw DDS 状态作为主判据，不要只盯 `/lightning/odometry`
3. 若要继续压低低速 deadband 影响，优先调脉冲参数，不建议回退到“整段抬速”

## 相关文档索引

- [2026-03-30-sdk-startup-timeout-fix.md](/home/unitree/ros2_ws/src/g1_cmd/docs/superpowers/plans/2026-03-30-sdk-startup-timeout-fix.md)
- [2026-04-10-g1-low-level-latency-debug.md](/home/unitree/ros2_ws/src/g1_cmd/docs/report/2026-04-10-g1-low-level-latency-debug.md)
