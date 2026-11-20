# HeroLauncher

英雄机器人发射机构模块：控制三个摩擦轮和一个拨弹盘，按裁判系统热量限制发射 / Hero robot launcher Module controlling three friction wheels and a trigger disc, with firing limited by the referee system heat

## 1. 模块作用 / Purpose

构造时，HeroLauncher 创建线程 `HeroLauncherThread`（栈深 `param.task_stack_depth`，优先级 `param.thread_priority`）。线程每轮先休眠 2 ms，再依次刷新电机反馈、更新摩擦轮目标并执行软启动、热量计算和拨弹状态机，最后计算 PID 输出并下发。摩擦轮以 `MODE_CURRENT` 下发；拨弹电机为 DM4310-2EC（`DMMotor`），由角度环与速度环串联，输出为输出轴力矩，以 `MODE_TORQUE` 下发。拨弹电机反馈状态不是“已使能”时（未使能、报错或尚未收到反馈），禁止拨弹，摩擦轮照常运行；失能时发送使能帧，报错时发送清错帧。任意摩擦轮 `Update()` 返回错误时，复位发射状态并放松全部电机。定时任务每 47 ms 绘制一次裁判系统 UI。

摩擦轮模式：`GetEvent()` 返回的 `LibXR::Event` 注册了 `HeroLauncher::LauncherEvent` 的 `SET_FRICMODE_RELAX`、`SET_FRICMODE_SAFE`、`SET_FRICMODE_READY`。`READY` 时摩擦轮 0、1 的目标转速为 `fric_setpoint_speed_0`，摩擦轮 2 的目标转速为 `fric_setpoint_speed_1`，单位 rpm；`RELAX` 时发送零电流并复位；`SAFE` 时目标以 8000 rpm/s 减速至零，取消供弹并保持拨弹当前位置。CMD 的 `CMD_EVENT_START_CTRL` 切换到 `SET_FRICMODE_RELAX`；`CMD_EVENT_LOST_CTRL` 复位全部发射状态并切换到 `SET_FRICMODE_RELAX`。

软启动：三路输出限幅从 0.1 起每周期增加 0.01，最大为 1.0；三轮均达到目标的 90% 并连续保持 50 ms 后允许供弹。

拨弹触发（仅摩擦轮模式为 `READY` 时）：`launcher_cmd` 的 `isfire` 上升沿进入 `SINGLE`，持续超过 200 ms 进入 `CONTINUE`，`isfire` 为假时回到 `SAFE`。

拨弹盘角度：拨弹电机反馈状态变为“已使能”后的第一个周期，拨弹盘角度直接取输出轴绝对角（`abs_angle` / `trig_gear_ratio`），设定角置为当前角；之后每周期累加 `abs_angle` 的增量（按最短路径处理 ±2π 翻转）。`Reset()`、失控或电机失能后，下次使能时重新由绝对角初始化。待发位为 `trig_ready_angle` + k × 2π / `num_trig_tooth`（k 为整数），不需要首发标定。前进方向为角度减小。

常规发弹：`SINGLE`、热量允许、摩擦轮就绪且上一发已结束时推进一次，并立即回到 `SAFE`，每次上升沿只发一发。目标角按以下规则计算：

- 设定角已在待发位上（上一发由本规则得到）：目标为设定角再前进一格。
- 设定角不在待发位上（刚使能，或刚切到 `SAFE`）：若当前角与某待发位的距离不超过 `trig_ready_tolerance`（无论尚未到位还是略微越过），目标为该待发位再前进一格；否则目标为前进方向上的下一个待发位。弹链在该格缺弹时，这一发为空拨。

以摩擦轮 2 的 |扭矩| > 0.05 判定出弹并计入热量；100 ms 内未判定出弹时结束这一发，不计热量，可再次发射。

热量：每判定一发出弹，热量加 100，并按裁判系统的 `shooter_cooling_value`（每秒冷却值）随时间冷却，每周期减去该值乘以周期时长；可发射数为 ⌊(`shooter_heat_limit` − 热量) / 100⌋，为 0 时拨弹盘不再推进。

UI：定时任务在图层 2 上轮流绘制三个摩擦轮状态圆（前两路超过 3500 rpm、第三路超过 2200 rpm 为青色，否则为橙色）、拨弹盘位置圆弧（拨弹盘角度尚未由绝对编码器初始化时为橙色，否则为青色）和两条瞄准线。

Upon construction, HeroLauncher creates the thread `HeroLauncherThread` (stack depth `param.task_stack_depth`, priority `param.thread_priority`). Each iteration sleeps for 2 ms first, then refreshes the motor feedback, updates wheel targets, runs soft start, heat calculation and the trigger state machine, and finally computes the PID outputs and sends them. The friction wheels are sent in `MODE_CURRENT`. The trigger motor is a DM4310-2EC (`DMMotor`) driven by an angle loop plus a speed loop whose output is output-shaft torque, sent in `MODE_TORQUE`. While the trigger motor feedback state is not "enabled" (disabled, faulted or no feedback received yet), feeding is blocked and the friction wheels keep running; a disabled motor gets an enable frame and a faulted one a clear-error frame. When any friction wheel `Update()` returns an error, the launch state is reset and all motors are relaxed. A timer task draws the referee system UI every 47 ms.

Friction wheel modes: `GetEvent()` returns a `LibXR::Event` on which `SET_FRICMODE_RELAX`, `SET_FRICMODE_SAFE` and `SET_FRICMODE_READY` of `HeroLauncher::LauncherEvent` are registered. In `READY`, the target speed of friction wheels 0 and 1 is `fric_setpoint_speed_0` and that of friction wheel 2 is `fric_setpoint_speed_1`, in rpm; `RELAX` sends zero current and resets launch state; `SAFE` ramps targets to zero at 8000 rpm/s, cancels loading and holds the feeder position. The CMD event `CMD_EVENT_START_CTRL` switches to `SET_FRICMODE_RELAX`; `CMD_EVENT_LOST_CTRL` resets all launch states and switches to `SET_FRICMODE_RELAX`.

Soft start: the three output limits ramp from 0.1 to 1.0 by 0.01 per cycle; all three wheels must reach 90% of their targets continuously for 50 ms before loading.

Trigger activation (only in `READY`): a rising edge of `isfire` in `launcher_cmd` enters `SINGLE`, holding it for more than 200 ms enters `CONTINUE`, and `isfire` being false returns to `SAFE`.

Trigger disc angle: in the first cycle after the trigger motor feedback state becomes "enabled", the disc angle is taken directly from the output-shaft absolute angle (`abs_angle` / `trig_gear_ratio`) and the setpoint is set to it; after that the increments of `abs_angle` are accumulated every cycle (the ±2π wrap is handled along the shortest path). After `Reset()`, lost control or a disabled motor, the angle is initialized again from the absolute angle on the next enable. The ready positions are `trig_ready_angle` + k × 2π / `num_trig_tooth` (k an integer), so no first-shot calibration is needed. The forward direction is decreasing angle.

Normal firing: one advance happens in `SINGLE` when heat allows, the friction wheels are ready and the previous shot has finished; the mode then returns to `SAFE` at once, so each rising edge fires one round. The target angle is:

- Setpoint on a ready position (the previous shot was set by this rule): the setpoint advances one more tooth.
- Setpoint not on a ready position (just enabled, or just switched to `SAFE`): if the current angle is within `trig_ready_tolerance` of a ready position (short of it or slightly past it), the target is that ready position advanced one tooth; otherwise it is the next ready position in the forward direction. If that slot of the belt is empty, this shot feeds nothing.

A |torque| of friction wheel 2 above 0.05 is taken as a round leaving and is counted into the heat; when no round is detected within 100 ms the shot ends without heat and another shot may start.

Heat: each detected round adds 100 to the heat, which cools over time at the referee system `shooter_cooling_value` (cooling per second), subtracting that value multiplied by the cycle time every cycle; the number of available shots is ⌊(`shooter_heat_limit` − heat) / 100⌋, and the trigger disc stops advancing when it is 0.

UI: the timer task draws in turn, on layer 2, the three friction wheel status circles (cyan when the front two exceed 3500 rpm and the back two exceed 2200 rpm, orange otherwise), the trigger disc position arc (orange until the disc angle has been initialized from the absolute encoder, cyan after) and two aiming lines.

## 2. 构造接口 / Constructor

```cpp
HeroLauncher(CMD& cmd,
             RMMotor& fric_motor_0,
             RMMotor& fric_motor_1,
             RMMotor& fric_motor_2,
             DMMotor& motor_trig,
             Referee& ref,
             const Param& param = {...});  // 节选 / excerpt
```

依赖：

- `cmd`：`CMD` 实例。
- `fric_motor_0` 至 `fric_motor_2`：`RMMotor`，三个摩擦轮电机；0、1 使用第一组目标，2 使用第二组目标，摩擦轮 2 同时用于出弹检测。
- `motor_trig`：`DMMotor`，拨弹电机，DM4310-2EC（输出轴带绝对编码器），拨弹盘直接装在输出轴上。
- `ref`：`Referee` 实例，用于 UI 绘制。

配置参数（`Param`；PID 为 `LibXR::PID<float>::Param`，字段为 `k, p, i, d, i_limit, out_limit, cycle`）：

- `task_stack_depth`：线程栈深，默认 1536。
- `launcher_param.trig_gear_ratio`：拨弹盘相对 DM 输出轴的减速比，默认 1（直连）。不为 1 时，输出轴绝对角与拨弹盘角度不再一一对应，待发位计算不成立。
- `launcher_param.num_trig_tooth`：拨弹盘齿数，即每发转过的格数，默认 6。
- `launcher_param.speed_sync`：保留字段，三轮控制始终使用独立速度环，默认 `false`。
- `launcher_param.trig_ready_angle`：任一待发位的输出轴编码器角，单位 rad，默认 0。手动标定，方法见第 7 节；与真实值相差 2π / `num_trig_tooth` 的整数倍不影响结果。
- `launcher_param.trig_ready_tolerance`：判断“已在待发位”的阈值，单位 rad，默认 0.087（约 5°）。
- `fric_setpoint_speed_0`：第一级摩擦轮目标转速，单位 rpm，默认 3900。
- `fric_setpoint_speed_1`：第二级摩擦轮目标转速，单位 rpm，默认 2700。
- `pid_trig_angle`：拨弹角度环，输入 rad，输出拨弹盘角速度参考 rad/s，默认 `{.k = 1.0f, .p = 30.0f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 20.0f, .cycle = true}`。
- `pid_trig_speed`：拨弹速度环，反馈为输出轴 `omega` / `trig_gear_ratio`（rad/s），输出为输出轴力矩 N·m，默认 `{.k = 1.0f, .p = 0.6f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 3.0f, .cycle = false}`。默认值为未经实车整定的初值。
- `pid_fric_speed_0` 至 `pid_fric_speed_2`：摩擦轮速度环，默认均为 `{.k = 1.0f, .p = 0.0003f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 1.0f, .cycle = false}`。
- `thread_priority`：线程优先级，默认 `LibXR::Thread::Priority::MEDIUM`。
- `launcher_cmd_topic_name`：订阅的发射控制命令 Topic 名称，默认 `"launcher_cmd"`，与 CMD 的 `launcher_cmd_topic_name` 一致。
- `launcher_ref_topic_name`：订阅的裁判系统发射数据 Topic 名称，默认 `"launcher_ref"`，与 Referee 的 `referee_launcher_tp_name` 一致。

Dependencies:

- `cmd`: the `CMD` instance.
- `fric_motor_0` to `fric_motor_2`: `RMMotor` objects for the three friction wheel motors; 0 and 1 use the first speed target and 2 uses the second speed target, and friction wheel 2 is also used for round detection.
- `motor_trig`: the `DMMotor` of the trigger motor, a DM4310-2EC (absolute encoder on the output shaft) with the trigger disc mounted directly on the output shaft.
- `ref`: the `Referee` instance, used for UI drawing.

Configuration parameters (`Param`; the PIDs are `LibXR::PID<float>::Param` with fields `k, p, i, d, i_limit, out_limit, cycle`):

- `task_stack_depth`: thread stack depth, default 1536.
- `launcher_param.trig_gear_ratio`: reduction of the trigger disc relative to the DM output shaft, default 1 (direct mount). With any other value the output-shaft absolute angle no longer maps one-to-one to the disc angle and the ready positions are wrong.
- `launcher_param.num_trig_tooth`: number of trigger disc teeth, that is the teeth advanced per round, default 6.
- `launcher_param.speed_sync`: reserved compatibility field; three-wheel control always uses independent loops, default `false`.
- `launcher_param.trig_ready_angle`: output-shaft encoder angle of any one ready position in rad, default 0. Calibrated by hand as described in section 7; an error of a whole multiple of 2π / `num_trig_tooth` does not matter.
- `launcher_param.trig_ready_tolerance`: distance within which the disc counts as at a ready position, in rad, default 0.087 (about 5°).
- `fric_setpoint_speed_0`: target speed of the first-stage friction wheels in rpm, default 3900.
- `fric_setpoint_speed_1`: target speed of the second-stage friction wheels in rpm, default 2700.
- `pid_trig_angle`: trigger angle loop, input in rad, output a disc angular velocity reference in rad/s, default `{.k = 1.0f, .p = 30.0f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 20.0f, .cycle = true}`.
- `pid_trig_speed`: trigger speed loop, feedback is the output-shaft `omega` / `trig_gear_ratio` (rad/s) and output is output-shaft torque in N·m, default `{.k = 1.0f, .p = 0.6f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 3.0f, .cycle = false}`. The defaults are starting values that have not been tuned on the robot.
- `pid_fric_speed_0` to `pid_fric_speed_2`: friction wheel speed loops, all default to `{.k = 1.0f, .p = 0.0003f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 1.0f, .cycle = false}`.
- `thread_priority`: thread priority, default `LibXR::Thread::Priority::MEDIUM`.
- `launcher_cmd_topic_name`: name of the subscribed launcher command Topic, default `"launcher_cmd"`, matching the `launcher_cmd_topic_name` of CMD.
- `launcher_ref_topic_name`: name of the subscribed referee launcher data Topic, default `"launcher_ref"`, matching the `referee_launcher_tp_name` of Referee.

## 3. Topic

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `param.launcher_cmd_topic_name`（默认 `launcher_cmd`） | 订阅 | `CMD::LauncherCMD` | 发射命令 `isfire` |
| `param.launcher_ref_topic_name`（默认 `launcher_ref`） | 订阅 | `Referee::LauncherPack` | 热量上限与冷却值 |

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `param.launcher_cmd_topic_name` (default `launcher_cmd`) | Subscribe | `CMD::LauncherCMD` | Fire command `isfire` |
| `param.launcher_ref_topic_name` (default `launcher_ref`) | Subscribe | `Referee::LauncherPack` | Heat limit and cooling value |

## 4. 配置示例 / Configuration Example

`xrobot instance add QDU-Robomaster/HeroLauncher` 写入的实例，依赖填写为其他模块实例的 id：`cmd` 取自 `QDU-Robomaster/CMD` 实例，三个摩擦轮取自 `QDU-Robomaster/RMMotor` 实例，`motor_trig` 取自 `QDU-Robomaster/DMMotor` 实例，`ref` 取自 `QDU-Robomaster/Referee` 实例，它们须在本实例之前列出。摩擦轮模式事件通常由 `EventBinder` 从遥控器事件绑定到本实例（`HeroLauncher::LauncherEvent::SET_FRICMODE_READY` 等）。

An instance written by `xrobot instance add QDU-Robomaster/HeroLauncher`, with the dependencies set to the ids of other Module instances: `cmd` comes from a `QDU-Robomaster/CMD` instance, the three friction wheels from `QDU-Robomaster/RMMotor` instances, `motor_trig` from a `QDU-Robomaster/DMMotor` instance, and `ref` from a `QDU-Robomaster/Referee` instance; they are listed before this instance. The friction wheel mode events are usually bound from the remote controller events to this instance by `EventBinder` (`HeroLauncher::LauncherEvent::SET_FRICMODE_READY` and others).

```yaml
modules:
  - module: QDU-Robomaster/DMMotor
    id: motor_trig
    args:
      - can_bus: can1
      - param:
          model: DMMotor::Model::MOTOR_DM4310
          reverse: false
          can_id: 3
  - module: QDU-Robomaster/HeroLauncher
    id: HeroLauncher_0
    args:
      - cmd: cmd
      - fric_motor_0: motor_fric_front_left
      - fric_motor_1: motor_fric_front_right
      - fric_motor_2: motor_fric_back_left
      - motor_trig: motor_trig
      - ref: ref
      - param:
          task_stack_depth: 1536
          launcher_param:
            trig_gear_ratio: 1.0f
            num_trig_tooth: 6
            speed_sync: false
            trig_ready_angle: 0.0f
            trig_ready_tolerance: 0.087f
          fric_setpoint_speed_0: 3730.0f
          fric_setpoint_speed_1: 4000.0f
          pid_trig_angle:
            k: 1.0f
            p: 30.0f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 20.0f
            cycle: true
          pid_trig_speed:
            k: 1.0f
            p: 0.6f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 3.0f
            cycle: false
          pid_fric_speed_0:
            k: 1.0f
            p: 0.00048f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 1.0f
            cycle: false
          pid_fric_speed_1:
            k: 1.0f
            p: 0.00048f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 1.0f
            cycle: false
          pid_fric_speed_2:
            k: 1.0f
            p: 0.00048f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 1.0f
            cycle: false
          thread_priority: LibXR::Thread::Priority::MEDIUM
          launcher_cmd_topic_name: "launcher_cmd"
          launcher_ref_topic_name: "launcher_ref"
          fric_only_test: false
          bench_ignore_heat_limit: false
```

## 5. 依赖与硬件 / Dependencies and Hardware

依赖：

- `QDU-Robomaster/RMMotor`：摩擦轮电机。
- `QDU-Robomaster/DMMotor`：拨弹电机。
- `QDU-Robomaster/Motor`：电机命令与反馈类型。
- `QDU-Robomaster/CMD`：发射命令类型与 CMD 事件。
- `QDU-Robomaster/Referee`：热量数据类型与 UI 绘制。
- LibXR。

硬件：三个摩擦轮电机通过 `RMMotor` 实例接入；拨弹电机为 DM4310-2EC，通过 `DMMotor` 实例接入，拨弹盘直接装在输出轴上，电机 PMAX 须设为 2π，与 `DMMotor` 的 DM4310 量程一致。摩擦轮 2 的转矩反馈用于出弹检测。

Dependencies:

- `QDU-Robomaster/RMMotor`: friction wheel motors.
- `QDU-Robomaster/DMMotor`: trigger motor.
- `QDU-Robomaster/Motor`: motor command and feedback types.
- `QDU-Robomaster/CMD`: launcher command type and CMD events.
- `QDU-Robomaster/Referee`: heat data type and UI drawing.
- LibXR.

Hardware: three friction wheel motors attached through `RMMotor` instances; the trigger motor is a DM4310-2EC attached through a `DMMotor` instance, with the trigger disc mounted directly on the output shaft and the motor PMAX set to 2π to match the DM4310 range in `DMMotor`. The torque feedback of friction wheel 2 is used for round detection.


## 6. 三摩擦轮控制与台架 / Three-wheel control and bench mode

三路独立 PID；`speed_sync` 保留为配置兼容字段，不启用旧四轮同步算法。
软启动输出限幅从 0.1 开始，每 2 ms 控制周期增加 0.01，最大为 1.0。
三轮实际转速方向与目标一致、均达到各自目标的 90% 并连续保持 50 ms 后，
才完成软启动并允许供弹。后续供弹仍检查三轮转速。

`READY` 设置目标；`SAFE` 取消待执行的供弹并锁定拨弹当前位置，摩擦轮目标
按 8000 rpm/s 减速至零；`RELAX` 和失控事件向摩擦轮发送零电流，拨弹电机发送失能帧。
任意摩擦轮反馈掉线时放松全部电机；拨弹电机未使能或报错时只禁止拨弹，摩擦轮照常运行。
UI 仅显示三轮，并删除旧第四轮图形。

| Param 字段 | 默认值 | 行为 |
| --- | --- | --- |
| `fric_only_test` | `false` | 为 `true` 时始终禁止拨弹输出，拨弹电机每周期发送失能帧；三轮反馈掉线仍停机 |
| `bench_ignore_heat_limit` | `false` | 为 `true` 时绕过热量限制；与空载开关相互独立 |

正常配置为 BSP 的 `User/RobotConfig/hero.yaml`；独立空载配置为
`User/RobotConfig/hero_fric_bench.yaml`，仅构造遥控、三轮、拨弹占位、裁判/UI
与指示模块，不启动底盘或云台。空载配置开启上述两个开关，拨弹电机即使不接，
也只发送失能帧。右拨杆上/中/下分别为 `RELAX`/`SAFE`/`READY`。

适配保留当前工程三路电机与拨弹的 CAN 地址、方向、目标速度和 PID 参数。
参考测试工程的接线和整定值没有覆盖当前配置；若用于另一套接线，应在 YAML 中填写实物参数。

从 BSP 根目录选择配置并编译：

```sh
xrobot gen -c User/RobotConfig/hero_fric_bench.yaml
cmake --build --preset debug
# 回到正常 Hero 配置
xrobot gen -c User/RobotConfig/hero.yaml
cmake --build --preset debug
# 使用实际 LibXR PID 的主机侧回归检查
python Modules/QDU-Robomaster/HeroLauncher/tests/run_host.py
```

The three wheels use independent speed loops. Soft-start output limiting ramps from
0.1 to 1.0 in steps of 0.01 per control cycle. All three wheels must reach 90% of
their signed targets for 50 ms before loading is allowed. SAFE cancels loading,
holds the feeder position and ramps wheel targets to zero at 8000 rpm/s. RELAX
and lost control send zero current to the wheels and a disable frame to the
trigger motor. A missing friction wheel relaxes every motor; a disabled or
faulted trigger motor only blocks feeding while the wheels keep running. The
optional empty-bench mode never drives the trigger motor and sends it a disable
frame every cycle. Heat bypass is a separate explicit parameter; both switches
default to false.

Host tests exercise the production launcher and PID through constructor, events,
command/referee topics and motor outputs. OS, clock, topics and motor drivers use
host adapters; these tests do not establish hardware or CAN timing validation.
Empty-wheel direction/stop checks, dummy-projectile indexing and restricted-speed
live checks still require the physical mechanism in that order.

## 7. 拨弹盘待发位标定 / Ready-position calibration

`trig_ready_angle` 只需在装配后或更换电机、拆装拨弹盘后标定一次：

1. 用达妙上位机设置拨弹电机的 CAN ID 与 Master ID（反馈 ID 为 0x10 + `can_id`），PMAX 设为 2π。
2. 确认前进方向为角度减小；方向相反时把 `DMMotor` 的 `reverse` 设为 `true`。须先确定 `reverse`，再读取角度。
3. 手动把拨弹盘转到待发位：弹丸已压到位、尚未被推进摩擦轮。
4. 用调试器读取 `HeroLauncher_0.param_trig_.position`（已按 `reverse` 处理），填入 `trig_ready_angle`。
5. 重新生成并烧录后，上电即可直接发射，无需首发标定。

`trig_ready_angle` only needs calibrating once after assembly, or after replacing the motor or remounting the disc:

1. With the Damiao host tool, set the trigger motor CAN ID and Master ID (the feedback ID is 0x10 + `can_id`) and set PMAX to 2π.
2. Check that the forward direction is decreasing angle; if it is not, set `reverse` of the `DMMotor` to `true`. Settle `reverse` before reading the angle.
3. Turn the disc by hand to a ready position: a round pressed into place but not yet pushed into the friction wheels.
4. Read `HeroLauncher_0.param_trig_.position` in the debugger (already corrected for `reverse`) and enter it as `trig_ready_angle`.
5. After regenerating and flashing, the launcher fires straight after power-up without a first-shot calibration.
