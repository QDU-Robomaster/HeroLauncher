# HeroLauncher

英雄机器人发射机构模块：控制四个摩擦轮和一个拨弹盘，按裁判系统热量限制发射 / Hero robot launcher Module controlling four friction wheels and a trigger disc, with firing limited by the referee system heat

## 1. 模块作用 / Purpose

构造时，HeroLauncher 创建线程 `HeroLauncherThread`（栈深 `param.task_stack_depth`，优先级 `param.thread_priority`）。线程每轮先休眠 2 ms，再依次刷新电机反馈、执行软启动、热量计算、拨弹状态机和摩擦轮目标更新，最后计算 PID 输出并下发。摩擦轮与拨弹电机都以 `MODE_CURRENT` 下发，拨弹由角度环与速度环串联。拨弹电机 `Update()` 返回错误（例如 `RMMotor` 长时间无反馈）时，复位发射状态并放松全部电机。定时任务每 47 ms 绘制一次裁判系统 UI。

摩擦轮模式：`GetEvent()` 返回的 `LibXR::Event` 注册了 `HeroLauncher::LauncherEvent` 的 `SET_FRICMODE_RELAX`、`SET_FRICMODE_SAFE`、`SET_FRICMODE_READY`。`READY` 时摩擦轮 0、1 的目标转速为 `fric_setpoint_speed_0`，摩擦轮 2、3 的目标转速为 `fric_setpoint_speed_1`，单位 rpm；`RELAX` 与 `SAFE` 时目标转速为 0，发射状态复位。CMD 的 `CMD_EVENT_START_CTRL` 切换到 `SET_FRICMODE_RELAX`；`CMD_EVENT_LOST_CTRL` 复位全部发射状态并切换到 `SET_FRICMODE_RELAX`。

软启动：摩擦轮 0 的转速超过 `fric_setpoint_speed_0` 之前，四路摩擦轮速度环的输出限幅为 0.1，之后为 1.0。

拨弹触发（摩擦轮模式不是 `RELAX` 时）：`launcher_cmd` 的 `isfire` 上升沿进入 `SINGLE`，持续超过 200 ms 进入 `CONTINUE`，`isfire` 为假时回到 `SAFE`。

首发标定：第一次发射命令后，拨弹盘设定角每周期后退 2π/1000，直到软启动完成且摩擦轮 2 的 |扭矩| > 0.05（视为出弹），此时记录拨弹盘零点，并把设定角置为零点减 0.65 rad。

常规发弹：`SINGLE` 且热量允许时，拨弹盘设定角前进一格（2π / `num_trig_tooth`）；以摩擦轮 2 的 |扭矩| > 0.05 判定出弹并计入热量。拨弹盘角度由电机 `abs_angle` 的增量除以 `trig_gear_ratio` 累加得到。

热量：每判定一发出弹，热量加 100，并按裁判系统 `shooter_cooling_value` 每周期冷却；可发射数为 ⌊(`shooter_heat_limit` − 热量) / 100⌋，为 0 时拨弹盘不再推进。

`speed_sync` 为 `true` 时，在前（0、1）与后（2、3）两对摩擦轮上叠加同步 PID 的输出。

UI：定时任务在图层 2 上轮流绘制四个摩擦轮状态圆（前两路超过 3500 rpm、后两路超过 2200 rpm 为青色，否则为橙色）、拨弹盘位置圆弧（未完成首发标定时为橙色）和两条瞄准线。

Upon construction, HeroLauncher creates the thread `HeroLauncherThread` (stack depth `param.task_stack_depth`, priority `param.thread_priority`). Each iteration sleeps for 2 ms first, then refreshes the motor feedback, runs the soft start, the heat calculation, the trigger state machine and the friction wheel target update, and finally computes the PID outputs and sends them. The friction wheel and trigger motors are all sent in `MODE_CURRENT`, with an angle loop plus a speed loop for the trigger. When the trigger motor `Update()` returns an error (for example an `RMMotor` without feedback for a long time), the launch state is reset and all motors are relaxed. A timer task draws the referee system UI every 47 ms.

Friction wheel modes: `GetEvent()` returns a `LibXR::Event` on which `SET_FRICMODE_RELAX`, `SET_FRICMODE_SAFE` and `SET_FRICMODE_READY` of `HeroLauncher::LauncherEvent` are registered. In `READY`, the target speed of friction wheels 0 and 1 is `fric_setpoint_speed_0` and that of friction wheels 2 and 3 is `fric_setpoint_speed_1`, in rpm; in `RELAX` and `SAFE` the target speed is 0 and the launch state is reset. The CMD event `CMD_EVENT_START_CTRL` switches to `SET_FRICMODE_RELAX`; `CMD_EVENT_LOST_CTRL` resets all launch states and switches to `SET_FRICMODE_RELAX`.

Soft start: until the speed of friction wheel 0 exceeds `fric_setpoint_speed_0`, the output limit of the four friction wheel speed loops is 0.1, and 1.0 afterwards.

Trigger activation (when the friction wheel mode is not `RELAX`): a rising edge of `isfire` in `launcher_cmd` enters `SINGLE`, holding it for more than 200 ms enters `CONTINUE`, and `isfire` being false returns to `SAFE`.

First-shot calibration: after the first fire command, the trigger disc setpoint angle moves back by 2π/1000 per cycle until the soft start has finished and the |torque| of friction wheel 2 exceeds 0.05, which is taken as a round leaving. The trigger disc zero point is then recorded and the setpoint angle is set to the zero point minus 0.65 rad.

Normal firing: in `SINGLE` with heat available, the trigger disc setpoint angle advances by one tooth (2π / `num_trig_tooth`); a |torque| of friction wheel 2 above 0.05 is taken as a round leaving and is counted into the heat. The trigger disc angle is accumulated from the increments of the motor `abs_angle` divided by `trig_gear_ratio`.

Heat: each detected round adds 100 to the heat, which cools every cycle by the referee system `shooter_cooling_value`; the number of available shots is ⌊(`shooter_heat_limit` − heat) / 100⌋, and the trigger disc stops advancing when it is 0.

When `speed_sync` is `true`, the output of a synchronization PID is added on the front pair (0, 1) and the back pair (2, 3) of friction wheels.

UI: the timer task draws in turn, on layer 2, the four friction wheel status circles (cyan when the front two exceed 3500 rpm and the back two exceed 2200 rpm, orange otherwise), the trigger disc position arc (orange until the first-shot calibration is complete) and two aiming lines.

## 2. 构造接口 / Constructor

```cpp
HeroLauncher(CMD& cmd,
             RMMotor& fric_motor_0,
             RMMotor& fric_motor_1,
             RMMotor& fric_motor_2,
             RMMotor& fric_motor_3,
             RMMotor& motor_trig,
             Referee& ref,
             const Param& param = {...});  // 节选 / excerpt
```

依赖：

- `cmd`：`CMD` 实例。
- `fric_motor_0` 至 `fric_motor_3`：`RMMotor`，四个摩擦轮电机；0、1 为第一级，2、3 为第二级，摩擦轮 2 同时用于出弹检测。
- `motor_trig`：`RMMotor`，拨弹电机。
- `ref`：`Referee` 实例，用于 UI 绘制。

配置参数（`Param`；PID 为 `LibXR::PID<float>::Param`，字段为 `k, p, i, d, i_limit, out_limit, cycle`）：

- `task_stack_depth`：线程栈深，默认 1536。
- `launcher_param.trig_gear_ratio`：拨弹电机减速比，默认 19.2032。
- `launcher_param.num_trig_tooth`：拨弹盘齿数，即每发转过的格数，默认 6。
- `launcher_param.speed_sync`：摩擦轮同步控制开关，默认 `false`。
- `fric_setpoint_speed_0`：第一级摩擦轮目标转速，单位 rpm，默认 3900。
- `fric_setpoint_speed_1`：第二级摩擦轮目标转速，单位 rpm，默认 2700。
- `pid_trig_angle`：拨弹角度环，默认 `{.k = 1.0f, .p = 2000.0f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 2000.0f, .cycle = true}`。
- `pid_trig_speed`：拨弹速度环，默认 `{.k = 1.0f, .p = 0.0013f, .i = 0.0f, .d = 0.0f, .i_limit = 1.0f, .out_limit = 1.0f, .cycle = false}`。
- `pid_fric_speed_0` 至 `pid_fric_speed_3`：摩擦轮速度环，默认均为 `{.k = 1.0f, .p = 0.0003f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 1.0f, .cycle = false}`。
- `thread_priority`：线程优先级，默认 `LibXR::Thread::Priority::MEDIUM`。
- `launcher_cmd_topic_name`：订阅的发射控制命令 Topic 名称，默认 `"launcher_cmd"`，与 CMD 的 `launcher_cmd_topic_name` 一致。
- `launcher_ref_topic_name`：订阅的裁判系统发射数据 Topic 名称，默认 `"launcher_ref"`，与 Referee 的 `referee_launcher_tp_name` 一致。

Dependencies:

- `cmd`: the `CMD` instance.
- `fric_motor_0` to `fric_motor_3`: `RMMotor` objects for the four friction wheel motors; 0 and 1 are the first stage and 2 and 3 the second stage, and friction wheel 2 is also used for round detection.
- `motor_trig`: the `RMMotor` of the trigger motor.
- `ref`: the `Referee` instance, used for UI drawing.

Configuration parameters (`Param`; the PIDs are `LibXR::PID<float>::Param` with fields `k, p, i, d, i_limit, out_limit, cycle`):

- `task_stack_depth`: thread stack depth, default 1536.
- `launcher_param.trig_gear_ratio`: trigger motor reduction ratio, default 19.2032.
- `launcher_param.num_trig_tooth`: number of trigger disc teeth, that is the teeth advanced per round, default 6.
- `launcher_param.speed_sync`: friction wheel synchronization switch, default `false`.
- `fric_setpoint_speed_0`: target speed of the first-stage friction wheels in rpm, default 3900.
- `fric_setpoint_speed_1`: target speed of the second-stage friction wheels in rpm, default 2700.
- `pid_trig_angle`: trigger angle loop, default `{.k = 1.0f, .p = 2000.0f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 2000.0f, .cycle = true}`.
- `pid_trig_speed`: trigger speed loop, default `{.k = 1.0f, .p = 0.0013f, .i = 0.0f, .d = 0.0f, .i_limit = 1.0f, .out_limit = 1.0f, .cycle = false}`.
- `pid_fric_speed_0` to `pid_fric_speed_3`: friction wheel speed loops, all default to `{.k = 1.0f, .p = 0.0003f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 1.0f, .cycle = false}`.
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

`xrobot instance add QDU-Robomaster/HeroLauncher` 写入的实例，依赖填写为其他模块实例的 id：`cmd` 取自 `QDU-Robomaster/CMD` 实例，四个摩擦轮与 `motor_trig` 取自 `QDU-Robomaster/RMMotor` 实例，`ref` 取自 `QDU-Robomaster/Referee` 实例，它们须在本实例之前列出。摩擦轮模式事件通常由 `EventBinder` 从遥控器事件绑定到本实例（`HeroLauncher::LauncherEvent::SET_FRICMODE_READY` 等）。

An instance written by `xrobot instance add QDU-Robomaster/HeroLauncher`, with the dependencies set to the ids of other Module instances: `cmd` comes from a `QDU-Robomaster/CMD` instance, the four friction wheels and `motor_trig` from `QDU-Robomaster/RMMotor` instances, and `ref` from a `QDU-Robomaster/Referee` instance; they are listed before this instance. The friction wheel mode events are usually bound from the remote controller events to this instance by `EventBinder` (`HeroLauncher::LauncherEvent::SET_FRICMODE_READY` and others).

```yaml
modules:
  - module: QDU-Robomaster/HeroLauncher
    id: HeroLauncher_0
    args:
      - cmd: cmd
      - fric_motor_0: motor_fric_front_left
      - fric_motor_1: motor_fric_front_right
      - fric_motor_2: motor_fric_back_left
      - fric_motor_3: motor_fric_back_right
      - motor_trig: motor_trig
      - ref: ref
      - param:
          task_stack_depth: 1536
          launcher_param:
            trig_gear_ratio: 19.2032f
            num_trig_tooth: 6
            speed_sync: false
          fric_setpoint_speed_0: 3730.0f
          fric_setpoint_speed_1: 4000.0f
          pid_trig_angle:
            k: 1.0f
            p: 2000.0f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 2000.0f
            cycle: true
          pid_trig_speed:
            k: 1.0f
            p: 0.0013f
            i: 0.0f
            d: 0.0f
            i_limit: 1.0f
            out_limit: 1.0f
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
          pid_fric_speed_3:
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
```

## 5. 依赖与硬件 / Dependencies and Hardware

依赖：

- `QDU-Robomaster/RMMotor`：摩擦轮与拨弹电机。
- `QDU-Robomaster/Motor`：电机命令与反馈类型。
- `QDU-Robomaster/CMD`：发射命令类型与 CMD 事件。
- `QDU-Robomaster/Referee`：热量数据类型与 UI 绘制。
- LibXR。

硬件：四个摩擦轮电机和一个拨弹电机，均通过 `RMMotor` 实例接入；摩擦轮 2 的转矩反馈用于出弹检测。

Dependencies:

- `QDU-Robomaster/RMMotor`: friction wheel and trigger motors.
- `QDU-Robomaster/Motor`: motor command and feedback types.
- `QDU-Robomaster/CMD`: launcher command type and CMD events.
- `QDU-Robomaster/Referee`: heat data type and UI drawing.
- LibXR.

Hardware: four friction wheel motors and one trigger motor, all attached through `RMMotor` instances; the torque feedback of friction wheel 2 is used for round detection.
