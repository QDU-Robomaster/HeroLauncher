# HeroLauncher

英雄机器人发射机构模块：控制四个摩擦轮（两级）和一个拨弹盘，按裁判系统热量限制发射。

## 工作方式

- 构造时创建线程 `HeroLauncherThread`（栈深 `param.task_stack_depth`，优先级
  `param.thread_priority`），每轮先休眠 2 ms 再执行：刷新电机反馈 → 软启动 / 热量 / 拨弹状态机 /
  摩擦轮目标 → PID 输出。
- 摩擦轮模式：`GetEvent()` 返回的 `LibXR::Event` 注册了 `HeroLauncher::LauncherEvent` 的
  `SET_FRICMODE_RELAX`、`SET_FRICMODE_SAFE`、`SET_FRICMODE_READY`。`READY` 时摩擦轮 0、1 的目标转速为
  `fric_setpoint_speed_0`，摩擦轮 2、3 为 `fric_setpoint_speed_1`（rpm）；`RELAX` / `SAFE` 时目标为 0，
  发射状态复位。CMD 的 `CMD_EVENT_START_CTRL` 切到 `SET_FRICMODE_RELAX`；`CMD_EVENT_LOST_CTRL`
  复位全部发射状态并切到 `SET_FRICMODE_RELAX`。
- 软启动：摩擦轮 0 的转速超过 `fric_setpoint_speed_0` 之前，四路摩擦轮速度环输出限幅 0.1，
  之后为 1.0。
- 拨弹触发（非 `RELAX` 时）：`launcher_cmd` 的 `isfire` 上升沿进入 `SINGLE`，按住超过 200 ms
  进入 `CONTINUE`，松开回到 `SAFE`。当前代码只在 `SINGLE` 下推进拨弹盘，`CONTINUE` 不会额外发弹。
- 首发标定：第一次发射命令后，拨弹盘设定角每周期后退 2π/1000，直到软启动完成且摩擦轮 2 的
  |扭矩| > 0.05（视为出弹），此时记录拨弹盘零点并把设定角置为零点 − 0.65 rad。
- 常规发弹：`SINGLE` 且热量允许时，拨弹盘设定角前进一格（2π / `num_trig_tooth`）；以摩擦轮 2 的
  |扭矩| > 0.05 判定出弹并计入热量。拨弹盘角度由电机 `abs_angle` 增量除以 `trig_gear_ratio`
  累加。
- 热量：每判定一发出弹热量加 100，按裁判系统 `shooter_cooling_value` 每周期冷却，
  可发射数 = ⌊(`shooter_heat_limit` − 热量) / 100⌋，为 0 时不再推进拨弹盘。
- 输出：摩擦轮与拨弹电机都以 `MODE_CURRENT` 下发（拨弹为角度环 + 速度环）。拨弹电机
  `Update()` 返回错误（如 `RMMotor` 长时间无反馈）时复位发射状态并放松全部电机。
- `speed_sync = true` 时在前（0、1）、后（2、3）两对摩擦轮上叠加同步 PID 输出；该同步 PID 的参数
  在代码中固定为 `p = 0`，当前不产生额外输出。
- 裁判系统 UI：定时任务每 47 ms 在图层 2 上轮流绘制四个摩擦轮状态圆（前两路 > 3500 rpm、
  后两路 > 2200 rpm 为青色，否则橙色）、拨弹盘位置圆弧（未完成首发标定时为橙色）和两条瞄准线。

Topic：

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `launcher_cmd` | 订阅 | `CMD::LauncherCMD` | 发射命令 `isfire` |
| `launcher_ref` | 订阅 | `Referee::LauncherPack` | 热量上限与冷却值 |

## 依赖

- `QDU-Robomaster/RMMotor`：摩擦轮与拨弹电机。
- `QDU-Robomaster/Motor`：电机命令与反馈类型。
- `QDU-Robomaster/CMD`：发射命令类型与 CMD 事件。
- `QDU-Robomaster/Referee`：热量数据类型与 UI 绘制。

无外部软件包，仅使用 LibXR。

## 构造接口

```cpp
HeroLauncher(CMD& cmd,
             RMMotor& fric_motor_0,
             RMMotor& fric_motor_1,
             RMMotor& fric_motor_2,
             RMMotor& fric_motor_3,
             RMMotor& motor_trig,
             Referee& ref,
             const Param& param = {...});
```

依赖：

- `cmd`：`CMD` 实例。
- `fric_motor_0..3`：`RMMotor`，四个摩擦轮电机；0、1 为第一级，2、3 为第二级（摩擦轮 2 同时用于出弹检测）。
- `motor_trig`：`RMMotor`，拨弹电机。
- `ref`：`Referee` 实例，用于 UI 绘制。

配置（`Param`；PID 为 `LibXR::PID<float>::Param`，字段 `k, p, i, d, i_limit, out_limit, cycle`）：

- `task_stack_depth`：线程栈深，默认 1536。
- `launcher_param.trig_gear_ratio`：拨弹电机减速比，默认 19.2032。
- `launcher_param.num_trig_tooth`：拨弹盘齿数（每发转过的格数），默认 6。
- `launcher_param.speed_sync`：摩擦轮同步控制开关，默认 `false`。
- `fric_setpoint_speed_0`：第一级摩擦轮目标转速 (rpm)，默认 3900。
- `fric_setpoint_speed_1`：第二级摩擦轮目标转速 (rpm)，默认 2700。
- `pid_trig_angle`：拨弹角度环，默认 `k = 1, p = 2000, out_limit = 2000, cycle = true`。
- `pid_trig_speed`：拨弹速度环，默认 `k = 1, p = 0.0013, i_limit = 1, out_limit = 1`。
- `pid_fric_speed_0..3`：摩擦轮速度环，默认 `k = 1, p = 0.0003, out_limit = 1`。
- `thread_priority`：线程优先级，默认 `MEDIUM`。

## 使用

```sh
xrobot module add QDU-Robomaster/HeroLauncher
xrobot setup
xrobot instance add QDU-Robomaster/HeroLauncher
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出；
把依赖填为已列出的实例 id：

```yaml
modules:
  - module: QDU-Robomaster/HeroLauncher
    id: herolauncher_0
    args:
      - cmd: cmd
      - fric_motor_0: motor_fric_front_left
      - fric_motor_1: motor_fric_front_right
      - fric_motor_2: motor_fric_back_left
      - fric_motor_3: motor_fric_back_right
      - motor_trig: motor_trig
      - ref: referee
      - param:
          task_stack_depth: '1536'
          launcher_param:
            trig_gear_ratio: 19.2032f
            num_trig_tooth: '6'
            speed_sync: 'false'
          fric_setpoint_speed_0: 3900.0f
          fric_setpoint_speed_1: 2700.0f
          pid_trig_angle:
            k: 1.0f
            p: 2000.0f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 2000.0f
            cycle: 'true'
          pid_trig_speed:
            k: 1.0f
            p: 0.0013f
            i: 0.0f
            d: 0.0f
            i_limit: 1.0f
            out_limit: 1.0f
            cycle: 'false'
          pid_fric_speed_0:
            k: 1.0f
            p: 0.0003f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 1.0f
            cycle: 'false'
          pid_fric_speed_1:
            k: 1.0f
            p: 0.0003f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 1.0f
            cycle: 'false'
          pid_fric_speed_2:
            k: 1.0f
            p: 0.0003f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 1.0f
            cycle: 'false'
          pid_fric_speed_3:
            k: 1.0f
            p: 0.0003f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 1.0f
            cycle: 'false'
          thread_priority: LibXR::Thread::Priority::MEDIUM
```

所有依赖都是其他模块实例的 id，须在本实例之前列出：`cmd` 为 `QDU-Robomaster/CMD` 实例，
四个摩擦轮与 `motor_trig` 为 `QDU-Robomaster/RMMotor` 实例，`referee` 为
`QDU-Robomaster/Referee` 实例。本例没有需要 BSP 用 `XR_REGISTER` 注册的对象。摩擦轮模式事件通常由
`EventBinder` 从遥控器事件绑定到本实例（`HeroLauncher::LauncherEvent::SET_FRICMODE_READY` 等）。

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/QDU-Robomaster/HeroLauncher`
（在 BSP 中）打印当前的构造函数。
