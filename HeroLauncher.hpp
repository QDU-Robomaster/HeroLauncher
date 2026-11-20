#pragma once
// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 英雄机器人发射机构模块：控制三个摩擦轮和一个拨弹盘，按裁判系统热量限制发射 / Hero robot launcher Module controlling three friction wheels and a trigger disc, with firing limited by the referee system heat
depends:
- id: QDU-Robomaster/RMMotor
  ref: same-or-dev
- id: QDU-Robomaster/DMMotor
  ref: same-or-dev
- id: QDU-Robomaster/CMD
  ref: same-or-dev
- id: QDU-Robomaster/Referee
  ref: same-or-dev
- id: QDU-Robomaster/Motor
  ref: same-or-dev
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include "CMD.hpp"
#include "DMMotor.hpp"
#include "Motor.hpp"
#include "RMMotor.hpp"
#include "Referee.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "mutex.hpp"
#include "pid.hpp"
#include "thread.hpp"
#define UI_HERO_LAUNCHER_LAYER 2

/**
 * @brief 英雄机器人发射机构模块：控制三个摩擦轮和一个拨弹盘，按裁判系统热量限制发射。
 *        Hero robot launcher Module controlling three friction wheels and a trigger
 *        disc, with firing limited by the referee system heat.
 */
class HeroLauncher
{
 public:
  /// 摩擦轮数量
  /// Number of friction wheels
  static constexpr int FRIC_NUM = 3;
  static constexpr float SOFT_START_INITIAL_OUT_LIMIT = 0.1f;
  static constexpr float SOFT_START_OUT_LIMIT_STEP = 0.01f;
  static constexpr float FRIC_READY_SPEED_RATIO = 0.9f;
  static constexpr uint32_t FRIC_READY_HOLD_TIME_MS = 50;
  static constexpr float FRIC_SAFE_DECELERATION_RPMPS = 8000.0f;

  /// DM 电机反馈状态：已使能
  /// DM motor feedback state: enabled
  static constexpr uint8_t DM_STATE_ENABLED = 1;
  /// DM 电机反馈状态：失能
  /// DM motor feedback state: disabled
  static constexpr uint8_t DM_STATE_DISABLED = 0;

  /**
   * @brief 拨弹模式。
   *        Trigger modes.
   */
  enum class TrigMode : uint8_t
  {
    RELAX = 0,  ///< 放松：拨弹电机输出为 0 Relax: trigger motor output is 0
    SAFE,       ///< 安全：保持当前设定角 Safe: hold the current setpoint angle
    SINGLE,     ///< 单发 Single shot
    CONTINUE,   ///< 持续按下 Fire held
  };

  /**
   * @brief 摩擦轮模式事件，数值同时是 `GetEvent()` 上注册的事件 ID。
   *        Friction wheel mode events; the values are also the event IDs registered on
   *        `GetEvent()`.
   */
  enum class LauncherEvent : uint8_t
  {
    SET_FRICMODE_RELAX,  ///< 放松：目标转速为 0，发射状态复位
                         ///< Relax: target speed 0, launch state reset
    SET_FRICMODE_SAFE,   ///< 安全：摩擦轮减速至零，拨弹保持当前位置
                         ///< Safe: ramp wheels to zero and hold the trigger position
    SET_FRICMODE_READY,  ///< 就绪：摩擦轮按目标转速运行
                         ///< Ready: friction wheels run at the target speed
  };

  /**
   * @brief 热量控制状态。
   *        Heat control state.
   */
  struct HeatControl
  {
    float heat;           ///< 当前热量 Current heat
    float last_heat;      ///< 上一次的热量 Previous heat
    float heat_limit;     ///< 热量上限 Heat limit
    float speed_limit;    ///< 弹丸初速上限 Projectile speed limit
    float cooling_rate;   ///< 冷却速率 Cooling rate
    float heat_increase;  ///< 每发增加的热量 Heat added per round

    uint8_t cooling_acc;  ///< 冷却增益 Cooling gain

    uint32_t available_shot;  ///< 热量范围内还可发射的数量
                              ///< Number of rounds still available within the heat
  };

  /**
   * @brief 发射器参数。
   *        Launcher parameters.
   */
  struct LauncherParam
  {
    float trig_gear_ratio;   ///< 拨弹盘相对 DM 输出轴的减速比，直连时为 1
                             ///< Disc reduction relative to the DM output shaft; 1 when
                             ///< the disc is mounted directly
    uint8_t num_trig_tooth;  ///< 拨弹盘齿数，即每发转过的格数
                             ///< Number of trigger disc teeth, the teeth advanced per
                             ///< round
    bool speed_sync;  ///< 保留配置字段；三轮使用独立速度环 Reserved; independent loops
    float trig_ready_angle;  ///< 某一待发位的输出轴编码器角 (rad)，手动读取
                             ///< Output-shaft encoder angle of one ready position (rad),
                             ///< read by hand
    float trig_ready_tolerance;  ///< 距待发位小于此值视为已就位 (rad)
                                 ///< Within this distance of a ready position the disc
                                 ///< counts as in position (rad)
  };

  /**
   * @brief 英雄发射机构配置参数。
   *        Hero launcher configuration parameters.
   */
  struct Param
  {
    uint32_t task_stack_depth;     ///< 线程栈深
                                   ///< Thread stack depth
    LauncherParam launcher_param;  ///< 发射器参数
                                   ///< Launcher parameters
    float fric_setpoint_speed_0;   ///< 第一级摩擦轮目标转速 (rpm)
                                   ///< Target speed of the first-stage friction wheels
                                   ///< (rpm)
    float fric_setpoint_speed_1;   ///< 第二级摩擦轮目标转速 (rpm)
                                   ///< Target speed of the second-stage friction wheels
                                   ///< (rpm)
    LibXR::PID<float>::Param pid_trig_angle;    ///< 拨弹角度环 PID
                                                ///< Trigger angle-loop PID
    LibXR::PID<float>::Param pid_trig_speed;    ///< 拨弹速度环 PID
                                                ///< Trigger speed-loop PID
    LibXR::PID<float>::Param pid_fric_speed_0;  ///< 摩擦轮 0 速度环 PID
                                                ///< Friction wheel 0 speed-loop PID
    LibXR::PID<float>::Param pid_fric_speed_1;  ///< 摩擦轮 1 速度环 PID
                                                ///< Friction wheel 1 speed-loop PID
    LibXR::PID<float>::Param pid_fric_speed_2;  ///< 摩擦轮 2 速度环 PID
                                                ///< Friction wheel 2 speed-loop PID
    LibXR::Thread::Priority thread_priority;    ///< 线程优先级
                                                ///< Thread priority
    const char* launcher_cmd_topic_name;        ///< 订阅的发射控制命令 Topic 名称
                                                ///< Name of the subscribed launcher
                                                ///< command Topic
    const char* launcher_ref_topic_name;        ///< 订阅的裁判系统发射数据 Topic 名称
                                                ///< Name of the subscribed referee
                                                ///< launcher data Topic
    bool fric_only_test;                        ///< 空载摩擦轮测试：拨弹始终零电流
    bool bench_ignore_heat_limit;  ///< 台架测试时绕过热量限制，正常运行应关闭
  };

  /**
   * @brief 构造 HeroLauncher，创建控制线程、UI 定时任务并注册事件。
   *        Construct HeroLauncher, create the control thread and the UI timer task, and
   *        register the events.
   *
   * @param cmd CMD 实例。
   *            CMD instance.
   * @param fric_motor_0 摩擦轮 0 电机。
   *                     Friction wheel 0 motor.
   * @param fric_motor_1 摩擦轮 1 电机。
   *                     Friction wheel 1 motor.
   * @param fric_motor_2 摩擦轮 2 电机，同时用于出弹检测。
   *                     Friction wheel 2 motor, also used for round detection.
   * @param motor_trig 拨弹电机（DM4310-2EC，输出轴绝对编码器）。
   *                   Trigger motor (DM4310-2EC with an output-shaft absolute encoder).
   * @param ref Referee 实例，用于 UI 绘制。
   *            Referee instance, used for UI drawing.
   * @param param 配置参数。
   *              Configuration parameters.
   */
  HeroLauncher(CMD& cmd, RMMotor& fric_motor_0, RMMotor& fric_motor_1,
               RMMotor& fric_motor_2, DMMotor& motor_trig, Referee& ref,
               const Param& param = {.task_stack_depth = 1536,
                                     .launcher_param = {.trig_gear_ratio = 1.0f,
                                                        .num_trig_tooth = 6,
                                                        .speed_sync = false,
                                                        .trig_ready_angle = 0.0f,
                                                        .trig_ready_tolerance = 0.087f},
                                     .fric_setpoint_speed_0 = 3900.0f,
                                     .fric_setpoint_speed_1 = 2700.0f,
                                     .pid_trig_angle = {.k = 1.0f,
                                                        .p = 30.0f,
                                                        .i = 0.0f,
                                                        .d = 0.0f,
                                                        .i_limit = 0.0f,
                                                        .out_limit = 20.0f,
                                                        .cycle = true},
                                     .pid_trig_speed = {.k = 1.0f,
                                                        .p = 0.6f,
                                                        .i = 0.0f,
                                                        .d = 0.0f,
                                                        .i_limit = 0.0f,
                                                        .out_limit = 3.0f,
                                                        .cycle = false},
                                     .pid_fric_speed_0 = {.k = 1.0f,
                                                          .p = 0.0003f,
                                                          .i = 0.0f,
                                                          .d = 0.0f,
                                                          .i_limit = 0.0f,
                                                          .out_limit = 1.0f,
                                                          .cycle = false},
                                     .pid_fric_speed_1 = {.k = 1.0f,
                                                          .p = 0.0003f,
                                                          .i = 0.0f,
                                                          .d = 0.0f,
                                                          .i_limit = 0.0f,
                                                          .out_limit = 1.0f,
                                                          .cycle = false},
                                     .pid_fric_speed_2 = {.k = 1.0f,
                                                          .p = 0.0003f,
                                                          .i = 0.0f,
                                                          .d = 0.0f,
                                                          .i_limit = 0.0f,
                                                          .out_limit = 1.0f,
                                                          .cycle = false},
                                     .thread_priority = LibXR::Thread::Priority::MEDIUM,
                                     .launcher_cmd_topic_name = "launcher_cmd",
                                     .launcher_ref_topic_name = "launcher_ref",
                                     .fric_only_test = false,
                                     .bench_ignore_heat_limit = false})
      : cmd_(&cmd),
        fric_only_test_(param.fric_only_test),
        bench_ignore_heat_limit_(param.bench_ignore_heat_limit),
        trig_angle_pid_(param.pid_trig_angle),
        trig_speed_pid_(param.pid_trig_speed),
        fric_speed_pid_({{LibXR::PID<float>(param.pid_fric_speed_0),
                          LibXR::PID<float>(param.pid_fric_speed_1),
                          LibXR::PID<float>(param.pid_fric_speed_2)}}),
        trig_gear_ratio_(param.launcher_param.trig_gear_ratio),
        num_trig_tooth_(param.launcher_param.num_trig_tooth),
        trig_ready_angle_(param.launcher_param.trig_ready_angle),
        trig_ready_tolerance_(param.launcher_param.trig_ready_tolerance),
        ref_(&ref)
  {
    launcher_cmd_topic_name_ = param.launcher_cmd_topic_name;
    launcher_ref_topic_name_ = param.launcher_ref_topic_name;
    motor_trig_ = &motor_trig;

    fric_motor_[0] = &fric_motor_0;
    fric_motor_[1] = &fric_motor_1;
    fric_motor_[2] = &fric_motor_2;

    param_fric_target_speed_[0] = param.fric_setpoint_speed_0;
    param_fric_target_speed_[1] = param.fric_setpoint_speed_1;

    last_wakeup_ = LibXR::Timebase::GetMicroseconds();

    thread_.Create(this, ThreadFunc, "HeroLauncherThread", param.task_stack_depth,
                   param.thread_priority);

    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, HeroLauncher* self, uint32_t event_id)
        {
          UNUSED(in_isr);
          UNUSED(event_id);
          self->mutex_.Lock();
          self->LostCtrl();
          self->mutex_.Unlock();
        },
        this);

    auto start_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, HeroLauncher* self, uint32_t event_id)
        {
          UNUSED(in_isr);
          UNUSED(event_id);
          self->mutex_.Lock();
          self->SetMode(static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_RELAX));
          self->mutex_.Unlock();
        },
        this);

    cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);
    cmd_->GetEvent().Register(CMD::CMD_EVENT_START_CTRL, start_ctrl_callback);

    auto event_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, HeroLauncher* self, uint32_t event_id)
        {
          UNUSED(in_isr);
          self->mutex_.Lock();
          self->SetMode(event_id);
          self->mutex_.Unlock();
        },
        this);
    event_.Register(static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_RELAX),
                    event_callback);
    event_.Register(static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_SAFE),
                    event_callback);
    event_.Register(static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_READY),
                    event_callback);

    void (*DrawUi)(HeroLauncher*) = [](HeroLauncher* hero) { hero->DrawUI(); };
    ui_timer_handle_ = LibXR::Timer::CreateTask(DrawUi, this, 47);
    LibXR::Timer::Add(ui_timer_handle_);
    LibXR::Timer::Start(ui_timer_handle_);
  }

  /**
   * @brief 控制线程函数：订阅发射命令与裁判系统数据，每 2 ms 执行一轮更新与控制。
   *        Control thread function that subscribes to the launcher command and the
   *        referee data and runs one update and control iteration every 2 ms.
   *
   * @param self HeroLauncher 实例指针。
   *             Pointer to the HeroLauncher instance.
   */
  static void ThreadFunc(HeroLauncher* self)
  {
    LibXR::Topic::ASyncSubscriber<CMD::LauncherCMD> cmd_sub(
        self->launcher_cmd_topic_name_);
    LibXR::Topic::ASyncSubscriber<Referee::LauncherPack> launcher_ref(
        self->launcher_ref_topic_name_);
    cmd_sub.StartWaiting();
    launcher_ref.StartWaiting();

    self->last_wakeup_time_ = LibXR::Timebase::GetMilliseconds();
    self->last_online_time_ = LibXR::Timebase::GetMicroseconds();

    while (true)
    {
      LibXR::Thread::Sleep(2);

      auto now = LibXR::Timebase::GetMicroseconds();
      self->dt_ = (now - self->last_online_time_).ToSecondf();
      self->last_online_time_ = now;

      if (cmd_sub.Available())
      {
        self->launcher_cmd_ = cmd_sub.GetData();
        cmd_sub.StartWaiting();
      }
      if (launcher_ref.Available())
      {
        self->ref_data_.rs.shooter_cooling_value =
            launcher_ref.GetData().rs.shooter_cooling_value;
        self->ref_data_.rs.shooter_heat_limit =
            launcher_ref.GetData().rs.shooter_heat_limit;
        launcher_ref.StartWaiting();
      }

      self->mutex_.Lock();
      self->Update();
      self->Solve();
      self->Control();
      self->mutex_.Unlock();
    }
  }

  /**
   * @brief 获取发射机构事件对象，`LauncherEvent` 的三个值注册在其上。
   *        Get the launcher event object on which the three values of `LauncherEvent`
   *        are registered.
   *
   * @return 事件对象的引用。
   *         Reference to the event object.
   */
  LibXR::Event& GetEvent() { return event_; }

 private:
  CMD* cmd_;
  bool fric_only_test_;
  bool bench_ignore_heat_limit_;

  TrigMode trig_mode_ = TrigMode::SAFE;

  HeatControl heat_ctrl_{};

  bool soft_start_finish_ = false;
  float soft_start_out_limit_ = SOFT_START_INITIAL_OUT_LIMIT;
  LibXR::MillisecondTimestamp fric_ready_time_ = 0;

  float dt_ = 0.0f;

  LibXR::MillisecondTimestamp now_ = 0;

  LibXR::MicrosecondTimestamp last_wakeup_;

  RMMotor* fric_motor_[FRIC_NUM];

  DMMotor* motor_trig_;

  LibXR::ErrorCode motor_state_ = LibXR::ErrorCode::OK;

  float trig_setpoint_angle_ = 0.0f;
  float trig_setpoint_speed_ = 0.0f;

  float trig_angle_ = 0.0f;
  bool trig_angle_valid_ = false;  // trig_angle_ 已由绝对编码器初始化
  bool setpoint_on_grid_ = false;  // 设定角已落在待发位网格上
  float trig_output_ = 0.0f;

  float param_fric_target_speed_[2] = {0.0f, 0.0f};
  float fric_target_rpm_[FRIC_NUM] = {};

  LibXR::PID<float> trig_angle_pid_;
  LibXR::PID<float> trig_speed_pid_;

  std::array<LibXR::PID<float>, FRIC_NUM> fric_speed_pid_;

  float trig_gear_ratio_;
  uint8_t num_trig_tooth_;
  float trig_ready_angle_;
  float trig_ready_tolerance_;

  bool fire_flag_ = false;    // 发射命令标志位
  uint8_t fired_ = 0;         // 已发射弹丸
  bool enable_fire_ = false;  // 拨弹盘旋转命令发出标志位
  bool mark_launch_ = false;  // 拨弹发射完成标志位

  LibXR::MillisecondTimestamp start_fire_time_ = 0;
  LibXR::MillisecondTimestamp finish_fire_time_ = 0;
  uint32_t real_launch_delay_ = 0.0f;

  bool last_fire_notify_ = false;
  bool press_continue_ = false;
  LibXR::MillisecondTimestamp fire_press_time_ = 0;

  LauncherEvent launcher_state_ = LauncherEvent::SET_FRICMODE_RELAX;

  Motor::Feedback param_motor_fric_[FRIC_NUM]{};

  Motor::Feedback param_trig_{};

  Motor::MotorCmd cmd_fric_[FRIC_NUM] = {
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 1.0f,
                      .velocity = 0},
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 1.0f,
                      .velocity = 0},
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 1.0f,
                      .velocity = 0}};
  Motor::MotorCmd cmd_trig_ =
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_TORQUE, .torque = 0.0f};

  // 线程与事件
  const char* launcher_cmd_topic_name_ = nullptr;
  const char* launcher_ref_topic_name_ = nullptr;
  LibXR::Thread thread_;
  LibXR::Mutex mutex_;
  LibXR::Event event_;

  Referee* ref_;

  uint8_t ui_step_ = 0;  // UI绘制步骤
  uint8_t ui_tick_ = 0;  // UI绘制计数器

  LibXR::MillisecondTimestamp last_wakeup_time_ = 0;
  LibXR::MicrosecondTimestamp last_online_time_ = 0;

  LibXR::Timer::TimerHandle ui_timer_handle_;  // UI 绘制定时器句柄（47 ms）
  /**
   * @brief 更新电机反馈与拨弹盘角度。
   *        Update the motor feedback and the trigger disc angle.
   */
  void Update()
  {
    this->last_wakeup_ = LibXR::Timebase::GetMicroseconds();
    this->now_ = LibXR::Timebase::GetMilliseconds();

    const float LAST_TRIG_MOTOR_ANGLE = LibXR::CycleValue<float>(param_trig_.abs_angle);
    motor_state_ = LibXR::ErrorCode::OK;
    for (int i = 0; i < FRIC_NUM; i++)
    {
      const auto STATE = fric_motor_[i]->Update();
      if (STATE != LibXR::ErrorCode::OK)
      {
        motor_state_ = STATE;
      }
      param_motor_fric_[i] = fric_motor_[i]->GetFeedback();
    }
    motor_trig_->Update();
    param_trig_ = motor_trig_->GetFeedback();

    // 仅在电机使能（反馈状态为 1）时信任角度；首次有效时直接取绝对角，之后累加增量。
    if (!TrigFeedbackOk())
    {
      trig_angle_valid_ = false;
      return;
    }
    if (!trig_angle_valid_)
    {
      trig_angle_ =
          static_cast<float>(LibXR::CycleValue<float>(param_trig_.abs_angle)) /
          trig_gear_ratio_;
      trig_setpoint_angle_ = trig_angle_;
      setpoint_on_grid_ = false;
      trig_angle_pid_.Reset();
      trig_speed_pid_.Reset();
      trig_angle_valid_ = true;
      return;
    }
    const float DELTA_MOTOR_ANGLE =
        LibXR::CycleValue<float>(param_trig_.abs_angle) - LAST_TRIG_MOTOR_ANGLE;
    this->trig_angle_ += DELTA_MOTOR_ANGLE / trig_gear_ratio_;
  }

  /** @brief 拨弹电机已使能且无错误。 */
  bool TrigFeedbackOk() const { return param_trig_.state == DM_STATE_ENABLED; }

  /** @brief 每格角度 (rad)。 */
  float TrigStep() const
  {
    return static_cast<float>(LibXR::TWO_PI) / static_cast<float>(num_trig_tooth_);
  }

  /**
   * @brief 计算下一发的目标角。前进方向为角度减小。
   *        距某待发位不超过 trig_ready_tolerance_ 时视为已就位，目标为该待发位再前进一格；
   *        否则目标为前进方向上的下一个待发位。
   */
  float NextReadyTarget() const
  {
    const float STEP = TrigStep();
    // phase ∈ [0, STEP)：当前角在其前进方向最近待发位之后多远
    float phase = std::fmod(trig_angle_ - trig_ready_angle_, STEP);
    if (phase < 0.0f)
    {
      phase += STEP;
    }
    if (phase <= trig_ready_tolerance_)
    {
      return trig_angle_ - phase - STEP;  // 稍未到位
    }
    if (phase >= STEP - trig_ready_tolerance_)
    {
      return trig_angle_ + (STEP - phase) - STEP;  // 稍越过待发位
    }
    return trig_angle_ - phase;  // 位于两格之间
  }

  /**
   * @brief 更新摩擦轮目标，再执行软启动、热量计算与拨弹状态机。
   *        Update wheel targets, then run soft start, heat and trigger state logic.
   */
  void Solve()
  {
    UpdateFricTarget();
    SoftStart();
    HeatLimit();
    UpdateTrigMode();
  }

  /**
   * @brief 执行电机输出；台架空载时禁止拨弹，其他反馈错误时复位。
   *        Run motor outputs; disable feeder in empty-bench mode and reset on faults.
   */
  void Control()
  {
    if (fric_only_test_)
    {
      if (motor_state_ == LibXR::ErrorCode::OK)
      {
        // 保留拨弹电机的 CAN 组帧槽位，但禁止拨弹输出。
        motor_trig_->Relax();
        FricPidControl();
      }
      else
      {
        Reset();
      }
      return;
    }

    if (motor_state_ == LibXR::ErrorCode::OK)
    {
      if (trig_angle_valid_)
      {
        NormalFireControl();
      }
      else
      {
        // 拨弹电机未使能或报错：禁止发射，摩擦轮照常运行。
        fire_flag_ = false;
        enable_fire_ = false;
      }
      real_launch_delay_ = (finish_fire_time_ - start_fire_time_).ToMillisecond();
      FricPidControl();
      TrigPidControl();
    }
    else
    {
      Reset();
    }
  }

  /**
   * @brief 设置摩擦轮模式。
   *        Set the friction wheel mode.
   *
   * @param mode 事件 ID，对应 `LauncherEvent`。
   *             Event ID, corresponding to `LauncherEvent`.
   */
  void SetMode(uint32_t mode)
  {
    LauncherEvent new_state = static_cast<LauncherEvent>(mode);
    if (new_state != launcher_state_ && (new_state == LauncherEvent::SET_FRICMODE_RELAX ||
                                         new_state == LauncherEvent::SET_FRICMODE_READY))
    {
      ResetFricControl();
    }
    if (new_state == LauncherEvent::SET_FRICMODE_SAFE &&
        launcher_state_ != LauncherEvent::SET_FRICMODE_SAFE)
    {
      trig_setpoint_angle_ = trig_angle_;
      setpoint_on_grid_ = false;
      fire_flag_ = false;
      enable_fire_ = false;
      mark_launch_ = true;
      for (int i = 0; i < FRIC_NUM; i++)
      {
        fric_target_rpm_[i] = param_motor_fric_[i].velocity;
      }
    }
    launcher_state_ = new_state;
  }

  /**
   * @brief 失去控制时复位全部发射状态并切换到 `SET_FRICMODE_RELAX`。
   *        Reset all launch states and switch to `SET_FRICMODE_RELAX` when control is
   *        lost.
   */
  void LostCtrl()
  {
    // 重置所有发射相关的状态变量到初始模式
    launcher_state_ = LauncherEvent::SET_FRICMODE_RELAX;
    trig_mode_ = TrigMode::RELAX;

    // 重置发射控制标志
    fire_flag_ = false;
    enable_fire_ = false;
    mark_launch_ = false;
    press_continue_ = false;

    // 重置计数器
    fired_ = 0;

    // 重置时间戳
    fire_press_time_ = 0;
    start_fire_time_ = 0;
    finish_fire_time_ = 0;

    // 角度在下次使能时由绝对编码器重新初始化
    trig_angle_valid_ = false;
    setpoint_on_grid_ = false;

    trig_output_ = 0.0f;

    // 重置速度目标值
    for (float& target : fric_target_rpm_)
    {
      target = 0.0f;
    }

    // 重置发射命令
    launcher_cmd_.isfire = false;
    last_fire_notify_ = false;

    // 重置延迟计算
    real_launch_delay_ = 0.0f;

    ResetFricControl();
  }

  /**
   * @brief 复位摩擦轮 PID、软启动限幅及就绪计时。
   */
  void ResetFricControl()
  {
    for (int i = 0; i < FRIC_NUM; i++)
    {
      fric_speed_pid_[i].Reset();
      fric_speed_pid_[i].SetOutLimit(SOFT_START_INITIAL_OUT_LIMIT);
      cmd_fric_[i].velocity = 0.0f;
    }
    soft_start_finish_ = false;
    soft_start_out_limit_ = SOFT_START_INITIAL_OUT_LIMIT;
    fric_ready_time_ = 0;
  }

  bool FricSpeedsReady()
  {
    if (launcher_state_ != LauncherEvent::SET_FRICMODE_READY)
    {
      return false;
    }
    for (int i = 0; i < FRIC_NUM; i++)
    {
      if (!std::isfinite(param_motor_fric_[i].velocity) ||
          !std::isfinite(fric_target_rpm_[i]) ||
          param_motor_fric_[i].velocity * fric_target_rpm_[i] <= 0.0f ||
          std::abs(param_motor_fric_[i].velocity) <
              std::abs(fric_target_rpm_[i]) * FRIC_READY_SPEED_RATIO)
      {
        return false;
      }
    }
    return true;
  }

  bool FricReady() { return soft_start_finish_ && FricSpeedsReady(); }

  static float RampToZero(float value, float step)
  {
    if (value > step)
    {
      return value - step;
    }
    if (value < -step)
    {
      return value + step;
    }
    return 0.0f;
  }

  /** @brief 复位发射状态并放松全部电机。 */
  void Reset()
  {
    fire_flag_ = false;
    enable_fire_ = false;
    mark_launch_ = false;
    press_continue_ = false;

    fired_ = 0;

    start_fire_time_ = 0;
    finish_fire_time_ = 0;

    launcher_cmd_.isfire = false;
    last_fire_notify_ = false;

    // Relax() 会使 DM 电机失能，角度在重新使能后由绝对编码器重新初始化
    trig_angle_valid_ = false;
    setpoint_on_grid_ = false;

    ResetFricControl();

    for (int i = 0; i < FRIC_NUM; i++)
    {
      fric_motor_[i]->Relax();
    }
    motor_trig_->Relax();
  }

  /**
   * @brief 按发射命令与摩擦轮模式更新拨弹模式。
   *        Update the trigger mode from the fire command and the friction wheel mode.
   */
  void UpdateTrigMode()
  {
    LibXR::MillisecondTimestamp now_time = LibXR::Timebase::GetMilliseconds();

    if (launcher_state_ == LauncherEvent::SET_FRICMODE_READY)
    {
      if (launcher_cmd_.isfire && !last_fire_notify_)
      {
        fire_press_time_ = now_time;
        press_continue_ = false;
        trig_mode_ = TrigMode::SINGLE;
      }
      else if (launcher_cmd_.isfire && last_fire_notify_)
      {
        if (!press_continue_ && (now_time - fire_press_time_ > 200))
        {
          press_continue_ = true;
        }
        if (press_continue_)
        {
          trig_mode_ = TrigMode::CONTINUE;
        }
      }
      else
      {
        trig_mode_ = TrigMode::SAFE;
        press_continue_ = false;
      }
    }
    else if (launcher_state_ == LauncherEvent::SET_FRICMODE_SAFE)
    {
      trig_mode_ = TrigMode::SAFE;
      press_continue_ = false;
    }
    else
    {
      trig_mode_ = TrigMode::RELAX;
      press_continue_ = false;
    }

    last_fire_notify_ = launcher_cmd_.isfire;
  }

  /**
   * @brief 按摩擦轮模式设置摩擦轮目标转速。
   *        Set the friction wheel target speeds from the friction wheel mode.
   */
  void UpdateFricTarget()
  {
    switch (launcher_state_)
    {
      case LauncherEvent::SET_FRICMODE_RELAX:
        for (float& target : fric_target_rpm_)
        {
          target = 0.0f;
        }
        soft_start_finish_ = false;
        Reset();
        break;
      case LauncherEvent::SET_FRICMODE_SAFE:
        for (int i = 0; i < FRIC_NUM; i++)
        {
          fric_target_rpm_[i] =
              RampToZero(fric_target_rpm_[i], FRIC_SAFE_DECELERATION_RPMPS * dt_);
        }
        break;
      case LauncherEvent::SET_FRICMODE_READY:
        fric_target_rpm_[0] = param_fric_target_speed_[0];
        fric_target_rpm_[1] = param_fric_target_speed_[0];
        fric_target_rpm_[2] = param_fric_target_speed_[1];
        break;
      default:
        break;
    }
  }

  /**
   * @brief 常规发弹：热量允许时推进到下一个待发位并检测出弹。
   *        设定角已在待发位网格上时直接前进一格；否则（上电、SAFE 后）按当前绝对角
   *        求下一个待发位，因此无需首发标定。
   *        Normal firing: advance to the next ready position when heat allows and
   *        detect the round. When the setpoint is on the ready grid it advances one
   *        tooth; otherwise (after power-up or SAFE) the next ready position is derived
   *        from the absolute angle, so no first-shot calibration is needed.
   */
  void NormalFireControl()
  {
    if (trig_mode_ == TrigMode::SINGLE)
    {
      if (!enable_fire_)
      {
        if (heat_ctrl_.available_shot && FricReady())
        {
          trig_setpoint_angle_ = setpoint_on_grid_
                                     ? trig_setpoint_angle_ - TrigStep()
                                     : NextReadyTarget();
          setpoint_on_grid_ = true;

          enable_fire_ = true;
          mark_launch_ = false;
          start_fire_time_ = LibXR::Timebase::GetMilliseconds();

          trig_mode_ = TrigMode::SAFE;
        }
      }
    }

    // 发射超时检测：超过 100 ms 未检测到出弹则重置状态
    if (start_fire_time_ > 0 && (now_ - start_fire_time_ > 100) && !mark_launch_)
    {
      fire_flag_ = false;
      enable_fire_ = false;
      start_fire_time_ = 0;
    }

    if (enable_fire_ && !mark_launch_)
    {  // 发弹状态检测
      if (std::abs(param_motor_fric_[2].torque) > 0.05)
      {
        fire_flag_ = false;

        fired_++;

        mark_launch_ = true;
        enable_fire_ = false;
        finish_fire_time_ = LibXR::Timebase::GetMilliseconds();
      }
    }
  }

  /**
   * @brief 计算并下发三路独立速度环输出；RELAX 始终发送零电流。
   */
  void FricPidControl()
  {
    if (launcher_state_ == LauncherEvent::SET_FRICMODE_RELAX)
    {
      // RELAX 持续发送零电流，禁止 PID 覆盖停机指令。
      for (int i = 0; i < FRIC_NUM; i++)
      {
        cmd_fric_[i].velocity = 0.0f;
        fric_motor_[i]->Relax();
      }
      return;
    }

    for (int i = 0; i < FRIC_NUM; i++)
    {
      cmd_fric_[i].velocity = fric_speed_pid_[i].Calculate(
          fric_target_rpm_[i], param_motor_fric_[i].velocity, dt_);
    }

    for (int i = 0; i < FRIC_NUM; i++)
    {
      fric_motor_[i]->Control(cmd_fric_[i]);
    }
  }

  /**
   * @brief 计算并下发拨弹角度环与速度环输出（输出轴力矩，MODE_TORQUE）。
   *        电机失能时发送使能帧，报错时发送清错帧，之后才下发力矩。
   *        Compute and send the trigger angle-loop and speed-loop outputs (output-shaft
   *        torque, MODE_TORQUE). A disabled motor gets an enable frame and a faulted
   *        motor a clear-error frame before any torque is sent.
   */
  void TrigPidControl()
  {
    if (trig_mode_ == TrigMode::RELAX)
    {
      trig_output_ = 0.0f;
      motor_trig_->Relax();
      return;
    }
    if (param_trig_.state == DM_STATE_DISABLED)
    {
      motor_trig_->Enable();
      return;
    }
    if (!TrigFeedbackOk())
    {
      motor_trig_->ClearError();
      return;
    }
    if (!trig_angle_valid_)
    {
      return;
    }

    trig_setpoint_speed_ =
        trig_angle_pid_.Calculate(trig_setpoint_angle_, trig_angle_, dt_);
    trig_output_ = trig_speed_pid_.Calculate(
        trig_setpoint_speed_, param_trig_.omega / trig_gear_ratio_, dt_);
    cmd_trig_.torque = trig_output_;
    motor_trig_->Control(cmd_trig_);
  }

  /**
   * @brief 摩擦轮软启动：转速达到目标前限制 PID 输出，达到后解除限制。
   *        Friction wheel soft start: limit the PID output until the speed reaches the
   *        target, then release the limit.
   */
  void SoftStart()
  {
    if (launcher_state_ != LauncherEvent::SET_FRICMODE_READY)
    {
      soft_start_finish_ = false;
      soft_start_out_limit_ = SOFT_START_INITIAL_OUT_LIMIT;
      fric_ready_time_ = 0;
      return;
    }

    if (soft_start_finish_)
    {
      for (LibXR::PID<float>& i : fric_speed_pid_)
      {
        i.SetOutLimit(1.0f);
      }
      return;
    }

    soft_start_out_limit_ =
        std::min(soft_start_out_limit_ + SOFT_START_OUT_LIMIT_STEP, 1.0f);
    for (LibXR::PID<float>& i : fric_speed_pid_)
    {
      i.SetOutLimit(soft_start_out_limit_);
    }

    const bool ALL_FRIC_READY = FricSpeedsReady();

    LibXR::MillisecondTimestamp now_time = LibXR::Timebase::GetMilliseconds();
    if (!ALL_FRIC_READY)
    {
      fric_ready_time_ = 0;
    }
    else if (fric_ready_time_ == 0)
    {
      fric_ready_time_ = now_time;
    }
    else if (now_time - fric_ready_time_ >= FRIC_READY_HOLD_TIME_MS)
    {
      soft_start_finish_ = true;
    }
  }

  /**
   * @brief 计算热量并更新可发射数量。
   *        Compute the heat and update the number of available shots.
   */
  void HeatLimit()
  {
    if (bench_ignore_heat_limit_)
    {
      heat_ctrl_.available_shot = 1U;
      fired_ = 0;
      return;
    }
    heat_ctrl_.heat_limit = ref_data_.rs.shooter_heat_limit;
    heat_ctrl_.heat_increase = 100.0f;
    heat_ctrl_.cooling_rate = ref_data_.rs.shooter_cooling_value;
    if (fired_ >= 1)
    {
      heat_ctrl_.heat += heat_ctrl_.heat_increase;
      fired_ = 0;
    }
    heat_ctrl_.heat -= heat_ctrl_.cooling_rate * dt_;  // 每个控制周期的冷却恢复
    heat_ctrl_.heat = std::max(heat_ctrl_.heat, 0.0f);
    float available_float = std::max(
        (heat_ctrl_.heat_limit - heat_ctrl_.heat) / heat_ctrl_.heat_increase, 0.0f);
    heat_ctrl_.available_shot = static_cast<uint32_t>(available_float);
  }

  CMD::LauncherCMD launcher_cmd_{};  // NOLINT
  Referee::LauncherPack ref_data_{};

  /**
   * @brief 绘制一步裁判系统 UI，五步循环。
   *        Draw one step of the referee system UI, in a cycle of five steps.
   */
  void DrawUI()
  {
    uint16_t robot_id = ref_->GetRobotID();
    uint16_t client_id = ref_->GetClientID(robot_id);

    // 首次绘制使用ADD，后续使用MODIFY
    Referee::UIFigureOp ADD_OP = Referee::UIFigureOp::UI_OP_MODIFY;
    if (this->ui_tick_ % 5 == 0)
    {
      ADD_OP = Referee::UIFigureOp::UI_OP_ADD;
    }

    // 摩擦轮颜色：前两路转速大于 3500 rpm、第三路大于 2200 rpm 为青色，否则为橙色
    auto fric_color_0 = (fabsf(param_motor_fric_[0].velocity) > 3500.0f)
                            ? Referee::UIColor::UI_COLOR_CYAN
                            : Referee::UIColor::UI_COLOR_ORANGE;
    auto fric_color_1 = (fabsf(param_motor_fric_[1].velocity) > 3500.0f)
                            ? Referee::UIColor::UI_COLOR_CYAN
                            : Referee::UIColor::UI_COLOR_ORANGE;
    auto fric_color_2 = (fabsf(param_motor_fric_[2].velocity) > 2200.0f)
                            ? Referee::UIColor::UI_COLOR_CYAN
                            : Referee::UIColor::UI_COLOR_ORANGE;

    // 拨弹盘位置计算
    float trig_pos = LibXR::CycleValue<float>(-this->trig_angle_) / 6.2832f * 360.0f;
    uint16_t arc_mid = static_cast<uint16_t>(trig_pos);
    uint16_t arc_start = static_cast<uint16_t>((arc_mid + 330) % 360);
    uint16_t arc_end = static_cast<uint16_t>((arc_mid + 30) % 360);
    if (arc_start >= arc_end) arc_end += 360;

    // 拨弹盘颜色：根据模式判断（校准模式为橙色，其他为青色）
    auto trig_color = trig_angle_valid_ ? Referee::UIColor::UI_COLOR_CYAN
                                        : Referee::UIColor::UI_COLOR_ORANGE;

    switch (ui_step_)
    {
      case 0:
      {
        // 绘制前摩擦轮状态
        Referee::UIFigure2 fig_front{};
        ref_->FillCircle(fig_front.interaction_figure[0], "lfl", ADD_OP,
                         UI_HERO_LAUNCHER_LAYER, fric_color_0, 4, 1600, 820, 25);
        ref_->FillCircle(fig_front.interaction_figure[1], "lfr", ADD_OP,
                         UI_HERO_LAUNCHER_LAYER, fric_color_1, 4, 1680, 820, 25);
        ref_->SendUIFigure2(robot_id, client_id, fig_front);
        this->ui_tick_ += 1;
        break;
      }
      case 1:
      {
        // 绘制第三轮并清理旧第四轮图形。
        Referee::UIFigure2 fig_back{};
        ref_->FillCircle(fig_back.interaction_figure[0], "lbl", ADD_OP,
                         UI_HERO_LAUNCHER_LAYER, fric_color_2, 4, 1600, 750, 25);
        ref_->FillCircle(fig_back.interaction_figure[1], "lbr",
                         Referee::UIFigureOp::UI_OP_DELETE, UI_HERO_LAUNCHER_LAYER,
                         Referee::UIColor::UI_COLOR_ORANGE, 4, 1680, 750, 25);
        ref_->SendUIFigure2(robot_id, client_id, fig_back);
        break;
      }
      case 2:
      {
        // 绘制拨弹盘位置
        Referee::UIFigure fig_trig{};
        ref_->FillArc(fig_trig, "lta", ADD_OP, UI_HERO_LAUNCHER_LAYER, trig_color, 3,
                      1640, 640, arc_start, arc_end, 80, 80);
        ref_->SendUIFigure(robot_id, client_id, fig_trig);
        break;
      }
      case 3:
      {
        // 竖直画线 - 主瞄准线
        Referee::UIFigure fig_line{};
        ref_->FillLine(fig_line, "ctrln", ADD_OP, UI_HERO_LAUNCHER_LAYER,
                       Referee::UIColor::UI_COLOR_WHITE, 2, 960, 150, 960, 900);
        ref_->SendUIFigure(robot_id, client_id, fig_line);
        break;
      }
      case 4:
      {
        // 水平瞄准线 - 下方第一条
        Referee::UIFigure fig_bot2{};
        ref_->FillLine(fig_bot2, "hlb1", ADD_OP, UI_HERO_LAUNCHER_LAYER,
                       Referee::UIColor::UI_COLOR_WHITE, 2, 940, 300, 980, 300);
        ref_->SendUIFigure(robot_id, client_id, fig_bot2);

        break;
      }
      default:
        break;
    }
    this->ui_step_ = (this->ui_step_ + 1) % 5;  // 5 步循环
  }
};
