#pragma once

/**
 * @file WebotsFireNotify.hpp
 * @brief Webots 发射机构仿真模块。
 */

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Webots launcher simulator with fire-rate, fire-delay and heat limits
constructor_args:
  - bullet_speed: 23.0
  - single_shot_heat: 10.0
  - shooter_heat_limit: 240.0
  - shooter_cooling_value: 40.0
  - max_fire_frequency_hz: 20.0
  - fire_delay_ms: 30.0
  - state_publish_period_ms: 10
template_args: []
required_hardware: []
depends:
  - qdu-future/WebotsReferee
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <webots/LED.hpp>
#include <webots/Robot.hpp>

#include "WebotsRefereeTypes.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "timebase.hpp"
#include "timer.hpp"

extern webots::Robot *_libxr_webots_robot_handle;

/**
 * @brief DevC LauncherCMD 接收的发射请求载荷。
 */
struct WebotsHostFireNotify
{
  bool isfire{false};  ///< 是否请求发射。
};

static_assert(sizeof(WebotsHostFireNotify) == 1);

/**
 * @brief Webots 发射机构。
 *
 * 本模块把 `host/fire_notify` 当作开火请求，而不是已发弹事实。请求通过射频、
 * 热量和延迟检查后，才发布 `webots_launcher/shot_event`，并用
 * `webots_launcher/state` 给 WebotsReferee 同步当前热量与射频。
 */
class WebotsFireNotify : public LibXR::Application
{
 public:
  /**
   * @brief 构造 Webots 发射机构仿真。
   *
   * @param bullet_speed 弹丸初速度，单位 m/s。
   * @param single_shot_heat 单发增加热量。
   * @param shooter_heat_limit 热量上限；小于等于 0 时不启用热量拒绝。
   * @param shooter_cooling_value 每秒热量恢复值。
   * @param max_fire_frequency_hz 最高射频；小于等于 0 时不启用射频拒绝。
   * @param fire_delay_ms 请求到真实出弹的延迟，单位 ms。
   * @param state_publish_period_ms 状态发布周期，单位 ms。
   */
  WebotsFireNotify(LibXR::HardwareContainer &, LibXR::ApplicationManager &app,
                   float bullet_speed = 23.0f, float single_shot_heat = 10.0f,
                   float shooter_heat_limit = 240.0f,
                   float shooter_cooling_value = 40.0f,
                   float max_fire_frequency_hz = 20.0f,
                   float fire_delay_ms = 30.0f,
                   int state_publish_period_ms = 10)
      : bullet_speed_(NonNegativeOrZero(bullet_speed)),
        single_shot_heat_(NonNegativeOrZero(single_shot_heat)),
        heat_limit_(NonNegativeOrZero(shooter_heat_limit)),
        cooling_rate_(NonNegativeOrZero(shooter_cooling_value)),
        max_fire_frequency_hz_(NonNegativeOrZero(max_fire_frequency_hz)),
        fire_delay_us_(MillisecondsToMicroseconds(fire_delay_ms)),
        min_fire_interval_us_(FrequencyToIntervalMicroseconds(max_fire_frequency_hz_))
  {
    led_ = _libxr_webots_robot_handle->getLED("fire_led");

    const uint64_t now = NowUs();
    last_update_time_us_ = now;

    auto cb = LibXR::Topic::Callback::Create(
        [](bool, WebotsFireNotify *self, LibXR::RawData &data)
        {
          auto *msg = reinterpret_cast<WebotsHostFireNotify *>(data.addr_);
          if (msg != nullptr && data.size_ == sizeof(WebotsHostFireNotify))
          {
            self->HandleFireRequest(msg->isfire);
          }
        },
        this);

    fire_notify_topic_.RegisterCallback(cb);

    auto timer_handle = LibXR::Timer::CreateTask<WebotsFireNotify *>(
        [](WebotsFireNotify *self)
        { self->Tick(); }, this,
        static_cast<uint32_t>(std::max(1, state_publish_period_ms)));

    LibXR::Timer::Add(timer_handle);
    LibXR::Timer::Start(timer_handle);

    WebotsRefereeTypes::WebotsLauncherState initial_state{};
    {
      LibXR::Mutex::LockGuard lock(state_mutex_);
      initial_state = BuildStateLocked(now);
      UpdateLedLocked(now);
    }
    state_topic_.Publish(initial_state);

    app.Register(*this);
  }

  /**
   * @brief 周期监控入口；发射机构由 topic 回调和 Timer 驱动。
   */
  void OnMonitor() override {}

 private:
  /**
   * @brief 处理一次开火请求。
   */
  void HandleFireRequest(bool request)
  {
    const uint64_t now = NowUs();
    bool has_shot_event = false;
    bool has_state = false;
    WebotsRefereeTypes::WebotsLauncherShotEvent shot_event{};
    WebotsRefereeTypes::WebotsLauncherState state{};

    {
      LibXR::Mutex::LockGuard lock(state_mutex_);

      CoolHeatLocked(now);

      if (!request)
      {
        UpdateLedLocked(now);
        return;
      }

      last_request_time_us_ = now;
      request_led_until_us_ = now + REQUEST_LED_HOLD_US;

      const auto reject_reason = CheckRejectReasonLocked(now);
      if (reject_reason != WebotsRefereeTypes::WebotsLauncherRejectReason::NONE)
      {
        last_reject_reason_ = reject_reason;
        state = BuildStateLocked(now);
        has_state = true;
        UpdateLedLocked(now);
      }
      else
      {
        pending_fire_ = true;
        request_time_us_ = now;
        pending_fire_time_us_ = now + fire_delay_us_;
        next_fire_request_us_ = now + min_fire_interval_us_;
        last_reject_reason_ =
            WebotsRefereeTypes::WebotsLauncherRejectReason::NONE;

        has_shot_event = ProcessPendingFireLocked(now, shot_event);
        state = BuildStateLocked(now);
        has_state = true;
        UpdateLedLocked(now);
      }
    }

    if (has_shot_event)
    {
      shot_event_topic_.Publish(shot_event);
    }
    if (has_state)
    {
      state_topic_.Publish(state);
    }
  }

  /**
   * @brief 定时推进冷却、延迟出弹和状态发布。
   */
  void Tick()
  {
    const uint64_t now = NowUs();
    bool has_shot_event = false;
    WebotsRefereeTypes::WebotsLauncherShotEvent shot_event{};
    WebotsRefereeTypes::WebotsLauncherState state{};

    {
      LibXR::Mutex::LockGuard lock(state_mutex_);
      CoolHeatLocked(now);
      has_shot_event = ProcessPendingFireLocked(now, shot_event);
      state = BuildStateLocked(now);
      UpdateLedLocked(now);
    }

    if (has_shot_event)
    {
      shot_event_topic_.Publish(shot_event);
    }
    state_topic_.Publish(state);
  }

  /**
   * @brief 按经过时间恢复热量。
   */
  void CoolHeatLocked(uint64_t now)
  {
    if (now <= last_update_time_us_)
    {
      return;
    }

    const float dt_s =
        static_cast<float>(now - last_update_time_us_) / MICROSECONDS_PER_SECOND_F;
    current_heat_ = std::max(0.0f, current_heat_ - cooling_rate_ * dt_s);
    last_update_time_us_ = now;
  }

  /**
   * @brief 到达发弹时刻后构造真实发弹事件。
   */
  bool ProcessPendingFireLocked(
      uint64_t now, WebotsRefereeTypes::WebotsLauncherShotEvent &event)
  {
    if (!pending_fire_ || now < pending_fire_time_us_)
    {
      return false;
    }

    const float heat_before = current_heat_;
    current_heat_ += single_shot_heat_;

    const uint64_t previous_fire_time = last_fire_time_us_;
    const uint64_t shot_interval_us =
        previous_fire_time == 0U ? 0U : now - previous_fire_time;

    last_fire_time_us_ = now;
    shot_count_++;
    pending_fire_ = false;
    pending_fire_time_us_ = 0;
    shot_led_until_us_ = now + SHOT_LED_HOLD_US;

    if (shot_interval_us > 0U)
    {
      current_fire_frequency_hz_ =
          MICROSECONDS_PER_SECOND_F / static_cast<float>(shot_interval_us);
    }
    else
    {
      current_fire_frequency_hz_ = 0.0f;
    }

    event.shot_id = shot_count_;
    event.request_time_us = request_time_us_;
    event.fire_time_us = now;
    event.shot_interval_us = shot_interval_us;
    event.bullet_speed = bullet_speed_;
    event.heat_before = heat_before;
    event.heat_after = current_heat_;
    event.heat_limit = heat_limit_;
    event.cooling_rate = cooling_rate_;
    event.single_shot_heat = single_shot_heat_;
    event.fire_delay_s =
        static_cast<float>(fire_delay_us_) / MICROSECONDS_PER_SECOND_F;
    event.min_fire_interval_s =
        static_cast<float>(min_fire_interval_us_) / MICROSECONDS_PER_SECOND_F;

    return true;
  }

  /**
   * @brief 计算当前请求是否应被拒绝。
   */
  WebotsRefereeTypes::WebotsLauncherRejectReason CheckRejectReasonLocked(
      uint64_t now) const
  {
    using WebotsRefereeTypes::WebotsLauncherRejectReason;

    if (!launcher_enabled_)
    {
      return WebotsLauncherRejectReason::DISABLED;
    }

    if (pending_fire_)
    {
      return WebotsLauncherRejectReason::PENDING;
    }

    if (now < next_fire_request_us_)
    {
      return WebotsLauncherRejectReason::RATE_LIMIT;
    }

    if (heat_limit_ > 0.0f && current_heat_ + single_shot_heat_ >= heat_limit_)
    {
      return WebotsLauncherRejectReason::HEAT_LIMIT;
    }

    return WebotsLauncherRejectReason::NONE;
  }

  /**
   * @brief 构造发射机构状态。
   */
  WebotsRefereeTypes::WebotsLauncherState BuildStateLocked(uint64_t now)
  {
    UpdateReportedFrequencyLocked(now);

    WebotsRefereeTypes::WebotsLauncherState state{};
    state.update_time_us = now;
    state.last_request_time_us = last_request_time_us_;
    state.last_fire_time_us = last_fire_time_us_;
    state.next_fire_request_us = next_fire_request_us_;
    state.pending_fire_time_us = pending_fire_time_us_;
    state.shot_count = shot_count_;
    state.current_heat = current_heat_;
    state.heat_limit = heat_limit_;
    state.cooling_rate = cooling_rate_;
    state.single_shot_heat = single_shot_heat_;
    state.bullet_speed = bullet_speed_;
    state.max_fire_frequency_hz = max_fire_frequency_hz_;
    state.fire_delay_s =
        static_cast<float>(fire_delay_us_) / MICROSECONDS_PER_SECOND_F;
    state.min_fire_interval_s =
        static_cast<float>(min_fire_interval_us_) / MICROSECONDS_PER_SECOND_F;
    state.current_fire_frequency_hz = current_fire_frequency_hz_;
    state.launcher_enabled = launcher_enabled_ ? 1U : 0U;
    state.can_fire =
        CheckRejectReasonLocked(now) ==
                WebotsRefereeTypes::WebotsLauncherRejectReason::NONE
            ? 1U
            : 0U;
    state.pending_fire = pending_fire_ ? 1U : 0U;
    state.last_reject_reason = static_cast<uint8_t>(last_reject_reason_);

    return state;
  }

  /**
   * @brief 无近期发弹时把上报射频恢复为 0。
   */
  void UpdateReportedFrequencyLocked(uint64_t now)
  {
    if (last_fire_time_us_ == 0U ||
        now - last_fire_time_us_ > FREQUENCY_REPORT_HOLD_US)
    {
      current_fire_frequency_hz_ = 0.0f;
    }
  }

  /**
   * @brief 更新 Webots LED，仅作请求和真实出弹的视觉提示。
   */
  void UpdateLedLocked(uint64_t now)
  {
    if (led_ == nullptr)
    {
      return;
    }

    const bool led_on =
        pending_fire_ || now < request_led_until_us_ || now < shot_led_until_us_;
    led_->set(led_on ? 255 : 0);
  }

  /**
   * @brief 获取当前 libxr 时间，单位 us。
   */
  static uint64_t NowUs()
  {
    return static_cast<uint64_t>(LibXR::Timebase::GetMicroseconds());
  }

  /**
   * @brief 非有限值和负数统一按 0 处理。
   */
  static float NonNegativeOrZero(float value)
  {
    return std::isfinite(value) ? std::max(0.0f, value) : 0.0f;
  }

  /**
   * @brief ms 转 us。
   */
  static uint64_t MillisecondsToMicroseconds(float milliseconds)
  {
    const float safe_ms = NonNegativeOrZero(milliseconds);
    return static_cast<uint64_t>(std::lround(safe_ms * 1000.0f));
  }

  /**
   * @brief 射频上限转最小发射间隔。
   */
  static uint64_t FrequencyToIntervalMicroseconds(float frequency_hz)
  {
    if (!std::isfinite(frequency_hz) || frequency_hz <= 0.0f)
    {
      return 0;
    }

    const auto interval_us = static_cast<uint64_t>(
        std::lround(MICROSECONDS_PER_SECOND_D / frequency_hz));
    return std::max<uint64_t>(1, interval_us);
  }

  static constexpr float MICROSECONDS_PER_SECOND_F = 1000000.0f;
  static constexpr double MICROSECONDS_PER_SECOND_D = 1000000.0;
  static constexpr uint64_t REQUEST_LED_HOLD_US = 20000;
  static constexpr uint64_t SHOT_LED_HOLD_US = 80000;
  static constexpr uint64_t FREQUENCY_REPORT_HOLD_US = 1000000;

  webots::LED *led_{nullptr};

  const float bullet_speed_{0.0f};
  const float single_shot_heat_{0.0f};
  const float heat_limit_{0.0f};
  const float cooling_rate_{0.0f};
  const float max_fire_frequency_hz_{0.0f};
  const uint64_t fire_delay_us_{0};
  const uint64_t min_fire_interval_us_{0};

  bool launcher_enabled_{true};
  bool pending_fire_{false};
  uint64_t request_time_us_{0};
  uint64_t last_request_time_us_{0};
  uint64_t last_fire_time_us_{0};
  uint64_t next_fire_request_us_{0};
  uint64_t pending_fire_time_us_{0};
  uint64_t request_led_until_us_{0};
  uint64_t shot_led_until_us_{0};
  uint64_t last_update_time_us_{0};
  uint64_t shot_count_{0};
  float current_heat_{0.0f};
  float current_fire_frequency_hz_{0.0f};
  WebotsRefereeTypes::WebotsLauncherRejectReason last_reject_reason_{
      WebotsRefereeTypes::WebotsLauncherRejectReason::NONE};

  LibXR::Mutex state_mutex_;
  LibXR::Topic::Domain host_domain_ = LibXR::Topic::Domain("host");
  LibXR::Topic fire_notify_topic_ =
      LibXR::Topic("fire_notify", sizeof(WebotsHostFireNotify), &host_domain_);
  LibXR::Topic::Domain launcher_domain_ = LibXR::Topic::Domain("webots_launcher");
  LibXR::Topic state_topic_ =
      LibXR::Topic("state", sizeof(WebotsRefereeTypes::WebotsLauncherState),
                   &launcher_domain_, true);
  LibXR::Topic shot_event_topic_ =
      LibXR::Topic("shot_event", sizeof(WebotsRefereeTypes::WebotsLauncherShotEvent),
                   &launcher_domain_, true);
};
