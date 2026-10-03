# WebotsFireNotify

Webots 发射机构仿真模块：按射频、发弹延迟与热量限制把开火请求转换为出弹事件 / Webots launcher simulation Module that turns fire requests into shot events under fire-rate, fire-delay and heat limits

## 1. 模块作用 / Purpose

WebotsFireNotify 订阅 `host/fire_notify`，把消息视为开火请求，并在射频、发弹延迟和热量限制通过后发布真实出弹事件。Webots `fire_led` 在请求、待发和真实出弹时短暂点亮，作为视觉提示；world 中存在该 LED 时生效。

发射规则：

- `host/fire_notify` 表示开火请求。
- 同一时刻只有一发处于发弹延迟队列，期间的新请求以 `PENDING` 拒绝。
- `max_fire_frequency_hz` 转换为最小请求间隔，距离上次被接受的请求不足该间隔时以 `RATE_LIMIT` 拒绝。
- `shooter_heat_limit > 0` 时，若本次发射会使热量达到或超过上限，以 `HEAT_LIMIT` 拒绝。
- `fire_delay_ms` 到期后（由周期定时任务推进）发布 `shot_event`，并在此刻增加热量。
- `shooter_cooling_value` 按秒恢复热量，热量下限为 0。
- 负数或非有限的浮点配置按 0 处理。

`WebotsReferee` 订阅 `webots_launcher` 域的两个 Topic，把热量上限和冷却值同步到 `robot_game_ref` 裁判摘要。

WebotsFireNotify subscribes to `host/fire_notify`, treats each message as a fire request, and publishes the actual shot event after the fire-rate, fire-delay and heat limits are passed. The Webots `fire_led` lights up briefly for a request, a pending shot and an actual shot as a visual hint, and takes effect when the world contains that LED.

Fire rules:

- `host/fire_notify` is a fire request.
- Only one shot is in the fire-delay queue at a time; new requests during that time are rejected with `PENDING`.
- `max_fire_frequency_hz` is converted into a minimum request interval; a request closer to the last accepted request than this interval is rejected with `RATE_LIMIT`.
- With `shooter_heat_limit > 0`, a shot that would bring the heat to or above the limit is rejected with `HEAT_LIMIT`.
- When `fire_delay_ms` expires (advanced by the periodic timer task), `shot_event` is published and the heat is increased at that moment.
- `shooter_cooling_value` recovers heat per second, with a lower bound of 0 for the heat.
- Negative or non-finite floating-point configuration values are treated as 0.

`WebotsReferee` subscribes to the two Topics of the `webots_launcher` domain and synchronizes the heat limit and the cooling value into the `robot_game_ref` referee summary.

## 2. 构造接口 / Constructor

```cpp
WebotsFireNotify(const Param& param = {.bullet_speed = 23.0f,
                                       .single_shot_heat = 10.0f,
                                       .shooter_heat_limit = 240.0f,
                                       .shooter_cooling_value = 40.0f,
                                       .max_fire_frequency_hz = 20.0f,
                                       .fire_delay_ms = 30.0f,
                                       .state_publish_period_ms = 10});
```

依赖：无。

配置参数（`Param`）：

- `bullet_speed`：弹丸初速度，单位 m/s，默认 `23.0`。
- `single_shot_heat`：单发增加的热量，默认 `10.0`。
- `shooter_heat_limit`：热量上限，默认 `240.0`；为 0 时不启用热量拒绝。
- `shooter_cooling_value`：每秒恢复的热量，默认 `40.0`。
- `max_fire_frequency_hz`：最大射频，单位 Hz，默认 `20.0`；为 0 时不启用射频拒绝。
- `fire_delay_ms`：请求到真实出弹的延迟，单位 ms，默认 `30.0`。
- `state_publish_period_ms`：状态发布与延迟推进的定时任务周期，单位 ms，默认 `10`，最小按 1 执行。

Dependencies: none.

Configuration parameters (`Param`):

- `bullet_speed`: projectile muzzle speed in m/s, default `23.0`.
- `single_shot_heat`: heat added by one shot, default `10.0`.
- `shooter_heat_limit`: heat limit, default `240.0`; with 0 the heat rejection is disabled.
- `shooter_cooling_value`: heat recovered per second, default `40.0`.
- `max_fire_frequency_hz`: maximum fire rate in Hz, default `20.0`; with 0 the rate rejection is disabled.
- `fire_delay_ms`: delay from request to the actual shot in ms, default `30.0`.
- `state_publish_period_ms`: period of the timer task that publishes the state and advances the delay, in ms, default `10`, at least 1 is used.

## 3. Topic

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `host/fire_notify` | 订阅 | 与 DevC `LauncherCMD{bool isfire}` 布局相同的 1 字节载荷 | `isfire = true` 为开火请求，`false` 的消息被忽略；构造时该 Topic 已存在（通常由 `QDU-Robomaster/Aimer` 创建），缺失时记录错误并抛出 `std::runtime_error` |
| `webots_launcher/state` | 发布 | `WebotsRefereeTypes::WebotsLauncherState` | 发射机构状态：热量、冷却、射频、延迟、待发状态、出弹计数和最近拒绝原因；构造时、每次处理开火请求时以及每 `state_publish_period_ms` 发布一次 |
| `webots_launcher/shot_event` | 发布 | `WebotsRefereeTypes::WebotsLauncherShotEvent` | 请求被接受并完成延迟后的出弹事件：请求与出弹时间（µs）、出弹间隔、弹速、出弹前后热量等 |

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `host/fire_notify` | Subscribe | 1-byte payload with the layout of DevC `LauncherCMD{bool isfire}` | `isfire = true` is a fire request and messages with `false` are ignored; the Topic exists at construction (usually created by `QDU-Robomaster/Aimer`), and a missing Topic is logged and throws `std::runtime_error` |
| `webots_launcher/state` | Publish | `WebotsRefereeTypes::WebotsLauncherState` | Launcher state: heat, cooling, fire rate, delay, pending state, shot count and the latest reject reason; published at construction, on every processed fire request and every `state_publish_period_ms` |
| `webots_launcher/shot_event` | Publish | `WebotsRefereeTypes::WebotsLauncherShotEvent` | Shot event after an accepted request has completed its delay: request and fire time (µs), shot interval, bullet speed, heat before and after the shot, and more |

## 4. 配置示例 / Configuration Example

`xrobot instance add QDU-Robomaster/WebotsFireNotify` 写入的实例，无依赖项，`param` 按需修改：

An instance written by `xrobot instance add QDU-Robomaster/WebotsFireNotify`, which has no dependencies; `param` is adjusted as needed:

```yaml
modules:
  - module: QDU-Robomaster/WebotsFireNotify
    id: WebotsFireNotify_0
    args:
      - param:
          bullet_speed: 23.0f
          single_shot_heat: 10.0f
          shooter_heat_limit: 240.0f
          shooter_cooling_value: 40.0f
          max_fire_frequency_hz: 20.0f
          fire_delay_ms: 30.0f
          state_publish_period_ms: 10
```

创建 `host/fire_notify` 的实例（如 `QDU-Robomaster/Aimer`）列在本实例之前。

The instance that creates `host/fire_notify` (such as `QDU-Robomaster/Aimer`) is listed before this instance.

## 5. 依赖与硬件 / Dependencies and Hardware

依赖：

- `QDU-Robomaster/WebotsReferee`：`WebotsRefereeTypes.hpp` 中的发射机构状态与出弹事件类型。
- Webots：模块调用 Webots C++ API（`webots/Robot.hpp`、`webots/LED.hpp`），在 LibXR Webots 后端下构建（CI 使用 `-DLIBXR_SYSTEM=webots -DLIBXR_DRIVER=webots -DWEBOTS_HOME=/usr/local/webots`）。
- LibXR。

硬件：Webots world 中的可选 LED `fire_led`。

Dependencies:

- `QDU-Robomaster/WebotsReferee`: the launcher state and shot event types in `WebotsRefereeTypes.hpp`.
- Webots: the Module calls the Webots C++ API (`webots/Robot.hpp`, `webots/LED.hpp`) and is built with the LibXR Webots backend (CI uses `-DLIBXR_SYSTEM=webots -DLIBXR_DRIVER=webots -DWEBOTS_HOME=/usr/local/webots`).
- LibXR.

Hardware: the optional LED `fire_led` in the Webots world.
