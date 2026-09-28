# WebotsFireNotify

`WebotsFireNotify` 是 Webots 中的发射机构仿真模块。它订阅 `host/fire_notify`，把消息视为开火请求，
并在射频、发弹延迟和热量限制通过后发布真实出弹事件。

## 输入输出

输入：

- `host/fire_notify`：DevC `LauncherCMD{bool isfire}` 同布局的 1 字节载荷，`isfire=true` 表示请求
  开火，`false` 被忽略。构造时必须已经存在该 topic（通常由 `QDU-Robomaster/Aimer` 创建），
  否则记录错误并抛出 `std::runtime_error`。

输出（`webots_launcher` 域）：

- `webots_launcher/state`：`WebotsRefereeTypes::WebotsLauncherState`，当前发射机构状态，包含热量、
  冷却、射频、延迟、待发状态、出弹计数和最近拒绝原因。构造时、每次处理开火请求时以及每
  `state_publish_period_ms` 发布一次。
- `webots_launcher/shot_event`：`WebotsRefereeTypes::WebotsLauncherShotEvent`，请求被接受并完成
  延迟后的真实出弹事件，包含请求/出弹时间（µs）、出弹间隔、弹速、出弹前后热量等。
- Webots `fire_led`：只作为请求、待发和真实出弹的短暂视觉提示，不作为真实出弹依据；world 中
  没有该 LED 时不点灯。

## 发射规则

- `host/fire_notify` 只代表请求，不代表已经发弹。
- 同一时刻只允许一发处于发弹延迟队列，期间的新请求以 `PENDING` 拒绝。
- `max_fire_frequency_hz` 转换为最小请求间隔，距离上次被接受的请求不足该间隔时以 `RATE_LIMIT` 拒绝。
- `shooter_heat_limit > 0` 时，若本次发射会使热量达到或超过上限，以 `HEAT_LIMIT` 拒绝。
- `fire_delay_ms` 到期后（由周期定时任务推进）才发布 `shot_event`，并在此刻增加热量。
- `shooter_cooling_value` 按秒恢复热量，热量不会低于 0。
- 负数或非有限的浮点配置按 0 处理。

`WebotsReferee` 订阅上述两个 topic，把热量上限和冷却值同步到 `robot_game_ref` 裁判摘要。

## 依赖

- `QDU-Robomaster/WebotsReferee`：复用 `WebotsRefereeTypes.hpp` 中的发射机构状态与出弹事件类型。

外部依赖：Webots。模块直接调用 Webots C++ API（`webots/Robot.hpp`、`webots/LED.hpp`），只能在
LibXR Webots 后端下构建（CI 使用 `-DLIBXR_SYSTEM=webots -DLIBXR_DRIVER=webots
-DWEBOTS_HOME=/usr/local/webots`）。

## 构造接口

```cpp
WebotsFireNotify(const Param& param = {.bullet_speed = 23.0f, .single_shot_heat = 10.0f,
                                       .shooter_heat_limit = 240.0f,
                                       .shooter_cooling_value = 40.0f,
                                       .max_fire_frequency_hz = 20.0f,
                                       .fire_delay_ms = 30.0f,
                                       .state_publish_period_ms = 10});
```

无依赖项。

配置（`Param`）：

- `bullet_speed`：弹丸初速度，单位 m/s，默认 `23.0`。
- `single_shot_heat`：单发增加热量，默认 `10.0`。
- `shooter_heat_limit`：热量上限，默认 `240.0`；为 0 时不启用热量拒绝。
- `shooter_cooling_value`：每秒恢复热量，默认 `40.0`。
- `max_fire_frequency_hz`：最大射频，单位 Hz，默认 `20.0`；为 0 时不启用射频拒绝。
- `fire_delay_ms`：请求到真实出弹的延迟，单位 ms，默认 `30.0`。
- `state_publish_period_ms`：状态发布与延迟推进的定时任务周期，单位 ms，默认 `10`，最小按 1 执行。

## 使用

```sh
xrobot module add QDU-Robomaster/WebotsFireNotify
xrobot setup
xrobot instance add QDU-Robomaster/WebotsFireNotify
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出。
本模块没有依赖项，按需修改 `param`：

```yaml
modules:
  - module: QDU-Robomaster/WebotsFireNotify
    id: webotsfirenotify_0
    args:
      - param:
          bullet_speed: 23.0f
          single_shot_heat: 10.0f
          shooter_heat_limit: 240.0f
          shooter_cooling_value: 40.0f
          max_fire_frequency_hz: 20.0f
          fire_delay_ms: 30.0f
          state_publish_period_ms: '10'
```

本模块不使用 BSP 对象，不需要 `XR_REGISTER`。创建 `host/fire_notify` 的实例（如
`QDU-Robomaster/Aimer`）必须在 `modules:` 中排在本实例之前。

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/QDU-Robomaster/WebotsFireNotify`
（在 BSP 中）打印当前的构造函数。
