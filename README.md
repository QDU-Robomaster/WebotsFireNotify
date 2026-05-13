# WebotsFireNotify

`WebotsFireNotify` 是 Webots 中的发射机构仿真模块。它订阅
`host/fire_notify`，把消息视为开火请求，并在射频、发弹延迟和热量限制通过后发布真实出弹事件。

## 输入输出

输入:

- `host/fire_notify`: DevC `LauncherCMD{bool isfire}` 同布局载荷，`isfire=true` 表示请求开火。

输出:

- `webots_launcher/state`: 当前发射机构状态，包含热量、冷却、射频、延迟、待发状态和最近拒绝原因。
- `webots_launcher/shot_event`: 真实接受并完成延迟后的出弹事件。
- Webots `fire_led`: 只作为请求、待发和真实出弹的短暂视觉提示，不作为真实出弹依据。

## 发射规则

- `host/fire_notify` 只代表请求，不代表已经发弹。
- 同一时刻只允许一发处于发弹延迟队列。
- `max_fire_frequency_hz` 转换为最小请求间隔，超过射频的请求会被拒绝。
- `fire_delay_ms` 到期后才发布 `shot_event`，并在此刻增加热量。
- `shooter_cooling_value` 按秒恢复热量，热量不会低于 0。
- `shooter_heat_limit > 0` 时，若本次发射会使热量达到或超过上限，请求会被拒绝。

## 配置

- `bullet_speed`: 弹丸初速度，单位 m/s。
- `single_shot_heat`: 单发增加热量。
- `shooter_heat_limit`: 热量上限；小于等于 0 时不启用热量拒绝。
- `shooter_cooling_value`: 每秒恢复热量。
- `max_fire_frequency_hz`: 最大射频；小于等于 0 时不启用射频拒绝。
- `fire_delay_ms`: 请求到真实出弹的延迟，单位 ms。
- `state_publish_period_ms`: 状态发布周期，单位 ms。

## 依赖

- `qdu-future/WebotsReferee`: 复用 `WebotsRefereeTypes.hpp` 中的裁判和发射机构共享类型。
