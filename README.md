# F610Controller

F610(C610)用コントローラー。速度PI、または位置PD→速度PIのカスケード制御を提供

## 入出力と単位

| トピック       | 型                                | 内容                      |
| -------------- | --------------------------------- | ------------------------- |
| `motor_target` | `f610_controller/msg/MotorTarget` | IDと速度/位置指令         |
| `motor_state`  | `f610_controller/msg/MotorState`  | 位置・速度・指令電流・I項 |
| `from_can_bus` | `can_msgs/msg/Frame`              | モーターフィードバック    |
| `to_can_bus`   | `can_msgs/msg/Frame`              | 電流指令                  |

## 暫定のパラメータ


| パラメータ                 | 既定値 | 単位                          |
| -------------------------- | -----: | ----------------------------- |
| `velocity_kp`              |   80.0 | mA / ローターrps              |
| `velocity_ki`              |   40.0 | mA / (ローターrps・s)         |
| `position_kp`              |    1.0 | ローターrps / ローターrad     |
| `position_kd`              |   0.01 | ローターrps / (ローターrad/s) |
| `gear_ratio`               |   36.0 | ローター回転 / 出力軸回転     |
| `position_tolerance_rad`   |  0.008 | 出力軸rad                     |
| `position_speed_limit_rps` |    5.0 | ローターrps                   |
| `current_limit_ma`         | 1000.0 | mA                            |
| `control_period_ms`        |     10 | ms、1〜20                     |

速度指令の例: `{id: 1, target_rps: 5.0, target_pos: 0.0}`
位置指令の例: `{id: 1, target_rps: 0.0, target_pos: 1.57}`
