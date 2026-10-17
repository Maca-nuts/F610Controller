# F610Controller

F610(C610)用コントローラー。速度PI制御と位置PD→速度PIのカスケード制御を提供

## コントローラーの起動

```bash
colcon build --packages-select f610_controller
source install/setup.bash
ros2 launch f610_controller f610_controller.launch.py

# 別のYAMLを指定する場合（絶対パス）
ros2 launch f610_controller f610_controller.launch.py params_file:=/absolute/path/motors.yaml
```

既定の設定は `config/f610_controller.yaml`。`motors.id_1`〜`motors.id_8` が
`MotorTarget.id` の1〜8に対応します。各IDに速度PIの `velocity_kp` / `velocity_ki`、
位置PDの `position_kp` / `position_kd` を指定できます。

```yaml
f610_control:
  ros__parameters:
    motors:
      id_1:
        velocity_kp: 80.0
        velocity_ki: 40.0
        position_kp: 1.0
        position_kd: 0.01
      id_2:
        velocity_kp: 100.0
        velocity_ki: 50.0
```
## 指令ノードの起動

```bash
ros2 run f610_controller motor_target_publisher --ros-args -p motor_id:=1 -p mode:=position -p target_pos:=1.57
```

### motor_target_publisher のパラメータ

| 引数           | 既定値 | 意味                                                               |
| -------------- | ------ | ------------------------------------------------------------------ |
| `motor_id`     | `1`    | モーターID。整数1〜8                                               |
| `mode`         | `auto` | `auto` / `position` / `velocity` / `stop`　(下記を参照)            |
| `target_pos`   | `0.0`  | 起動原点からの出力軸位置 [rad]。正負の多回転指定が可能             |
| `target_turns` | `0.0`  | 起動原点からの出力軸回転数。内部で `回転数 × 2π` radへ変換         |
| `target_rps`   | `0.0`  | 出力軸速度 [回転/秒]。内部で `gear_ratio` 倍してローター速度へ変換 |


| mode       | 動作                                                                                 |
| ---------- | ------------------------------------------------------------------------------------ |
| `position` | 指定位置へ移動して保持。位置0も原点への位置指令                                      |
| `velocity` | `target_rps` で速度制御。速度0は速度制御による停止                                   |
| `stop`     | 電流指令0。位置保持なし。目標値はすべて0にする                                       |
| `auto`     | 非ゼロの位置目標があれば位置制御、非ゼロの速度目標があれば速度制御。すべて0なら電流0 |

### radによる±101回転と原点復帰

多回転を前提に書いてるので2πで折返しなどはせず、回り続ける

e.g. 出力軸101回転は `101 × 2π = 634.6017160251382 rad` 

motor_target_publisherの使い方例
```bash
# +101回転の位置（rad指定）
ros2 run f610_controller motor_target_publisher --ros-args -p mode:=position -p target_pos:=634.6017160251382
# -101回転の位置（rad指定）
ros2 run f610_controller motor_target_publisher --ros-args -p mode:=position -p target_pos:=-634.6017160251382
# 同じ+101回転を回転数で指定
ros2 run f610_controller motor_target_publisher --ros-args -p mode:=position -p target_turns:=101
# 原点へ戻って保持
ros2 run f610_controller motor_target_publisher --ros-args -p mode:=position -p target_pos:=0
# 出力軸を1 rpsで速度制御（gear_ratio=36ならローターは36 rps）
ros2 run f610_controller motor_target_publisher --ros-args -p mode:=velocity -p target_rps:=1
# 電流指令0
ros2 run f610_controller motor_target_publisher --ros-args -p mode:=stop
```


## 入出力と単位

| トピック       | 型                                | 内容                      |
| -------------- | --------------------------------- | ------------------------- |
| `motor_target` | `f610_controller/msg/MotorTarget` | IDと速度/位置指令         |
| `motor_state`  | `f610_controller/msg/MotorState`  | 位置・速度・指令電流・I項 |
| `from_can_bus` | `can_msgs/msg/Frame`              | モーターフィードバック    |
| `to_can_bus`   | `can_msgs/msg/Frame`              | 電流指令                  |

`target_rps` は出力軸の速度です。`motor_state.velocity_rps` と
`position_speed_limit_rps`、制御ゲインは引き続きローター基準です。

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

## 更新記録

V1.0.0 プロジェクト公開

V1.1.0 専用のpublisherを追加
