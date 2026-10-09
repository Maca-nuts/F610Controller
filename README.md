# F610Controller

C610用ROS 2コントローラー。速度PI、または位置PD→速度PIのカスケード制御で電流を送信します。

**実機ゲイン調整は途中です。位置精度は未達。** 測定値と実機試験の再開手順は
[HANDOFF.md](HANDOFF.md)を参照してください。2026-10-09に制御ノードを整理し、
仮想CANの統合テスト8件に合格しました。この整理後の実機試験はまだ行っていません。

## 入出力と単位

| トピック | 型 | 内容 |
| --- | --- | --- |
| `motor_target` | `f610_controller/msg/MotorTarget` | IDと速度/位置指令 |
| `motor_state` | `f610_controller/msg/MotorState` | 約100 Hzの位置・速度・指令電流・I項 |
| `from_can_bus` | `can_msgs/msg/Frame` | モーターフィードバック |
| `to_can_bus` | `can_msgs/msg/Frame` | 電流指令 |

- `id`: 1〜8。
- `target_rps`: **ローター軸rps**。
- `target_pos`: **減速後の出力軸rad**。`gear_ratio`の既定値は36:1。
- 片方だけ非0にする。両方0は0 mA停止であり、位置0への移動/位置保持ではない。
- 両方非0や非有限値は、そのIDを次周期から0 mAにする。不正IDは無視する。
- 指令またはフィードバックが500 ms途絶えると0 mAにする。

以前の実装/初期試験では位置をローターradとしていたが、ユーザーの指定に合わせて
現在は出力軸radへ変更した。速度指令はローターrpsのまま。

`motor_state.position_rad`は出力軸位置、`rotor_position_rad`はローター位置、
`velocity_rps`と`rotor_rpm`はローター速度。
`commanded_current_ma`は送信する電流指令で、実測電流ではない。

## 制御と位置原点

速度PI:

```text
error = target_rps - measured_rpm / 60
integral += velocity_ki * error * dt
current_ma = velocity_kp * error + integral
```

電流飽和を悪化させる積分を抑止する。状態はID別に保持し、停止・タイムアウト・
モード切替・不正指令・周期異常などでリセットする。

位置PD:

```text
error_rotor_rad = target_pos * gear_ratio - rotor_position_rad
omega_rotor = measured_rpm / 60 * 2π
target_rps = position_kp * error_rotor_rad - position_kd * omega_rotor
```

目標速度を`±position_speed_limit_rps`に制限し、速度PIへ渡す。
D項は実測速度を使う。位置目標通過時には速度I項をリセットする。
位置許容帯内では毎周期I項をリセットし、位置P項を止めて速度による減衰を残す。
許容帯は実際の位置精度や停止距離を保証するものではない。

各IDの初回フィードバックを位置原点とする。8192カウント/回転を折り返し補正して
多回転積算する。停止・モード変更で原点は保持し、ノード再起動で変わる。
連続受信間に半回転以上動くと回転数を復元できないため、通信断後などは原点を再設定する。

## 暫定パラメータ

起動時のみ設定できる。現在の値は最終調整済みではない。

| パラメータ | 既定値 | 単位 |
| --- | ---: | --- |
| `velocity_kp` | 80.0 | mA / ローターrps |
| `velocity_ki` | 40.0 | mA / (ローターrps・s) |
| `position_kp` | 1.0 | ローターrps / ローターrad |
| `position_kd` | 0.01 | ローターrps / (ローターrad/s) |
| `gear_ratio` | 36.0 | ローター回転 / 出力軸回転 |
| `position_tolerance_rad` | 0.008 | 出力軸rad |
| `position_speed_limit_rps` | 5.0 | ローターrps |
| `current_limit_ma` | 1000.0 | mA |
| `control_period_ms` | 10 | ms、1〜20 |

ゲイン・許容帯は有限の非負値。減速比・上限値は有限の正値。
コードの電流上限設定は最大10000 mAだが、**今回の実機でユーザーが許可した上限は4000 mA**。
全IDに共通のゲイン・減速比で、実機測定はID 1のみ。

CAN標準データフレーム0x201〜0x208、DLC 6以上、角度0〜8191のみ受理する。
電流指令はID 1〜4を0x200、5〜8を0x1FFへまとめて送信する。
指令を受けたグループだけ送信し、そのグループの未指令IDには0を詰める。
同一グループを別ノードから同時に制御しないこと。

## ビルドと起動

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select f610_controller
source install/setup.bash
ros2 run f610_controller f610_checker
```

ノード自体はSocketCANを開かない。通常運用では別途CANブリッジが必要。
調整スクリプトは専用ROSドメインからSocketCANへ直接中継するため、通常運用と経路が異なる。

速度指令の例: `{id: 1, target_rps: 5.0, target_pos: 0.0}`。
位置指令の例: `{id: 1, target_rps: 0.0, target_pos: 1.57}`（出力軸約90度）。
停止指令の例: `{id: 1, target_rps: 0.0, target_pos: 0.0}`。

## 仮想フィードバックによるテスト

```bash
source install/setup.bash
ROS_LOG_DIR=/tmp/f610_test_logs ROS_DOMAIN_ID=175 \
  colcon test --packages-select f610_controller \
  --ctest-args -R controller_integration --output-on-failure
colcon test-result --test-result-base build/f610_controller --verbose
```

実機CANへ接続しない8件のテストを用意している。
2026-10-09の制御ノード整理後に8件すべて成功済み。
