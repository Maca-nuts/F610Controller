#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "can_msgs/msg/frame.hpp"
#include "f610_controller/msg/motor_target.hpp"
#include "f610_controller/msg/motor_state.hpp"
#include "rclcpp/rclcpp.hpp"

class F610Controller : public rclcpp::Node
{
    using Clock = std::chrono::steady_clock;
    using Target = f610_controller::msg::MotorTarget;
    using Frame = can_msgs::msg::Frame;
    using State = f610_controller::msg::MotorState;

public:
    F610Controller() : Node("f610_control")
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        const int control_period_ms = declare_parameter<int>("control_period_ms", 10, descriptor);
        velocity_kp_ = parameter("velocity_kp", 80.0);                                // 速度P
        velocity_ki_ = parameter("velocity_ki", 40.0);                                // 速度I
        position_kp_ = parameter("position_kp", 1.0);                                 // 位置P
        position_kd_ = parameter("position_kd", 0.01);                                // 位置D
        position_tolerance_rad_ = parameter("position_tolerance_rad", 0.008);         // 位置許容誤差[rad]
        gear_ratio_ = parameter("gear_ratio", 36.0, true);                            // エンコーダーと軸のギヤ比
        current_limit_ma_ = parameter("current_limit_ma", 1000.0, true);              // 電流制限[mA]
        position_speed_limit_rps_ = parameter("position_speed_limit_rps", 5.0, true); // 位置制御の速度制限[rps]
        state_pub_ = create_publisher<State>("motor_state", 10);
        can_pub_ = create_publisher<Frame>("to_can_bus", 10);
        target_sub_ = create_subscription<Target>(
            "motor_target", 10, [this](Target::ConstSharedPtr msg)
            { on_target(*msg); });
        can_sub_ = create_subscription<Frame>(
            "from_can_bus", 100, [this](Frame::ConstSharedPtr frame)
            { on_feedback(*frame); });
        last_tick_ = Clock::now();
        timer_ = create_wall_timer(std::chrono::milliseconds(control_period_ms),
                                   [this]()
                                   { control_step(); });
    }

private:
    static constexpr double kTwoPi = 6.28318530717958647692;         // π x 2
    static constexpr auto kTimeout = std::chrono::milliseconds(500); // 500ms

    struct Motor
    {
        Target target{};
        std::optional<Clock::time_point> target_time;
        std::optional<Clock::time_point> feedback_time;
        std::optional<int> previous_angle;
        std::optional<double> previous_position_error;
        int64_t rotor_position_counts = 0;
        double integral_ma = 0.0;
        int16_t rpm = 0;
        int16_t command_ma = 0;

        // エンコーダーからの累積カウントを回転速度[rad]に変換する
        double rotor_position_rad() const
        {
            return static_cast<double>(rotor_position_counts) * kTwoPi / 8192.0;
        }
        // 名前の通り
        void reset_control()
        {
            integral_ma = 0.0;
            previous_position_error.reset();
        }
    };

    double parameter(const std::string &name, double value, bool positive = false)
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        value = declare_parameter<double>(name, value, descriptor);
        if (!std::isfinite(value) || value < 0.0 || (positive && value == 0.0))
        {
            throw std::invalid_argument(name + " must be finite and " +
                                        (positive ? "positive" : "nonnegative"));
        }
        return value;
    }
    // 500msより古いデータは無視する
    static bool fresh(const std::optional<Clock::time_point> &time, Clock::time_point now)
    {
        return time && now - *time < kTimeout;
    }
    // F610からのフィードバックを読み取り
    void on_feedback(const Frame &frame)
    {
        if (frame.is_extended || frame.is_rtr || frame.is_error ||
            frame.id < 0x201 || frame.id > 0x208 || frame.dlc < 6)
        {
            return;
        }
        const int angle = (static_cast<int>(frame.data[0]) << 8) | frame.data[1];
        if (angle >= 8192)
            return;
        auto &motor = motors_[frame.id - 0x201];
        if (motor.previous_angle)
        {
            int delta = angle - *motor.previous_angle;
            if (delta > 4096)
                delta -= 8192;
            if (delta < -4096)
                delta += 8192;
            motor.rotor_position_counts += delta;
        }
        else
        {
            // 全体再起動時は最初の実測位置を原点にする。単回転角度を
            // 絶対的な出力軸位置や、電源断前の多回転位置として扱わない。
            motor.rotor_position_counts = 0;
            motor.reset_control();
            RCLCPP_INFO(get_logger(), "Motor %u initialized: current position is origin (encoder=%d)",
                        static_cast<unsigned>(frame.id - 0x200), angle);
        }
        motor.previous_angle = angle;
        motor.rpm = static_cast<int16_t>((static_cast<uint16_t>(frame.data[2]) << 8) |
                                         frame.data[3]);
        motor.feedback_time = Clock::now();
    }

    // 速度PI
    double rps_to_current(Motor &motor, double target_rps, double dt)
    {
        const double error = target_rps - motor.rpm / 60.0; // 現在の速度誤差(rps)を算出
        const double proportional = velocity_kp_ * error;
        const double increment = velocity_ki_ * error * dt;
        const double candidate = std::clamp(motor.integral_ma + increment,
                                            -current_limit_ma_, current_limit_ma_);
        const double output = proportional + candidate;
        // 飽和抑制
        if ((output <= current_limit_ma_ || increment < 0.0) &&
            (output >= -current_limit_ma_ || increment > 0.0))
        {
            motor.integral_ma = candidate;
        }
        return std::clamp(proportional + motor.integral_ma, -current_limit_ma_, current_limit_ma_);
    }

    // 位置PD
    double pos_to_rps(Motor &motor)
    {
        const double error = motor.target.target_pos * gear_ratio_ - motor.rotor_position_rad(); // 現在の角度誤差を算出
        // 目標通過後に、通過前の積分電流で押し続けないようにする。
        if (motor.previous_position_error && error * *motor.previous_position_error <= 0.0)
        {
            motor.integral_ma = 0.0;
        }
        motor.previous_position_error = error;
        // ↓rpm→rad/sの変換と誤差が許容範囲ならIをリセット
        const double angular_velocity = motor.rpm * kTwoPi / 60.0;
        const bool at_target = std::abs(error) <= position_tolerance_rad_ * gear_ratio_;
        if (at_target)
        {
            // 許容範囲内ではI項を溜めず、D項と速度P項による減衰だけを残す。
            motor.integral_ma = 0.0;
        }
        //
        return std::clamp((at_target ? 0.0 : position_kp_ * error) - position_kd_ * angular_velocity,
                          -position_speed_limit_rps_, position_speed_limit_rps_);
    }

    static uint8_t control_mode(const Target &target)
    {
        if (target.mode != Target::AUTO)
            return target.mode;
        if (target.target_pos != 0.0)
            return Target::POSITION;
        return target.target_rps != 0.0 ? Target::VELOCITY : Target::STOP;
    }

    void on_target(const Target &target)
    {
        if (target.id < 1 || target.id > 8)
        {
            RCLCPP_WARN(get_logger(), "Motor ID must be in 1..8: %d", target.id);
            return;
        }
        auto &motor = motors_[target.id - 1];
        used_groups_[(target.id - 1) / 4] = true;
        const auto mode = control_mode(target);
        if (!std::isfinite(target.target_rps) || !std::isfinite(target.target_pos) ||
            !std::isfinite(target.target_pos * gear_ratio_) ||
            target.mode > Target::POSITION ||
            (mode == Target::POSITION && target.target_rps != 0.0) ||
            (mode == Target::VELOCITY && target.target_pos != 0.0) ||
            (mode == Target::STOP && (target.target_rps != 0.0 || target.target_pos != 0.0)))
        {
            motor.target_time.reset();
            motor.reset_control();
            RCLCPP_WARN(get_logger(), "Invalid target for motor %d", target.id);
            return;
        }
        const auto now = Clock::now();
        if (!fresh(motor.target_time, now) || !fresh(motor.feedback_time, now) ||
            mode == Target::STOP || control_mode(motor.target) != mode)
        {
            motor.reset_control();
        }
        motor.target = target;
        motor.target_time = now;
    }

    void publish_state(int id, const Motor &motor)
    {
        State state;
        state.id = id;
        state.position_rad = motor.rotor_position_rad() / gear_ratio_;
        state.rotor_position_rad = motor.rotor_position_rad();
        state.velocity_rps = motor.rpm / 60.0;
        state.rotor_rpm = motor.rpm;
        state.commanded_current_ma = motor.command_ma;
        state.velocity_integral_ma = motor.integral_ma;
        state_pub_->publish(state);
    }

    void control_step()
    {
        const auto tick = Clock::now();
        const double dt = std::chrono::duration<double>(tick - last_tick_).count();
        last_tick_ = tick;
        const bool valid_dt = dt > 0.0 && dt < 0.5;
        const bool publish_state = tick - last_state_tick_ >= std::chrono::milliseconds(10);
        if (publish_state)
            last_state_tick_ = tick;
        for (int id = 1; id <= 8; ++id)
        {
            auto &motor = motors_[id - 1];
            if (!used_groups_[(id - 1) / 4])
                continue;
            double current_ma = 0.0;
            const bool feedback_ok = fresh(motor.feedback_time, tick);
            if (valid_dt && feedback_ok && fresh(motor.target_time, tick) &&
                control_mode(motor.target) != Target::STOP)
            {
                const double target_rps = control_mode(motor.target) == Target::POSITION
                                              ? pos_to_rps(motor)
                                              : motor.target.target_rps;
                current_ma = rps_to_current(motor, target_rps, dt);
            }
            else
            {
                motor.reset_control();
            }
            motor.command_ma = static_cast<int16_t>(std::lround(current_ma));
            if (publish_state && feedback_ok && motor.target.id == id)
            {
                this->publish_state(id, motor);
            }
        }
        publish_commands();
    }

    void publish_commands()
    {
        for (std::size_t group = 0; group < used_groups_.size(); ++group)
        {
            if (!used_groups_[group])
            {
                continue;
            }
            Frame frame{};
            frame.header.stamp = now();
            frame.id = group == 0 ? 0x200 : 0x1FF;
            frame.dlc = 8;
            for (std::size_t slot = 0; slot < 4; ++slot)
            {
                const auto raw = static_cast<uint16_t>(motors_[group * 4 + slot].command_ma);
                frame.data[slot * 2] = static_cast<uint8_t>(raw >> 8);
                frame.data[slot * 2 + 1] = static_cast<uint8_t>(raw);
            }
            can_pub_->publish(frame);
        }
    }

    double velocity_kp_, velocity_ki_, position_kp_, position_kd_;
    double position_tolerance_rad_, gear_ratio_;
    double current_limit_ma_, position_speed_limit_rps_;
    std::array<Motor, 8> motors_{};
    std::array<bool, 2> used_groups_{};
    Clock::time_point last_tick_;
    Clock::time_point last_state_tick_{};
    rclcpp::Publisher<State>::SharedPtr state_pub_;
    rclcpp::Subscription<Target>::SharedPtr target_sub_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
    rclcpp::Publisher<Frame>::SharedPtr can_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<F610Controller>());
    rclcpp::shutdown();
    return 0;
}
