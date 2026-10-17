#include <chrono>
#include <cstdint>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "f610_controller/msg/motor_target.hpp"
#include "rclcpp/rclcpp.hpp"

class MotorTargetPublisher : public rclcpp::Node
{
public:
    MotorTargetPublisher() : Node("motor_target_publisher")
    {
        const auto motor_id = declare_parameter<int>("motor_id", 1);
        const double target_rps = read_target_parameter("target_rps");
        double target_pos = read_target_parameter("target_pos");
        const double target_turns = read_target_parameter("target_turns");
        const auto mode = declare_parameter<std::string>("mode", "auto");
        if (target_turns != 0.0)
        {
            if (target_pos != 0.0)
                throw std::invalid_argument("set only one of target_pos and target_turns");
            target_pos = target_turns * 6.28318530717958647692;
        }
        using Target = f610_controller::msg::MotorTarget;
        if (mode == "auto")
            target_.mode = Target::AUTO;
        else if (mode == "stop")
            target_.mode = Target::STOP;
        else if (mode == "velocity")
            target_.mode = Target::VELOCITY;
        else if (mode == "position")
            target_.mode = Target::POSITION;
        else
            throw std::invalid_argument("mode must be auto, stop, velocity or position");
        if ((mode == "position" && target_rps != 0.0) ||
            (mode == "velocity" && target_pos != 0.0) ||
            (mode == "stop" && (target_rps != 0.0 || target_pos != 0.0)))
            throw std::invalid_argument("target values conflict with mode");
        target_.id = motor_id;
        target_.target_rps = target_rps;
        target_.target_pos = target_pos;
        publisher_ = create_publisher<f610_controller::msg::MotorTarget>("motor_target", 10);
        RCLCPP_INFO(get_logger(), "Publishing motor %d: mode=%s, target_rps=%.3f output-shaft rps, target_pos=%.9f rad",
                    static_cast<int>(motor_id), mode.c_str(), target_rps, target_pos);
        timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]()
                                   {
                                       if (publisher_->get_subscription_count() == 0)
                                       {
                                           RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                                                "No motor_target subscriber; start controller node");
                                       }
                                       publisher_->publish(target_); });
    }

private:
    // 起動時の目標値を読み込む。未指定なら 0.0、整数・小数のどちらも使用可能。
    double read_target_parameter(const char *name)
    {
        // パラメータを宣言し、起動時に指定された値を取得する。
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.dynamic_typing = true; // 100（整数）と 100.0（小数）の両方を受け付ける
        descriptor.read_only = true;      // 起動後の変更は禁止する
        const auto parameter = declare_parameter(name, rclcpp::ParameterValue(0.0), descriptor);

        // 数値は double にそろえ、文字列や真偽値などはエラーにする。
        double target = 0.0;
        if (parameter.get_type() == rclcpp::PARAMETER_INTEGER)
        {
            target = static_cast<double>(parameter.get<int64_t>());
        }
        else if (parameter.get_type() == rclcpp::PARAMETER_DOUBLE)
        {
            target = parameter.get<double>();
        }
        else
        {
            throw std::invalid_argument(" must be a number");
        }
        return target;
    }

    f610_controller::msg::MotorTarget target_{};
    rclcpp::Publisher<f610_controller::msg::MotorTarget>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<MotorTargetPublisher>());
    }
    catch (const std::exception &error)
    {
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
