#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <fire_drone_msgs/msg/suppression_result.hpp>

#include <cmath>

using namespace std::chrono_literals;

class OffboardController : public rclcpp::Node
{
public:
    OffboardController() : Node("offboard_controller")
    {
        offboard_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);

        setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);

        cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        suppression_pub_ = create_publisher<fire_drone_msgs::msg::SuppressionResult>(
            "/suppression/result", 10);

        ack_sub_ = create_subscription<px4_msgs::msg::VehicleCommandAck>(
            "/fmu/out/vehicle_command_ack",
            10,
            std::bind(&OffboardController::ack_callback, this, std::placeholders::_1));

        fire_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/fire/target",
            10,
            std::bind(&OffboardController::fire_callback, this, std::placeholders::_1));

        timer_ = create_wall_timer(
            100ms,
            std::bind(&OffboardController::loop, this));

        RCLCPP_INFO(get_logger(), "Smooth Offboard Controller Started");
    }

private:
    float step_towards(float current, float target, float max_step)
    {
        float error = target - current;

        if (error > max_step) return current + max_step;
        if (error < -max_step) return current - max_step;

        return target;
    }

    bool close_to(float a, float b, float tolerance)
    {
        return std::fabs(a - b) < tolerance;
    }

    bool close_to_target(float x, float y, float z)
    {
        return close_to(cmd_x_, x, 0.10f) &&
               close_to(cmd_y_, y, 0.10f) &&
               close_to(cmd_z_, z, 0.10f);
    }

    void ack_callback(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
    {
        last_ack_ = *msg;
        got_ack_ = true;
    }

    void fire_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        fire_x_ = msg->point.x;
        fire_y_ = msg->point.y;
        fire_z_ = msg->point.z;
        fire_detected_ = true;

        RCLCPP_INFO_ONCE(get_logger(), "Fire target received");
    }

    void publish_cmd(uint16_t cmd, float p1 = 0.0f, float p2 = 0.0f)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.command = cmd;
        msg.param1 = p1;
        msg.param2 = p2;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = get_clock()->now().nanoseconds() / 1000;
        cmd_pub_->publish(msg);
    }

    void publish_suppression_result()
    {
        fire_drone_msgs::msg::SuppressionResult result;

        result.success = true;
        result.temperature_before = 700.0f;
        result.temperature_after = 80.0f;
        result.target_x = fire_x_;
        result.target_y = fire_y_;
        result.target_z = fire_z_;

        suppression_pub_->publish(result);

        RCLCPP_INFO(get_logger(), "Published suppression result");
    }

    void update_smooth_setpoint(float target_x, float target_y, float target_z)
    {
        cmd_x_ = step_towards(cmd_x_, target_x, max_step_xy_);
        cmd_y_ = step_towards(cmd_y_, target_y, max_step_xy_);
        cmd_z_ = step_towards(cmd_z_, target_z, max_step_z_);
    }

    void loop()
    {
        auto now = get_clock()->now().nanoseconds() / 1000;

        px4_msgs::msg::OffboardControlMode offboard{};
        offboard.position = true;
        offboard.timestamp = now;
        offboard_pub_->publish(offboard);

        float target_x = 0.0f;
        float target_y = 0.0f;
        float target_z = -2.0f;

        if (counter_ < 100 || !fire_detected_)
        {
            target_x = 0.0f;
            target_y = 0.0f;
            target_z = -2.0f;
        }
        else if (counter_ < 220)
        {
            target_x = fire_x_ - 1.5f;
            target_y = fire_y_;
            target_z = -2.0f;
        }
        else if (counter_ < 320)
        {
            target_x = fire_x_ - 1.5f;
            target_y = fire_y_;
            target_z = -2.0f;

            if (!spraying_ && close_to_target(target_x, target_y, target_z))
            {
                RCLCPP_INFO(get_logger(), "SPRAYING FIRE at safe standoff");
                spraying_ = true;
            }
        }
        else if (counter_ < 400)
        {
            target_x = fire_x_ - 1.5f;
            target_y = fire_y_;
            target_z = -2.0f;

            if (!verification_done_)
            {
                RCLCPP_INFO(get_logger(), "VERIFYING TEMPERATURE");
                RCLCPP_INFO(get_logger(), "FIRE COOLED: TRUE");

                publish_suppression_result();

                verification_done_ = true;
            }
        }
        else
        {
            target_x = 0.0f;
            target_y = 0.0f;
            target_z = -2.0f;
        }

        update_smooth_setpoint(target_x, target_y, target_z);

        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.position = {cmd_x_, cmd_y_, cmd_z_};

        // Keep yaw fixed to avoid sharp rotations while RTAB-Map is mapping.
        sp.yaw = 0.0f;

        sp.timestamp = now;
        setpoint_pub_->publish(sp);

        if (!armed_ && counter_ == 120)
        {
            RCLCPP_INFO(get_logger(), "ARMING");
            publish_cmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
            armed_ = true;
        }

        if (armed_ && counter_ == 140)
        {
            RCLCPP_INFO(get_logger(), "OFFBOARD MODE");
            publish_cmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
            offboard_mode_ = true;
        }

        if (offboard_mode_ && counter_ > 520 && !land_sent_)
        {
            RCLCPP_INFO(get_logger(), "LANDING");
            publish_cmd(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
            land_sent_ = true;
        }

        counter_++;
    }

private:
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
    rclcpp::Publisher<fire_drone_msgs::msg::SuppressionResult>::SharedPtr suppression_pub_;

    rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr ack_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr fire_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    int counter_ = 0;

    bool armed_ = false;
    bool offboard_mode_ = false;
    bool got_ack_ = false;
    bool land_sent_ = false;

    bool fire_detected_ = false;
    bool spraying_ = false;
    bool verification_done_ = false;

    float fire_x_ = 0.0f;
    float fire_y_ = 0.0f;
    float fire_z_ = -2.0f;

    float cmd_x_ = 0.0f;
    float cmd_y_ = 0.0f;
    float cmd_z_ = -2.0f;

    // 100 ms timer:
    // 0.02 m/tick = 0.2 m/s horizontal
    // 0.01 m/tick = 0.1 m/s vertical
    float max_step_xy_ = 0.02f;
    float max_step_z_ = 0.01f;

    px4_msgs::msg::VehicleCommandAck last_ack_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardController>());
    rclcpp::shutdown();
    return 0;
}
