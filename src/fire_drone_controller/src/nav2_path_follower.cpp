#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <nav_msgs/msg/path.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include <cmath>
#include <chrono>
#include <limits>

using namespace std::chrono_literals;

class Nav2PathFollower : public rclcpp::Node
{
public:
    Nav2PathFollower() : Node("nav2_path_follower")
    {
        auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10))
            .best_effort()
            .durability_volatile();

        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/fire/nav2_path",
            10,
            std::bind(&Nav2PathFollower::path_callback, this, std::placeholders::_1));

        local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1",
            px4_qos,
            std::bind(&Nav2PathFollower::local_position_callback, this, std::placeholders::_1));

        offboard_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode",
            px4_qos);

        trajectory_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint",
            px4_qos);

        cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command",
            px4_qos);

        timer_ = create_wall_timer(
            100ms,
            std::bind(&Nav2PathFollower::timer_callback, this));

        RCLCPP_INFO(get_logger(), "Nav2 PX4 path follower started.");
    }

private:
    void publish_cmd(uint16_t cmd, float p1 = 0.0f, float p2 = 0.0f)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = now().nanoseconds() / 1000;
        msg.command = cmd;
        msg.param1 = p1;
        msg.param2 = p2;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;

        cmd_pub_->publish(msg);
    }

    double distance2D(double x1, double y1, double x2, double y2) const
    {
        const double dx = x1 - x2;
        const double dy = y1 - y2;
        return std::sqrt(dx * dx + dy * dy);
    }

    size_t find_nearest_waypoint_index(const nav_msgs::msg::Path &path)
    {
        size_t best_index = 0;
        double best_dist = std::numeric_limits<double>::infinity();

        for (size_t i = 0; i < path.poses.size(); ++i)
        {
            const auto &p = path.poses[i].pose.position;
            double d = distance2D(p.x, p.y, drone_x_, drone_y_);

            if (d < best_dist)
            {
                best_dist = d;
                best_index = i;
            }
        }

        return best_index;
    }

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty())
        {
            RCLCPP_WARN(get_logger(), "Received empty Nav2 path. Ignoring.");
            return;
        }

        if (!has_position_)
        {
            RCLCPP_WARN(get_logger(), "Received path but no PX4 position yet. Ignoring.");
            return;
        }

        if (has_path_)
        {
            const auto &old_goal = current_path_.poses.back().pose.position;
            const auto &new_goal = msg->poses.back().pose.position;

            double goal_change = distance2D(
                old_goal.x,
                old_goal.y,
                new_goal.x,
                new_goal.y);

            if (goal_change < 3.0)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    3000,
                    "Already following similar path. Ignoring new path.");
                return;
            }

            RCLCPP_WARN(
                get_logger(),
                "New path goal changed significantly %.2f m. Switching to new path.",
                goal_change);
        }

        current_path_ = *msg;
        current_waypoint_index_ = find_nearest_waypoint_index(current_path_);
        has_path_ = true;

        const auto &nearest =
            current_path_.poses[current_waypoint_index_].pose.position;

        RCLCPP_INFO(
            get_logger(),
            "Accepted new frontier path with %zu waypoints. Start index=%zu nearest=(%.2f, %.2f) Last=(%.2f, %.2f)",
            current_path_.poses.size(),
            current_waypoint_index_,
            nearest.x,
            nearest.y,
            current_path_.poses.back().pose.position.x,
            current_path_.poses.back().pose.position.y);
    }

    void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        drone_x_ = msg->y;
        drone_y_ = -msg->x;
        drone_z_ = -msg->z;
        has_position_ = true;
    }

    void publish_offboard_mode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = now().nanoseconds() / 1000;
        offboard_pub_->publish(msg);
    }

    void publish_setpoint(double target_x, double target_y)
    {
        px4_msgs::msg::TrajectorySetpoint setpoint{};

        setpoint.position[0] = -target_y;
        setpoint.position[1] = target_x;
        setpoint.position[2] = -3.0;

        setpoint.yaw = std::numeric_limits<float>::quiet_NaN();
        setpoint.timestamp = now().nanoseconds() / 1000;

        trajectory_pub_->publish(setpoint);
    }

    void publish_hold_setpoint()
    {
        px4_msgs::msg::TrajectorySetpoint setpoint{};

        setpoint.position[0] = -drone_y_;
        setpoint.position[1] = drone_x_;
        setpoint.position[2] = -3.0;

        setpoint.yaw = std::numeric_limits<float>::quiet_NaN();
        setpoint.timestamp = now().nanoseconds() / 1000;

        trajectory_pub_->publish(setpoint);
    }

    void arm_and_offboard_logic()
    {
        /*
         * PX4 requires continuous offboard_control_mode + trajectory_setpoint
         * before ARM and before switching to OFFBOARD.
         */

        if (!has_position_)
        {
            return;
        }

        if (!armed_ && counter_ == 120)
        {
            RCLCPP_INFO(get_logger(), "ARMING");

            publish_cmd(
                px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0f);

            armed_ = true;
        }

        if (armed_ && !offboard_mode_ && counter_ == 140)
        {
            RCLCPP_INFO(get_logger(), "OFFBOARD MODE");

            publish_cmd(
                px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                1.0f,
                6.0f);

            offboard_mode_ = true;
        }
    }

    void timer_callback()
    {
        publish_offboard_mode();

        if (!has_position_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                3000,
                "Waiting for PX4 local position...");
            return;
        }

        /*
         * Always publish a trajectory setpoint before arming/offboard.
         * If there is no path yet, hold current position at z=-3.
         */
        if (!has_path_ || current_path_.poses.empty())
        {
            publish_hold_setpoint();
            arm_and_offboard_logic();
            counter_++;
            return;
        }

        while (current_waypoint_index_ < current_path_.poses.size())
        {
            const auto &p =
                current_path_.poses[current_waypoint_index_].pose.position;

            double d = distance2D(
                p.x,
                p.y,
                drone_x_,
                drone_y_);

            if (d >= reach_threshold_)
            {
                break;
            }

            current_waypoint_index_++;
        }

        if (current_waypoint_index_ >= current_path_.poses.size())
        {
            RCLCPP_INFO(
                get_logger(),
                "Finished current frontier path. Waiting for next path...");

            has_path_ = false;
            current_waypoint_index_ = 0;

            publish_hold_setpoint();
            arm_and_offboard_logic();
            counter_++;
            return;
        }

        const auto &pose =
            current_path_.poses[current_waypoint_index_].pose;

        const double target_x = pose.position.x;
        const double target_y = pose.position.y;

        const double dist =
            distance2D(target_x, target_y, drone_x_, drone_y_);

        publish_setpoint(target_x, target_y);

        arm_and_offboard_logic();
        counter_++;

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "wp=%zu/%zu target_ros=(%.2f, %.2f, 3.0) drone_ros=(%.2f, %.2f, %.2f) dist=%.2f | sent_px4_ned=(%.2f, %.2f, -3.0) armed=%d offboard=%d",
            current_waypoint_index_,
            current_path_.poses.size(),
            target_x,
            target_y,
            drone_x_,
            drone_y_,
            drone_z_,
            dist,
            -target_y,
            target_x,
            armed_,
            offboard_mode_);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path current_path_;

    bool has_path_ = false;
    bool has_position_ = false;

    bool armed_ = false;
    bool offboard_mode_ = false;

    int counter_ = 0;

    size_t current_waypoint_index_ = 0;

    double drone_x_ = 0.0;
    double drone_y_ = 0.0;
    double drone_z_ = 0.0;

    double reach_threshold_ = 0.30;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Nav2PathFollower>());
    rclcpp::shutdown();
    return 0;
}
