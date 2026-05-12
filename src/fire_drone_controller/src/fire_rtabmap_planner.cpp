#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

class FireRtabmapPlanner : public rclcpp::Node
{
public:
    FireRtabmapPlanner() : Node("fire_rtabmap_planner")
    {
        auto path_qos = rclcpp::QoS(10).transient_local().reliable();

        alarm_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/fire/truck_alarm",
            10,
            std::bind(&FireRtabmapPlanner::alarm_callback, this, std::placeholders::_1));

        rtabmap_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/rtabmap/global_path",
            10,
            std::bind(&FireRtabmapPlanner::path_callback, this, std::placeholders::_1));

        rtabmap_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/rtabmap/goal",
            10);

        controller_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/fire/planned_path",
            path_qos);

        RCLCPP_INFO(this->get_logger(), "Fire RTAB-Map planner started.");
        RCLCPP_INFO(this->get_logger(), "Waiting for /fire/truck_alarm...");
    }

private:
    void alarm_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        if (goal_sent_)
        {
            return;
        }

        alarm_x_ = msg->point.x;
        alarm_y_ = msg->point.y;

        RCLCPP_WARN(
            this->get_logger(),
            "Received fire alarm area: x=%.2f y=%.2f",
            alarm_x_,
            alarm_y_);

        publish_rtabmap_goal();

        goal_sent_ = true;
    }

    void publish_rtabmap_goal()
    {
        geometry_msgs::msg::PoseStamped goal;

        // Important: manual publishing worked with stamp = 0.
        goal.header.stamp.sec = 0;
        goal.header.stamp.nanosec = 0;
        goal.header.frame_id = "map";

        goal.pose.position.x = alarm_x_;
        goal.pose.position.y = alarm_y_;
        goal.pose.position.z = 0.0;

        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = 0.0;
        goal.pose.orientation.w = 1.0;

        rtabmap_goal_pub_->publish(goal);

        RCLCPP_INFO(
            this->get_logger(),
            "Published RTAB-Map goal: x=%.2f y=%.2f",
            alarm_x_,
            alarm_y_);
    }

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (!goal_sent_)
        {
            return;
        }

        if (msg->poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty RTAB-Map path.");
            return;
        }

        nav_msgs::msg::Path path_to_controller = *msg;

        path_to_controller.header.stamp = this->get_clock()->now();
        path_to_controller.header.frame_id = "map";

        controller_path_pub_->publish(path_to_controller);

        RCLCPP_INFO(
            this->get_logger(),
            "Published /fire/planned_path. Waypoints: %zu",
            path_to_controller.poses.size());
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr alarm_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr rtabmap_path_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr rtabmap_goal_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr controller_path_pub_;

    bool goal_sent_ = false;

    double alarm_x_ = 0.0;
    double alarm_y_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FireRtabmapPlanner>());
    rclcpp::shutdown();
    return 0;
}
