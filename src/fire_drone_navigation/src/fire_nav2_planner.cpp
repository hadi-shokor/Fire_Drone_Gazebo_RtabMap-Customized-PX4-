#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <std_msgs/msg/bool.hpp>

using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
using GoalHandleComputePathToPose =
    rclcpp_action::ClientGoalHandle<ComputePathToPose>;

class FireNav2Planner : public rclcpp::Node
{
public:
    FireNav2Planner()
        : Node("fire_nav2_planner")
    {
        mission_done_sub_ =
            this->create_subscription<std_msgs::msg::Bool>(
                "/fire/mission_reached",
                10,
                std::bind(
                    &FireNav2Planner::mission_done_callback,
                    this,
                    std::placeholders::_1));

        exploration_goal_sub_ =
            this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/fire/current_exploration_goal",
                10,
                std::bind(
                    &FireNav2Planner::goal_callback,
                    this,
                    std::placeholders::_1));

        path_pub_ =
            this->create_publisher<nav_msgs::msg::Path>(
                "/fire/nav2_path",
                10);

        nav2_client_ =
            rclcpp_action::create_client<ComputePathToPose>(
                this,
                "/compute_path_to_pose");

        RCLCPP_INFO(this->get_logger(), "Fire Nav2 Planner started.");
        RCLCPP_INFO(this->get_logger(), "Waiting for /fire/current_exploration_goal...");
    }

private:
    void mission_done_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg->data)
        {
            return;
        }

        mission_done_ = true;
        request_in_progress_ = false;

        RCLCPP_WARN(
            this->get_logger(),
            "Mission reached. Fire Nav2 Planner stopped.");
    }

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (mission_done_)
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                3000,
                "Mission reached. Ignoring new exploration goal.");
            return;
        }

        double goal_x = msg->pose.position.x;
        double goal_y = msg->pose.position.y;

        if (request_in_progress_)
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                3000,
                "Path request already in progress. Skipping this goal.");
            return;
        }

        if (!nav2_client_->wait_for_action_server(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available.");
            return;
        }

        ComputePathToPose::Goal goal_msg;

        goal_msg.goal.header.frame_id = "map";
        goal_msg.goal.header.stamp = this->now();

        goal_msg.goal.pose.position.x = goal_x;
        goal_msg.goal.pose.position.y = goal_y;
        goal_msg.goal.pose.position.z = 0.0;
        goal_msg.goal.pose.orientation.w = 1.0;

        goal_msg.planner_id = "GridBased";

        RCLCPP_WARN(
            this->get_logger(),
            "Requesting Nav2 path to exploration frontier: x=%.2f y=%.2f",
            goal_x,
            goal_y);

        request_in_progress_ = true;

        auto send_goal_options =
            rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();

        send_goal_options.result_callback =
            std::bind(
                &FireNav2Planner::result_callback,
                this,
                std::placeholders::_1);

        nav2_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void result_callback(const GoalHandleComputePathToPose::WrappedResult &result)
    {
        request_in_progress_ = false;

        if (mission_done_)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Mission reached. Ignoring computed path result.");
            return;
        }

        if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to compute path.");
            return;
        }

        auto path = result.result->path;

        if (path.poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty Nav2 path.");
            return;
        }

        path_pub_->publish(path);

        RCLCPP_INFO(
            this->get_logger(),
            "Published /fire/nav2_path with %zu waypoints.",
            path.poses.size());
    }

private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_done_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr exploration_goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp_action::Client<ComputePathToPose>::SharedPtr nav2_client_;

    bool request_in_progress_ = false;
    bool mission_done_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FireNav2Planner>());
    rclcpp::shutdown();
    return 0;
}
