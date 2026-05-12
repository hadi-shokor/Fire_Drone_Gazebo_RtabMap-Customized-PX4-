#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <limits>
#include <vector>

class FireFrontierPlanner : public rclcpp::Node
{
public:
    FireFrontierPlanner() : Node("fire_frontier_planner")
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        fire_alarm_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/fire/truck_alarm", 10,
            std::bind(&FireFrontierPlanner::fireAlarmCallback, this, std::placeholders::_1));

        auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/fire/flyable_grid", map_qos,
            std::bind(&FireFrontierPlanner::mapCallback, this, std::placeholders::_1));

        auto goal_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        frontier_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/fire/current_exploration_goal", goal_qos);

        mission_done_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/fire/mission_reached", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&FireFrontierPlanner::planningLoop, this));

        RCLCPP_INFO(this->get_logger(), "Fire frontier planner started.");
    }

private:
    void fireAlarmCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        fire_x_ = msg->point.x;
        fire_y_ = msg->point.y;
        has_fire_alarm_ = true;

        RCLCPP_INFO(
            this->get_logger(),
            "Received fire alarm target: x=%.2f y=%.2f",
            fire_x_,
            fire_y_);
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_ = *msg;
        has_map_ = true;

        RCLCPP_INFO_ONCE(
            this->get_logger(),
            "Received occupancy grid: width=%u height=%u resolution=%.3f",
            map_.info.width,
            map_.info.height,
            map_.info.resolution);
    }

    bool getRobotPoseFromTF(double &robot_x, double &robot_y)
    {
        try
        {
            auto transform = tf_buffer_->lookupTransform(
                "map",
                "base_link",
                tf2::TimePointZero);

            robot_x = transform.transform.translation.x;
            robot_y = transform.transform.translation.y;
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                3000,
                "Could not get map -> base_link TF: %s",
                ex.what());
            return false;
        }
    }

    bool isInsideMap(int x, int y) const
    {
        return x >= 0 &&
               y >= 0 &&
               x < static_cast<int>(map_.info.width) &&
               y < static_cast<int>(map_.info.height);
    }

    int index(int x, int y) const
    {
        return y * map_.info.width + x;
    }

    bool isFree(int x, int y) const
    {
        if (!isInsideMap(x, y))
        {
            return false;
        }

        int value = map_.data[index(x, y)];
        return value >= 0 && value < 50;
    }

    bool isUnknown(int x, int y) const
    {
        if (!isInsideMap(x, y))
        {
            return false;
        }

        return map_.data[index(x, y)] == -1;
    }

    bool isFrontier(int x, int y) const
    {
        if (!isFree(x, y))
        {
            return false;
        }

        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                if (dx == 0 && dy == 0)
                {
                    continue;
                }

                int nx = x + dx;
                int ny = y + dy;

                if (isInsideMap(nx, ny) && isUnknown(nx, ny))
                {
                    return true;
                }
            }
        }

        return false;
    }

    void gridToWorld(int gx, int gy, double &wx, double &wy) const
    {
        wx = map_.info.origin.position.x + (gx + 0.5) * map_.info.resolution;
        wy = map_.info.origin.position.y + (gy + 0.5) * map_.info.resolution;
    }

    double distance(double x1, double y1, double x2, double y2) const
    {
        double dx = x1 - x2;
        double dy = y1 - y2;
        return std::sqrt(dx * dx + dy * dy);
    }

    void publishMissionReached(double robot_to_fire)
    {
        std_msgs::msg::Bool done;
        done.data = true;
        mission_done_pub_->publish(done);

        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            3000,
            "Reached fire area. Mission complete. distance=%.2f",
            robot_to_fire);
    }

    void planningLoop()
    {
        if (!has_fire_alarm_ || !has_map_)
        {
            return;
        }

        double robot_x = 0.0;
        double robot_y = 0.0;

        if (!getRobotPoseFromTF(robot_x, robot_y))
        {
            return;
        }

        const double robot_to_fire = distance(robot_x, robot_y, fire_x_, fire_y_);

        if (robot_to_fire < fire_reached_distance_)
        {
            publishMissionReached(robot_to_fire);
            return;
        }

        const double map_min_x = map_.info.origin.position.x;
        const double map_min_y = map_.info.origin.position.y;
        const double map_max_x = map_min_x + map_.info.width * map_.info.resolution;
        const double map_max_y = map_min_y + map_.info.height * map_.info.resolution;

        double best_x = 0.0;
        double best_y = 0.0;
        double best_score = std::numeric_limits<double>::infinity();

        int frontier_count = 0;
        int valid_frontier_count = 0;
        int rejected_bounds = 0;
        int rejected_distance = 0;
        int rejected_backward = 0;

        for (int y = 1; y < static_cast<int>(map_.info.height) - 1; y++)
        {
            for (int x = 1; x < static_cast<int>(map_.info.width) - 1; x++)
            {
                if (!isFrontier(x, y))
                {
                    continue;
                }

                frontier_count++;

                double wx = 0.0;
                double wy = 0.0;
                gridToWorld(x, y, wx, wy);

                if (wx < map_min_x + map_margin_ || wx > map_max_x - map_margin_ ||
                    wy < map_min_y + map_margin_ || wy > map_max_y - map_margin_)
                {
                    rejected_bounds++;
                    continue;
                }

                const double dist_to_robot = distance(wx, wy, robot_x, robot_y);
                const double dist_to_fire = distance(wx, wy, fire_x_, fire_y_);

                if (dist_to_robot < min_frontier_distance_from_robot_ ||
                    dist_to_robot > max_frontier_distance_from_robot_)
                {
                    rejected_distance++;
                    continue;
                }

                // Allow sideways movement, but block frontiers that clearly move away from the fire.
                if (dist_to_fire > robot_to_fire + progress_tolerance_)
                {
                    rejected_backward++;
                    continue;
                }

                valid_frontier_count++;

                const double score =
                    dist_to_fire +
                    0.2 * dist_to_robot;

                if (score < best_score)
                {
                    best_score = score;
                    best_x = wx;
                    best_y = wy;
                }
            }
        }

        if (frontier_count == 0)
        {
            RCLCPP_WARN(this->get_logger(), "No frontier found.");
            return;
        }

        if (valid_frontier_count == 0)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "No valid frontier. robot=(%.2f, %.2f), fire=(%.2f, %.2f), robot_to_fire=%.2f | frontiers=%d rejected_bounds=%d rejected_distance=%d rejected_backward=%d",
                robot_x,
                robot_y,
                fire_x_,
                fire_y_,
                robot_to_fire,
                frontier_count,
                rejected_bounds,
                rejected_distance,
                rejected_backward);
            return;
        }

        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp = this->get_clock()->now();
        goal.header.frame_id = "map";
        goal.pose.position.x = best_x;
        goal.pose.position.y = best_y;
        goal.pose.position.z = 0.0;
        goal.pose.orientation.w = 1.0;

        frontier_goal_pub_->publish(goal);

        RCLCPP_INFO(
            this->get_logger(),
            "Selected frontier goal: x=%.2f y=%.2f | robot=(%.2f, %.2f) | fire=(%.2f, %.2f) | frontiers=%d valid=%d | score=%.2f | robot_to_fire=%.2f goal_to_fire=%.2f",
            best_x,
            best_y,
            robot_x,
            robot_y,
            fire_x_,
            fire_y_,
            frontier_count,
            valid_frontier_count,
            best_score,
            robot_to_fire,
            distance(best_x, best_y, fire_x_, fire_y_));
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr fire_alarm_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr frontier_goal_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_done_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    nav_msgs::msg::OccupancyGrid map_;

    bool has_fire_alarm_ = false;
    bool has_map_ = false;

    double fire_x_ = 0.0;
    double fire_y_ = 0.0;

    const double fire_reached_distance_ = 8.0;
    const double min_frontier_distance_from_robot_ = 2.0;
    const double max_frontier_distance_from_robot_ = 5.0;
    const double map_margin_ = 0.5;
    const double progress_tolerance_ = 1.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FireFrontierPlanner>());
    rclcpp::shutdown();
    return 0;
}
