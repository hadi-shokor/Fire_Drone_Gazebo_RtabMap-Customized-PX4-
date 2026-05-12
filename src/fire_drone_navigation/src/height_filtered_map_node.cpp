#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cmath>

class HeightFilteredMapNode : public rclcpp::Node
{
public:
    HeightFilteredMapNode() : Node("height_filtered_map_node")
    {
        auto map_qos = rclcpp::QoS(1)
            .reliable()
            .transient_local();

        auto cloud_qos = rclcpp::QoS(1)
            .reliable()
            .transient_local();

        auto output_qos = rclcpp::QoS(1)
            .reliable()
            .transient_local();

        base_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/rtabmap/grid_prob_map",
            map_qos,
            std::bind(&HeightFilteredMapNode::map_callback, this, std::placeholders::_1));

        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/rtabmap/cloud_map",
            cloud_qos,
            std::bind(&HeightFilteredMapNode::cloud_callback, this, std::placeholders::_1));

        filtered_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/fire/flyable_grid",
            output_qos);

        RCLCPP_INFO(get_logger(), "Height filtered map node started.");
        RCLCPP_INFO(get_logger(), "Subscribing with transient_local QoS.");
        RCLCPP_INFO(get_logger(), "Blocking obstacles with height >= 2.8 m.");
    }

private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        base_map_ = *msg;
        has_map_ = true;

        RCLCPP_INFO_ONCE(
            get_logger(),
            "Received /rtabmap/grid_prob_map: width=%u height=%u resolution=%.3f",
            base_map_.info.width,
            base_map_.info.height,
            base_map_.info.resolution);

        publish_filtered_map();
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        latest_cloud_ = *msg;
        has_cloud_ = true;

        RCLCPP_INFO_ONCE(
            get_logger(),
            "Received /rtabmap/cloud_map: points step=%u width=%u height=%u",
            latest_cloud_.point_step,
            latest_cloud_.width,
            latest_cloud_.height);

        publish_filtered_map();
    }

    bool world_to_grid(double wx, double wy, int &gx, int &gy) const
    {
        if (!has_map_)
        {
            return false;
        }

        double origin_x = base_map_.info.origin.position.x;
        double origin_y = base_map_.info.origin.position.y;
        double res = base_map_.info.resolution;

        gx = static_cast<int>((wx - origin_x) / res);
        gy = static_cast<int>((wy - origin_y) / res);

        return gx >= 0 &&
               gy >= 0 &&
               gx < static_cast<int>(base_map_.info.width) &&
               gy < static_cast<int>(base_map_.info.height);
    }

    int index(int gx, int gy) const
    {
        return gy * static_cast<int>(base_map_.info.width) + gx;
    }

    void publish_filtered_map()
    {
        if (!has_map_ || !has_cloud_)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                3000,
                "Waiting for both map and cloud. has_map=%d has_cloud=%d",
                has_map_,
                has_cloud_);
            return;
        }

        nav_msgs::msg::OccupancyGrid output = base_map_;

        /*
         * Important:
         * RTAB-Map grid is used as the base.
         * Then we add 3D obstacles that are high enough to block the drone.
         *
         * Drone flies at about 3 m.
         * Any obstacle point with z >= 2.8 m blocks that x-y cell.
         */
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(latest_cloud_, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(latest_cloud_, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(latest_cloud_, "z");

        int marked = 0;

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            double x = *iter_x;
            double y = *iter_y;
            double z = *iter_z;

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            {
                continue;
            }

            if (z < obstacle_height_threshold_)
            {
                continue;
            }

            int gx = 0;
            int gy = 0;

            if (!world_to_grid(x, y, gx, gy))
            {
                continue;
            }

            output.data[index(gx, gy)] = 100;
            marked++;
        }

        output.header.stamp = now();
        output.header.frame_id = "map";

        filtered_map_pub_->publish(output);

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            3000,
            "Published /fire/flyable_grid | height obstacles marked=%d",
            marked);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr base_map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr filtered_map_pub_;

    nav_msgs::msg::OccupancyGrid base_map_;
    sensor_msgs::msg::PointCloud2 latest_cloud_;

    bool has_map_ = false;
    bool has_cloud_ = false;

    double obstacle_height_threshold_ = 2.8;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeightFilteredMapNode>());
    rclcpp::shutdown();
    return 0;
}
