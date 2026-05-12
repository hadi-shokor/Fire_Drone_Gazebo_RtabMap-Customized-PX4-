#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <random>
#include <sstream>
#include <cstdlib>
#include <string>
#include <cmath>

using namespace std::chrono_literals;

class FireSourceSim : public rclcpp::Node
{
public:
    FireSourceSim() : Node("fire_source_sim")
    {
        this->declare_parameter<std::string>("world", "baylands");
        this->declare_parameter<std::string>(
            "model_path",
            "/home/user/fire_drone_ws/models/real_fire/model.sdf");

        world_ = this->get_parameter("world").get_parameter_value().get<std::string>();
        model_path_ = this->get_parameter("model_path").get_parameter_value().get<std::string>();

        exact_fire_publisher_ =
            this->create_publisher<geometry_msgs::msg::PointStamped>(
                "/fire/target", 10);

        truck_alarm_publisher_ =
            this->create_publisher<geometry_msgs::msg::PointStamped>(
                "/fire/truck_alarm", 10);

        rng_ = std::mt19937(rd_());

        x_dist_ = std::uniform_real_distribution<float>(-33.0f, -33.0f);
        y_dist_ = std::uniform_real_distribution<float>(-12.0f, -12.0f);
        
        //x_dist_ = std::uniform_real_distribution<float>(-33.0f, -33.0f);
        //y_dist_ = std::uniform_real_distribution<float>(-10.0f, -10.0f);

        // 20 m^2 square around fire:
        // side length = sqrt(20) = 4.472 m
        // half side = 2.236 m
        alarm_noise_dist_ = std::uniform_real_distribution<float>(-2.236f, 2.236f);

        fire_x_ = x_dist_(rng_);
        fire_y_ = y_dist_(rng_);

        // Generate ONE fixed approximate alarm point inside the 20 m^2 area
        alarm_x_ = fire_x_ + alarm_noise_dist_(rng_);
        alarm_y_ = fire_y_ + alarm_noise_dist_(rng_);

        RCLCPP_INFO(
            this->get_logger(),
            "Generated real fire: x=%.2f y=%.2f in world=%s",
            fire_x_,
            fire_y_,
            world_.c_str());

        RCLCPP_WARN(
            this->get_logger(),
            "Generated truck alarm point inside 20 m^2 area: x=%.2f y=%.2f",
            alarm_x_,
            alarm_y_);

        spawn_fire_in_gazebo();

        timer_ = this->create_wall_timer(
            1s,
            [this]()
            {
                publish_exact_fire();
                publish_truck_alarm();
            });
    }

private:
    void spawn_fire_in_gazebo()
    {
        std::stringstream cmd;

        cmd << "gz service -s /world/" << world_ << "/create "
            << "--reqtype gz.msgs.EntityFactory "
            << "--reptype gz.msgs.Boolean "
            << "--timeout 3000 "
            << "--req 'sdf_filename: \"" << model_path_ << "\", "
            << "name: \"real_fire\", "
            << "pose: {position: {x: " << fire_x_
            << ", y: " << fire_y_
            << ", z: 0.2}}'";

        RCLCPP_INFO(this->get_logger(), "Spawning real_fire model in Gazebo...");

        int ret = std::system(cmd.str().c_str());

        if (ret == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Spawn command sent successfully");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Spawn command failed");
        }
    }

    void publish_truck_alarm()
    {
        geometry_msgs::msg::PointStamped msg;

        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";

        msg.point.x = alarm_x_;
        msg.point.y = alarm_y_;
        msg.point.z = 0.0;

        truck_alarm_publisher_->publish(msg);

        RCLCPP_WARN(
            this->get_logger(),
            "ALARM to truck: fixed approximate point x=%.2f y=%.2f | real fire x=%.2f y=%.2f",
            alarm_x_,
            alarm_y_,
            fire_x_,
            fire_y_);
    }

    void publish_exact_fire()
    {
        geometry_msgs::msg::PointStamped msg;

        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";

        msg.point.x = fire_x_;
        msg.point.y = fire_y_;
        msg.point.z = -2.0;

        exact_fire_publisher_->publish(msg);

        RCLCPP_INFO(
            this->get_logger(),
            "Publishing exact fire target: x=%.2f y=%.2f z=%.2f",
            msg.point.x,
            msg.point.y,
            msg.point.z);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr exact_fire_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr truck_alarm_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::random_device rd_;
    std::mt19937 rng_;

    std::uniform_real_distribution<float> x_dist_;
    std::uniform_real_distribution<float> y_dist_;
    std::uniform_real_distribution<float> alarm_noise_dist_;

    std::string world_;
    std::string model_path_;

    float fire_x_ = 5.0f;
    float fire_y_ = 2.0f;

    float alarm_x_ = 0.0f;
    float alarm_y_ = 0.0f;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FireSourceSim>());
    rclcpp::shutdown();
    return 0;
}
