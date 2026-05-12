#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <cmath>
#include <cstdint>

class FireDetectorApproach : public rclcpp::Node
{
public:
    FireDetectorApproach() : Node("fire_detector")
    {
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/world/city/model/x500_fire_drone_0/link/camera_link/sensor/camera/image",
            10,
            std::bind(&FireDetectorApproach::image_callback, this, std::placeholders::_1));

        local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1",
            rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
            std::bind(&FireDetectorApproach::local_position_callback, this, std::placeholders::_1));

        offboard_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode",
            rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());

        setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint",
            rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&FireDetectorApproach::control_loop, this));

        RCLCPP_INFO(get_logger(), "Fire detector max-scan center-and-approach started.");
    }

private:
    enum class State
    {
        SEARCH_FIRE,
        FIND_MAX_PIXELS,
        RETURN_TO_BEST_YAW,
        CENTER_FIRE,
        APPROACH_FIRE,
        STOPPED
    };

    void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        px4_x_ = msg->x;
        px4_y_ = msg->y;
        px4_z_ = msg->z;
        px4_heading_ = msg->heading;
        has_position_ = true;
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (msg->encoding != "rgb8")
        {
            fire_pixels_ = 0;
            filtered_fire_pixels_ = 0.0;
            fire_detected_ = false;
            fire_center_error_x_ = 9999.0;
            return;
        }

        uint64_t fire_pixels = 0;
        uint64_t sum_u = 0;

        for (uint32_t v = 0; v < msg->height; ++v)
        {
            for (uint32_t u = 0; u < msg->width; ++u)
            {
                const uint32_t idx = v * msg->step + u * 3;

                const uint8_t r = msg->data[idx + 0];
                const uint8_t g = msg->data[idx + 1];
                const uint8_t b = msg->data[idx + 2];

                    const bool fire_like =
                        r >= 230 &&
                        g >= 120 &&
                        g <= 190 &&
                        b <= 80 &&
                        r > g + 40 &&
                        g > b + 50;

                if (fire_like)
                {
                    fire_pixels++;
                    sum_u += u;
                }
            }
        }

        fire_pixels_ = fire_pixels;

        filtered_fire_pixels_ =
            0.60 * filtered_fire_pixels_ +
            0.40 * static_cast<double>(fire_pixels_);

        fire_detected_ = filtered_fire_pixels_ >= min_fire_pixels_;

        image_center_x_ = static_cast<double>(msg->width) / 2.0;

        if (fire_pixels > 0)
        {
            fire_center_x_ =
                static_cast<double>(sum_u) / static_cast<double>(fire_pixels);

            fire_center_error_x_ =
                fire_center_x_ - image_center_x_;
        }
        else
        {
            fire_center_error_x_ = 9999.0;
        }

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Fire raw=%lu filtered=%.0f detected=%d center_x=%.1f error=%.1f state=%d",
            fire_pixels_,
            filtered_fire_pixels_,
            fire_detected_,
            fire_center_x_,
            fire_center_error_x_,
            static_cast<int>(state_));
    }

    void publish_offboard_mode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = now().nanoseconds() / 1000;
        msg.position = true;
        offboard_pub_->publish(msg);
    }

    void publish_setpoint(double x, double y, double z, double yaw)
    {
        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.timestamp = now().nanoseconds() / 1000;
        sp.position[0] = static_cast<float>(x);
        sp.position[1] = static_cast<float>(y);
        sp.position[2] = static_cast<float>(z);
        sp.yaw = static_cast<float>(yaw);
        setpoint_pub_->publish(sp);
    }

    double wrap_pi(double a)
    {
        while (a > M_PI)
            a -= 2.0 * M_PI;

        while (a < -M_PI)
            a += 2.0 * M_PI;

        return a;
    }

    double yaw_error(double target, double current)
    {
        return wrap_pi(target - current);
    }

    void start_max_scan()
    {
        state_ = State::FIND_MAX_PIXELS;

        scan_yaw_ = px4_heading_;
        best_yaw_ = px4_heading_;
        best_fire_pixels_ = filtered_fire_pixels_;

        drop_counter_ = 0;
        centered_counter_ = 0;

        /*
         * Choose scan direction based on where fire appears in image.
         * error < 0  => fire is left  => rotate negative yaw
         * error > 0  => fire is right => rotate positive yaw
         */
        if (fire_center_error_x_ < 0.0)
        {
            scan_direction_ = -1.0;
        }
        else
        {
            scan_direction_ = 1.0;
        }

        RCLCPP_WARN(
            get_logger(),
            "Fire detected. Starting max-pixel scan. direction=%.1f center_error=%.1f",
            scan_direction_,
            fire_center_error_x_);
    }

    void control_loop()
    {
        publish_offboard_mode();

        if (!has_position_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Waiting for PX4 local position...");
            return;
        }

        const double hold_z = -3.0;

        if (state_ == State::STOPPED)
        {
            publish_setpoint(px4_x_, px4_y_, hold_z, locked_yaw_);
            return;
        }

        /*
         * STATE 1:
         * No fire detected.
         * Drone yaws only until fire appears.
         */
        if (state_ == State::SEARCH_FIRE)
        {
            if (fire_detected_)
            {
                start_max_scan();
                return;
            }

            scan_yaw_ = wrap_pi(px4_heading_ + search_yaw_rate_ * dt_);

            publish_setpoint(px4_x_, px4_y_, hold_z, scan_yaw_);

            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Searching fire. Yaw only. yaw=%.2f raw=%lu filtered=%.0f",
                scan_yaw_,
                fire_pixels_,
                filtered_fire_pixels_);

            return;
        }

        /*
         * STATE 2:
         * Fire detected.
         * Yaw in selected direction and find maximum fire pixels.
         */
        if (state_ == State::FIND_MAX_PIXELS)
        {
            scan_yaw_ =
                wrap_pi(px4_heading_ + scan_direction_ * fine_yaw_rate_ * dt_);

            if (filtered_fire_pixels_ > best_fire_pixels_)
            {
                best_fire_pixels_ = filtered_fire_pixels_;
                best_yaw_ = px4_heading_;
                drop_counter_ = 0;
            }
            else if (best_fire_pixels_ > lock_min_best_pixels_ &&
                     filtered_fire_pixels_ < best_fire_pixels_ * lock_drop_ratio_)
            {
                drop_counter_++;
            }
            else
            {
                drop_counter_ = 0;
            }

            publish_setpoint(px4_x_, px4_y_, hold_z, scan_yaw_);

            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Max scan. yaw=%.2f best_yaw=%.2f dir=%.1f raw=%lu filtered=%.0f best=%.0f drops=%d",
                scan_yaw_,
                best_yaw_,
                scan_direction_,
                fire_pixels_,
                filtered_fire_pixels_,
                best_fire_pixels_,
                drop_counter_);

            if (best_fire_pixels_ > lock_min_best_pixels_ &&
                drop_counter_ >= required_drop_frames_)
            {
                state_ = State::RETURN_TO_BEST_YAW;

                RCLCPP_WARN(
                    get_logger(),
                    "Max pixels found. Returning to best_yaw=%.2f best_pixels=%.0f",
                    best_yaw_,
                    best_fire_pixels_);

                return;
            }

            return;
        }

        /*
         * STATE 3:
         * Return to yaw where fire pixels were maximum.
         */
        if (state_ == State::RETURN_TO_BEST_YAW)
        {
            publish_setpoint(px4_x_, px4_y_, hold_z, best_yaw_);

            const double err = yaw_error(best_yaw_, px4_heading_);

            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Returning to best yaw. current=%.2f target=%.2f error=%.2f fire=%d filtered=%.0f",
                px4_heading_,
                best_yaw_,
                err,
                fire_detected_,
                filtered_fire_pixels_);

            if (std::fabs(err) <= best_yaw_tolerance_)
            {
                if (fire_detected_)
                {
                    locked_yaw_ = px4_heading_;
                    centered_counter_ = 0;
                    state_ = State::CENTER_FIRE;

                    RCLCPP_WARN(
                        get_logger(),
                        "Best yaw reached and fire visible. Starting camera centering.");
                }
                else
                {
                    state_ = State::SEARCH_FIRE;

                    RCLCPP_WARN(
                        get_logger(),
                        "Best yaw reached but fire not visible. Returning to search.");
                }

                return;
            }

            return;
        }

        /*
         * STATE 4:
         * Fire is visible after maximum-pixel scan.
         * Now center the fire blob in camera.
         */
        if (state_ == State::CENTER_FIRE)
        {
            if (!fire_detected_)
            {
                state_ = State::SEARCH_FIRE;

                RCLCPP_WARN(
                    get_logger(),
                    "Fire lost during centering. Returning to search.");

                return;
            }

            if (std::fabs(fire_center_error_x_) <= center_tolerance_px_)
            {
                centered_counter_++;
            }
            else
            {
                centered_counter_ = 0;
            }

            if (centered_counter_ >= required_centered_frames_)
            {
                locked_yaw_ = px4_heading_;
                state_ = State::APPROACH_FIRE;

                RCLCPP_WARN(
                    get_logger(),
                    "Fire centered. Locking yaw %.2f and moving forward. error=%.1f",
                    locked_yaw_,
                    fire_center_error_x_);

                return;
            }

            /*
             * Centering direction:
             * error < 0 means fire is left, so yaw command should be negative.
             * error > 0 means fire is right, so yaw command should be positive.
             */
            double yaw_correction =
                centering_gain_ * fire_center_error_x_;

            if (yaw_correction > max_center_yaw_step_)
            {
                yaw_correction = max_center_yaw_step_;
            }
            else if (yaw_correction < -max_center_yaw_step_)
            {
                yaw_correction = -max_center_yaw_step_;
            }

            scan_yaw_ = wrap_pi(px4_heading_ + yaw_correction);

            publish_setpoint(px4_x_, px4_y_, hold_z, scan_yaw_);

            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Centering fire. center_x=%.1f image_center=%.1f error=%.1f yaw_cmd=%.2f centered=%d",
                fire_center_x_,
                image_center_x_,
                fire_center_error_x_,
                scan_yaw_,
                centered_counter_);

            return;
        }

        /*
         * STATE 5:
         * Fire is centered.
         * Move forward.
         */
        if (state_ == State::APPROACH_FIRE)
        {
            if (filtered_fire_pixels_ >= stop_fire_pixels_)
            {
                state_ = State::STOPPED;

                publish_setpoint(px4_x_, px4_y_, hold_z, locked_yaw_);

                RCLCPP_WARN(
                    get_logger(),
                    "STOPPED. Fire threshold reached. filtered_pixels=%.0f raw=%lu",
                    filtered_fire_pixels_,
                    fire_pixels_);

                return;
            }

            const double target_x =
                px4_x_ + forward_step_ * std::cos(locked_yaw_);

            const double target_y =
                px4_y_ + forward_step_ * std::sin(locked_yaw_);

            publish_setpoint(target_x, target_y, hold_z, locked_yaw_);

            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Forward approach. target=(%.2f, %.2f, %.2f) locked_yaw=%.2f raw=%lu filtered=%.0f center_error=%.1f",
                target_x,
                target_y,
                hold_z,
                locked_yaw_,
                fire_pixels_,
                filtered_fire_pixels_,
                fire_center_error_x_);

            return;
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    State state_ = State::SEARCH_FIRE;

    bool has_position_ = false;
    bool fire_detected_ = false;

    double px4_x_ = 0.0;
    double px4_y_ = 0.0;
    double px4_z_ = 0.0;
    double px4_heading_ = 0.0;

    double scan_yaw_ = 0.0;
    double best_yaw_ = 0.0;
    double locked_yaw_ = 0.0;

    double scan_direction_ = 1.0;

    uint64_t fire_pixels_ = 0;
    double filtered_fire_pixels_ = 0.0;
    double best_fire_pixels_ = 0.0;

    double fire_center_x_ = 0.0;
    double image_center_x_ = 0.0;
    double fire_center_error_x_ = 9999.0;

    int drop_counter_ = 0;
    int centered_counter_ = 0;

    const double dt_ = 0.1;

    // Faster yaw search
    const double search_yaw_rate_ = 0.80;

    // Faster max-pixel scan
    const double fine_yaw_rate_ = 0.25;

    // Faster forward movement
    const double forward_step_ = 0.30;

    const double min_fire_pixels_ = 200.0;
    const double stop_fire_pixels_ = 70000.0;

    const double lock_min_best_pixels_ = 1000.0;
    const double lock_drop_ratio_ = 0.45;
    const int required_drop_frames_ = 3;

    const double best_yaw_tolerance_ = 0.15;

    const double center_tolerance_px_ = 80.0;
    const int required_centered_frames_ = 2;

    const double centering_gain_ = 0.006;
    const double max_center_yaw_step_ = 0.25;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FireDetectorApproach>());
    rclcpp::shutdown();
    return 0;
}
