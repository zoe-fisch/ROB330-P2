#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "mbot_interfaces/msg/pose2_d_array.hpp"

class SubmissionPathPublisher : public rclcpp::Node {
public:
    SubmissionPathPublisher() : Node("submission_path_publisher") {
        pub_ = this->create_publisher<mbot_interfaces::msg::Pose2DArray>("/waypoints", 10);

        // Republish every second so late-connecting subscribers always receive the waypoints
        timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    [this]() {
        mbot_interfaces::msg::Pose2DArray goal_array;
        goal_array.poses = {
            pose(0.61,  0.00,  0.0),
            pose(0.61, -0.61, -M_PI_2),
            pose(1.22, -0.61,  0.0),
            pose(1.22,  0.61,  M_PI_2),
            pose(1.83,  0.61,  0.0),
            pose(1.83, -0.61, -M_PI_2),
            pose(2.44, -0.61,  0.0),
            pose(2.44,  0.00,  M_PI_2),
            pose(3.05,  0.00,  0.0)
        };

        pub_->publish(goal_array);

        RCLCPP_INFO(
            this->get_logger(),
            "Published submission trajectory with %zu poses.",
            goal_array.poses.size());

        timer_->cancel();
    });
    }

private:
    rclcpp::Publisher<mbot_interfaces::msg::Pose2DArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    static geometry_msgs::msg::Pose2D pose(double x, double y, double theta) {
        geometry_msgs::msg::Pose2D p;
        p.x = x;
        p.y = y;
        p.theta = theta;
        return p;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubmissionPathPublisher>());
    rclcpp::shutdown();
    return 0;
}