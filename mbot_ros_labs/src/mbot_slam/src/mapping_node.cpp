#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "common/mapping.hpp"
#include "common/moving_laser_scan.hpp"

#include <iostream>

class MappingNode : public rclcpp::Node
{
public:
    MappingNode()
    : Node("mbot_mapping_node"),
      grid_(10.0, 10.0, 0.05, -5.0, -5.0),  // 10x10m map, 5cm resolution, origin at center
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
    {
        RCLCPP_INFO(this->get_logger(), "MappingNode will start in 1 second...");
        std::this_thread::sleep_for(std::chrono::seconds(1));

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MappingNode::scanCallback, this, std::placeholders::_1));

        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).transient_local());

        double publish_rate = 1.0; // Hz
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate),
            std::bind(&MappingNode::publishMap, this));
    }

private:
    OccupancyGrid grid_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool first_scan_received_ = false;
    int debug_scan_count_ = 0;

    void scanCallback(sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        if (!first_scan_received_){
            RCLCPP_INFO(this->get_logger(), "First scan received. Start to mapping...");
            first_scan_received_ = true;
        }

        // Lookup pose at first ray, bail out if unavailable
        geometry_msgs::msg::Pose scan_start_pose;
        try {
            geometry_msgs::msg::TransformStamped tf =
                tf_buffer_->lookupTransform("odom", "lidar_link", scan->header.stamp,
                                            rclcpp::Duration::from_seconds(0.1));
            scan_start_pose.position.x  = tf.transform.translation.x;
            scan_start_pose.position.y  = tf.transform.translation.y;
            scan_start_pose.position.z  = tf.transform.translation.z;
            scan_start_pose.orientation = tf.transform.rotation;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Failed to lookup odom transform: %s", ex.what());
            return;
        }

        // Lookup pose at last ray, falls back to start pose
        const rclcpp::Time scan_end_time = scan->header.stamp + rclcpp::Duration::from_seconds(scan->scan_time);
        geometry_msgs::msg::Pose scan_end_pose = scan_start_pose;
        try {
            geometry_msgs::msg::TransformStamped tf =
                tf_buffer_->lookupTransform("odom", "lidar_link", scan_end_time,
                                            rclcpp::Duration::from_seconds(0.1));
            scan_end_pose.position.x  = tf.transform.translation.x;
            scan_end_pose.position.y  = tf.transform.translation.y;
            scan_end_pose.position.z  = tf.transform.translation.z;
            scan_end_pose.orientation = tf.transform.rotation;
        } catch (tf2::TransformException &ex) {
            RCLCPP_DEBUG(this->get_logger(), "Using start pose as fallback: %s", ex.what());
        }

        // Deskew the lidar scan from the robot pose at the first ray
        // to the robot pose at the last ray
        MovingLaserScan deskewed_scan(*scan, scan_start_pose, scan_end_pose);

        if (debug_scan_count_ < 3) {
            std::cout << "\n===== CALLING updateGrid =====" << std::endl;
        }

        updateGrid(deskewed_scan);

        debug_scan_count_++;
    }

    void updateGrid(const MovingLaserScan& scan)
    {
        bool debug = debug_scan_count_ < 3;

        if (debug) {
            std::cout << "ENTERED updateGrid" << std::endl;
        }

        int ray_count = 0;
        int nonempty_ray_count = 0;
        int total_cells = 0;

        for (const auto& ray : scan)
        {
            ray_count++;

            if (debug && ray_count <= 5) {
                std::cout << "ray " << ray_count
                          << " origin=(" << ray.origin.x
                          << ", " << ray.origin.y << ")"
                          << " theta=" << ray.theta
                          << " range=" << ray.range
                          << std::endl;
            }

            auto ray_cells = bresenhamRayTrace(
                ray.origin.x, ray.origin.y, ray.theta, ray.range, grid_);

            if (debug && ray_count <= 5) {
                std::cout << "ray_cells size = "
                          << ray_cells.size()
                          << std::endl;
            }

            if (!ray_cells.empty()) {
                nonempty_ray_count++;
                total_cells += ray_cells.size();

                const size_t free_cell_count = ray.has_hit ? ray_cells.size() - 1 : ray_cells.size();
                for (size_t j = 0; j < free_cell_count; ++j) {
                    grid_.markCellFree(ray_cells[j].first, ray_cells[j].second);
                }

                if (ray.has_hit) {
                    const auto& endpoint = ray_cells.back();
                    if (debug && ray_count <= 5) {
                        std::cout << "calling markCellOccupied("
                                  << endpoint.first << ", "
                                  << endpoint.second << ")"
                                  << std::endl;
                    }
                    grid_.markCellOccupied(endpoint.first, endpoint.second);
                }
            }
        }

        if (debug) {
            std::cout << "----- updateGrid summary -----" << std::endl;
            std::cout << "rays = " << ray_count << std::endl;
            std::cout << "nonempty rays = " << nonempty_ray_count << std::endl;
            std::cout << "total cells = " << total_cells << std::endl;
            std::cout << "------------------------------" << std::endl;
        }
    }

    void publishMap()
    {
        nav_msgs::msg::OccupancyGrid map_msg;
        map_msg.header.frame_id = "map";
        map_msg.header.stamp = this->now();

        map_msg.info.resolution = grid_.getResolution();
        map_msg.info.width      = grid_.getWidth();
        map_msg.info.height     = grid_.getHeight();
        map_msg.info.origin.position.x = grid_.getOriginX();
        map_msg.info.origin.position.y = grid_.getOriginY();
        map_msg.info.origin.position.z = 0;
        map_msg.info.origin.orientation.w = 1.0;

        map_msg.data = grid_.getOccupancyGrid();
        map_pub_->publish(map_msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MappingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}