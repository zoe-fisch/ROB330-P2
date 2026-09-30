#include "common/moving_laser_scan.hpp"
#include "common/utils.hpp"
#include <cmath>

// Linear interpolation for geometry_msgs::msg::Point
inline geometry_msgs::msg::Point linearInterpolatePoint(const geometry_msgs::msg::Point& a,
                                           const geometry_msgs::msg::Point& b,
                                           float t)
{
    geometry_msgs::msg::Point out;
    out.x = a.x + t * (b.x - a.x);
    out.y = a.y + t * (b.y - a.y);
    out.z = a.z + t * (b.z - a.z);
    return out;
}


MovingLaserScan::MovingLaserScan(const sensor_msgs::msg::LaserScan& scan,
                                 const Pose& start_pose,
                                 const Pose& end_pose)
{
    deskewScan(scan, start_pose, end_pose);
}

void MovingLaserScan::deskewScan(const sensor_msgs::msg::LaserScan& scan,
                                     const Pose& start_pose,
                                     const Pose& end_pose)
{
    const size_t num_rays = scan.ranges.size();
    rays_.clear();
    rays_.reserve(num_rays);

    // Extract yaw from quaternions (world-frame base yaw at scan start/end)
    const double yaw_start = yawFromQuaternion(start_pose.orientation);
    const double yaw_end = yawFromQuaternion(end_pose.orientation);
    
    const double delta_yaw = angleDiff(yaw_end, yaw_start);
    
    // Iterate over all beams
    for (size_t i = 0; i < num_rays; ++i)
    {
        const float range = scan.ranges[i];

        // Drop garbage / under-range / out-of-range
        if (std::isnan(range) || range < scan.range_min || range > scan.range_max) {
            continue;
        }
        float frac = 0.0f;
        if (num_rays > 1) {
            frac = static_cast<float>(i) / static_cast<float>(num_rays - 1);
        }
        const double delta_lidar = scan.angle_min + i * scan.angle_increment;

        InterpolatedRay ray;
        ray.origin = linearInterpolatePoint(start_pose.position, end_pose.position, frac);
        ray.theta = static_cast<float>(yaw_start + frac * delta_yaw + delta_lidar);
        ray.range = range;

        rays_.push_back(ray);
    }
}