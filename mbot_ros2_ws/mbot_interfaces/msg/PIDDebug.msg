# Debug data for tuning the waypoint-following PID controller
float64 target_distance         # distance to the active goal
float64 signed_distance_error   # error projected onto the robot heading (+ = ahead)
float64 signed_angle_error      # heading error, wrapped to [-pi, pi]
float64 target_angle            # bearing to the active goal in the odom/map frame