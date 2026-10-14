# mbot_ros_labs

This is the repository for classes use ROB2 MBot. Each folder in this repository is a ros package. And each one of them could be a standalone challenge.

To use:
1. Make a directory
    ```bash
    cd ~
    mkdir mbot_ros_labs
    ```
2. Clone this repository as `src`
    ```bash
    cd ~/mbot_ros_labs
    gh repo clone mbot-project/mbot_ros_labs src
    ```
3. Compile the packages
    ```bash
    colcon build
    source install/setup.bash
    ```

## Event 1: autonomous square

`event_1.py` is a standalone controller that publishes directly to `/cmd_vel`.
Do not run it at the same time as another velocity controller, such as
`mbot_nav/controller_node`.

Check the route and state transitions without ROS or a robot:

```bash
python3 event_1.py --dry-run --ccw --laps 1
```

On the robot, use separate terminals to start SLAM and the controller. Verify
the isolated rotation and short route before attempting the full four laps:

```bash
source install/setup.bash
ros2 launch mbot_slam competition_slam.launch.py
```

```bash
source install/setup.bash
python3 event_1.py --rotation-test
python3 event_1.py --laps 1 --sides 1
python3 event_1.py --laps 1
python3 event_1.py --freeze-after-first
```

Each run writes a timestamped CSV under `~/mbot_logs/event_1/`. To see which
nodes currently publish velocity commands, run:

```bash
ros2 topic info /cmd_vel --verbose
```
