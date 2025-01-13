This package provides a unit-tested waypoint follower node.

1. To run the tests with the default goal:

```
. /opt/ros/galactic/setup.bash
cd ~/ros2_ws
colcon build --packages-select tortoisebot_waypoints --cmake-args --cmake-clean-cache
. install/setup.bash
colcon test --packages-select tortoisebot_waypoints --event-handler=console_direct+
```

This should pass. If it fails the first time (because Gazebo fails to launch correctly), simply run the test again.

2. To change the goal to an unreachable pose, replace the arguments in the TortoisebotActionServerTests::goal initialization inside `test/tortoisebot_action_server_test.cpp` (line 64):

```
sed -i 's/g\.position\.y = 0\.3/g\.position\.y=-3\.0/g' ~/ros2_ws/src/tortoisebot_waypoints/test/tortoisebot_action_server_test.cpp
# repeat step 1 here
```

This time the test should fail. The test continuously monitors the action and fails if neither the position error nor the yaw error improves within 10s.

3. To launch only the tests without the simulation launching automatically:

Rebuild the package with the option `ENABLE_STANDALONE_GTEST` (<strong>making sure the package is rebuilt from scratch</strong>):

```
colcon build --packages-select tortoisebot_waypoints --cmake-args -DENABLE_STANDALONE_GTEST=ON --cmake-clean-cache
```

and then launch the simulation manually as well as the test similarly as before:

```
ros2 launch tortoisebot_bringup bringup.launch.py use_sim_time:=True  # terminal 1
colcon test --packages-select tortoisebot_waypoints --event-handler=console_direct+  # terminal 2
```
