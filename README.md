This package provides a unit-tested waypoint follower node.

1. To run the tests with the default goal:

```
source /opt/ros/noetic/setup.bash
cd ~/simulation_ws
catkin_make --only-pkg-with-deps tortoisebot_waypoints
source devel/setup.bash
rostest tortoisebot_waypoints waypoints_test.test
```

This should pass; please wait around 15sec after the robot reaches the final pose. If it fail the first time (because Gazebo fails to launch correctly), simply run the test again.

2. To change the goal to an unreachable pose, replace the arguments to the `WaypointActionGoal` constructor in `src/waypoints_test/test/waypoints_test.py` (line 30):

```
sed -i 's/\(x=\)-0.6/\1-3/' ~/simulation_ws/src/tortoisebot_waypoints/test/waypoints_test.py
# repeat step 1 here
```

This time the test should fail.

3. To launch the test without the simulation, comment out the corresponding section in `waypoints_test.test`, rebuild the package, and launch the test (after manually launching the simulation) as described above.
