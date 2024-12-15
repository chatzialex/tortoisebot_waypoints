- To launch the tests alongside the simulation:

```
colcon test --packages-select tortoisebot_waypoints --event-handler=console_direct+
```

- To launch only the tests:

Rebuild the package with the option `ENABLE_STANDALONE_GTEST` (<strong>making sure the package is rebuilt from scratch</strong>):

```
colcon build --packages-select tortoisebot_waypoints --cmake-args -DENABLE_STANDALONE_GTEST=ON --cmake-clean-cache
```

and then launch the simulation manually as well as the test similarly as before:

```
ros2 launch tortoisebot_bringup bringup.launch.py use_sim_time:=True  # terminal 1
colcon test --packages-select tortoisebot_waypoints --event-handler=console_direct+  # terminal 2
```
