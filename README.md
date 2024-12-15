- To launch the test alongside the simulation:

```
colcon test tortoisebot_waypoints waypoints_test.test
```

If this fails the first time (because Gazebo fails to launch correctly), simply run the test again.

- To launch the test without the simulation, comment out the corresponding section in `waypoints_test.test`, rebuild the package, and launch the test (after manually launching the simulation) as described above.
