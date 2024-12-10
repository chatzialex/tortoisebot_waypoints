import launch
import launch_testing.markers
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import unittest


def generate_test_description():
  simulation_launch_file = os.path.join(
    get_package_share_directory('tortoisebot_bringup'),
    'launch',
    'bringup.launch.py',
  )
  simulation_launch_action = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(simulation_launch_file),
    launch_arguments={'use_sim_time': 'True'}.items()
  )

  gtest_file = os.path.join(
    get_package_prefix('tortoisebot_waypoints'),
    'lib',
    'tortoisebot_waypoints',
    'tortoisebot_waypoints_test'
  )
  gtest_action = launch_testing.actions.gtest.GTest(
    path=gtest_file,
    name='tortoisebot_waypoints_test',
    output='screen'
  )

  return LaunchDescription([
    simulation_launch_action,
    launch.actions.TimerAction(
        period=10.0, actions=[launch_testing.actions.ReadyToTest()]
    ),
    gtest_action,
   ]), {
     "gtest_action": gtest_action
   }


class TestGTestWaitForCompletion(unittest.TestCase):
    def test_gtest_run_complete(self, gtest_action):
        self.proc_info.assertWaitForShutdown(gtest_action, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    def test_gtest_pass(self, proc_info, gtest_action):
        launch_testing.asserts.assertExitCodes(proc_info, process=gtest_action)
