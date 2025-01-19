import launch
import launch_testing.markers
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.launch_description_sources import PythonLaunchDescriptionSource
import unittest
import os
import signal
from subprocess import check_output, CalledProcessError


def generate_test_description():
  headless = LaunchConfiguration('headless')
  headless_arg = launch.actions.DeclareLaunchArgument(
    name='headless',
    default_value='false',
    description='Enable headless mode for Gazebo'
  )

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
    headless_arg,
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

@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_kill_gazebo(self):
        processes_to_kill = ["gzserver", "gzclient"]
        for process_name in processes_to_kill:
            try:
                pids = check_output(["pidof", "-z", process_name])
                pids = pids.decode('utf-8').strip().split('\0')

                for pid in pids:
                    pid = int(pid)
                    os.kill(pid, signal.SIGTERM)

                    try:
                        os.waitpid(pid, 0)
                    except ChildProcessError:
                        pass

                    else:
                        try:
                            os.kill(pid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass

            except CalledProcessError:
                pass
            except Exception:
                pass
