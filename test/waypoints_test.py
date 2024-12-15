#! /usr/bin/env python

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point
from tortoisebot_waypoints.msg import WaypointActionGoal
from tf.transformations import euler_from_quaternion
from tortoisebot_action_client import TortoisebotActionClient
import rospy
import unittest
import rostest
import math

PKG = 'tortoisebot_waypoints'
NAME = 'waypoints_test'

def compute_yaw(orientation : Quaternion) -> float:
    orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
    _, _, yaw = euler_from_quaternion (orientation_list)
    return yaw

def compute_planar_distance(point1 : Point, point2: Point) -> float:
    return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

def normalize_angle(angle):
    # Normalize angle to be within [-pi, pi)
    normalized_angle = (angle + math.pi) % (2 * math.pi) - math.pi
    return normalized_angle

class TestWaypointsActionServer(unittest.TestCase):
    _action_goal = WaypointActionGoal(position=Point(x=-0.6, y=-0.5), yaw=0.0)
    _dist_precision = 0.05
    _yaw_precision = math.pi / 90 # +/- 2 degree allowed
    
    def setUp(self):
        rospy.init_node('waypoints_test')
        rospy.wait_for_message('/odom', Odometry)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.yaw = 0
        self.position = Point()
        self.action_client_wrapper = TortoisebotActionClient()
        self.action_client_wrapper.send_goal(self._action_goal)

    def odom_callback(self, msg):
        self.yaw = compute_yaw(msg.pose.pose.orientation)
        self.position = msg.pose.pose.position

    def test_end_position(self):
        distance_from_goal = compute_planar_distance(self.position, self._action_goal.position)
        self.assertTrue(
            distance_from_goal <= self._dist_precision,
            f"Robot too far from goal ({distance_from_goal} > {self._dist_precision})"
        )

    def test_end_orientation(self):
        error_yaw = abs(normalize_angle(self.yaw - self._action_goal.yaw))
        self.assertTrue(
            error_yaw <= self._yaw_precision,
            f"Robot orientation too far from goal ({error_yaw} > {self._yaw_precision})"
        )


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestWaypointsActionServer)
