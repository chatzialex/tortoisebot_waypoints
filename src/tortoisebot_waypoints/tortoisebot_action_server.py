#! /usr/bin/env python3
import rospy
import time
import actionlib

from tortoisebot_waypoints.msg import WaypointActionFeedback, WaypointActionResult, WaypointActionAction
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import math

def normalize_angle(angle):
    # Normalize angle to be within [-pi, pi)
    normalized_angle = (angle + math.pi) % (2 * math.pi) - math.pi
    return normalized_angle

class WaypointActionClass(object):

    # create messages that are used to publish feedback/result
    _feedback = WaypointActionFeedback()
    _result = WaypointActionResult()

    # topics
    _pub_cmd_vel = None
    _sub_odom = None

    # go to point vars
    # robot state variables
    _position = Point()
    _yaw = 0
    # machine state
    _state = 'idle'
    # goal
    _des_pos = Point()
    _des_yaw = 0.0
    # parameters
    _yaw_precision = math.pi / 90 # +/- 2 degree allowed
    _dist_precision = 0.05
    _max_angular_vel_error_yaw = 10*_yaw_precision  # [rad] 20
    _max_speed_error_pos = _dist_precision  # [m]
    _max_angular_vel = 0.65  # [rad/s]
    _max_speed = 0.1  # [m/s]

    def __init__(self):
        # creates the action server
        self._as = actionlib.SimpleActionServer("tortoisebot_as", WaypointActionAction, self.goal_callback, False)
        self._as.start()

        # define a loop rate
        self._rate = rospy.Rate(25)

        # topics
        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.wait_for_message('/odom', Odometry)
        self._sub_odom = rospy.Subscriber('/odom', Odometry, self._clbk_odom)
        rospy.loginfo("Action server started")

    def _clbk_odom(self, msg):
        # position
        self._position = msg.pose.pose.position

        # yaw
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self._yaw = euler[2]

    def goal_callback(self, goal):
        rospy.loginfo("goal %s received" % str(goal))

        # helper variables
        success = True

        # define desired position
        self._des_pos = goal.position
        self._des_yaw = goal.yaw

        # perform task
        while success:
            # update vars
            err_pos = math.sqrt(pow(self._des_pos.y - self._position.y, 2) + pow(self._des_pos.x - self._position.x, 2))
            if err_pos > self._dist_precision:
                desired_yaw = math.atan2(self._des_pos.y - self._position.y, self._des_pos.x - self._position.x)
            else:
                desired_yaw = self._des_yaw
            err_yaw = normalize_angle(desired_yaw - self._yaw)
            rospy.loginfo("Current Yaw: %s" % str(self._yaw))
            rospy.loginfo("Desired Yaw: %s" % str(desired_yaw))
            rospy.loginfo("Error Yaw: %s" % str(err_yaw))
            # logic goes here
            if err_pos <= self._dist_precision and math.fabs(err_yaw) <= self._yaw_precision:
                break
            elif self._as.is_preempt_requested():
                # cancelled
                rospy.loginfo("The goal has been cancelled/preempted")
                self._as.set_preempted()
                success = False
            elif math.fabs(err_yaw) > self._yaw_precision:
                # fix yaw
                rospy.loginfo("fix yaw")
                self._state = 'fix yaw'
                twist_msg = Twist()
                twist_msg.angular.z = max(min(err_yaw*self._max_angular_vel/self._max_angular_vel_error_yaw, self._max_angular_vel), -self._max_angular_vel)
                rospy.loginfo("publishing w_z: %s" % str(twist_msg.angular.z))
                self._pub_cmd_vel.publish(twist_msg)
            else:
                # go to point
                rospy.loginfo("go to point")
                self._state = 'go to point'
                twist_msg = Twist()
                twist_msg.linear.x = max(min(err_pos*self._max_speed/self._max_speed_error_pos, self._max_speed), -self._max_speed)
                twist_msg.angular.z = max(min(err_yaw*self._max_angular_vel/self._max_angular_vel_error_yaw, self._max_angular_vel), -self._max_angular_vel)
                self._pub_cmd_vel.publish(twist_msg)

            # send feedback
            self._feedback.position = self._position
            self._feedback.yaw = self._yaw
            self._feedback.state = self._state
            self._as.publish_feedback(self._feedback)

            # loop rate
            self._rate.sleep()

        # stop
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        self._pub_cmd_vel.publish(twist_msg)

        # return success
        if success:
            self._result.success = True
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('tortoisebot_as')
    WaypointActionClass()
    rospy.spin()
