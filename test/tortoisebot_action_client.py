from tortoisebot_waypoints.msg import WaypointActionAction, WaypointActionGoal, WaypointActionResult
import rospy
import actionlib

class TortoisebotActionClient():
    action_server_name = 'tortoisebot_as'
    timeout_server_sec = 5.0
    timeout_result_sec = 30.0

    def __init__(self):
        self.action_client = actionlib.SimpleActionClient(self.action_server_name, WaypointActionAction)
        self.action_result = WaypointActionResult()

    def send_goal(self,goal : WaypointActionGoal):
        rospy.loginfo(f"Waiting for {self.action_server_name} action server...")
        self.action_client.wait_for_server()
        
        if not self.action_client.wait_for_server(rospy.Duration(self.timeout_server_sec)):
            rospy.logerr(f"Timeout waiting for {self.action_server_name} action server.")
            return

        rospy.loginfo(f"Sending goal x={goal.position.x} y={goal.position.y}.")
        self.action_client.send_goal(goal)
        
        if not self.action_client.wait_for_result(rospy.Duration(self.timeout_result_sec)):
            rospy.loginfo("Timeout waiting for result.")
            return
        
        rospy.loginfo("Got result.")
        self.action_result = self.action_client.get_result()
