#!/usr/bin/env python3

# ROS
import rospy
import hello_helpers.hello_misc as hm
import argparse

# Messages
from std_msgs.msg import Float32, Int8
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped

#Triggers
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from sira.srv import GetMarkerLocation, GetMarkerLocationRequest, GetMarkerLocationResponse
from sira.srv import FUNMAPReachToPoint, FUNMAPReachToPointRequest, FUNMAPReachToPointResponse
from sira.srv import AddObstacle, AddObstacleRequest, AddObstacleResponse


class GraspOffFloor(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        self.rate = 10

    def trigger_grasp_marker_callback(self, request):
        success = False
        message = "unknown error"
        tf_wrt_map = None

        rospy.loginfo("moving lift up")
        self.move_to_pose({'joint_lift': 0.2})

        # 1. Find the marker location
        try:
            success = False
            message = "Couldn't find marker location"
            rospy.loginfo("first attempt to find marker")
            gml_response = self.trigger_get_marker_location(GetMarkerLocationRequest())
            if gml_response.success == True:
                rospy.loginfo("success on first attempt to find marker")
                success = True
                message = ""
                tf_wrt_map = gml_response.tf_wrt_map
            else:
                if "haven't seen" in gml_response.message:
                    # Robot hasn't searched for the marker yet
                    rospy.loginfo("head scan for marker")
                    self.trigger_marker_scan(TriggerRequest())
                    rospy.loginfo("second attempt to find marker")
                    gml_response2 = self.trigger_get_marker_location(GetMarkerLocationRequest())
                    if gml_response2.success == True:
                        rospy.loginfo("success on second attempt to find marker")
                        success = True
                        message = ""
                        tf_wrt_map = gml_response2.tf_wrt_map
        except:
            success = False
            message = "Exception while searching for marker location."

        # 2. Add virtual obstacle on marker
        if success:
            try:
                rospy.loginfo("adding virtual obstacle on marker")
                req = AddObstacleRequest()
                req.header.stamp = tf_wrt_map.header.stamp
                req.header.frame_id = tf_wrt_map.header.frame_id
                req.x_m = tf_wrt_map.transform.translation.x
                req.y_m = tf_wrt_map.transform.translation.y
                add_obstacle_response = self.trigger_add_obstacle(req)
                if add_obstacle_response.success == True:
                    rospy.loginfo("success on adding virtual obstacle on marker")
                    success = True
                    message = ""
            except:
                success = False
                message = "Exception while adding obstacle to MHI"

        # 3. Plan and move to marker's grasp point
        if success: # succeeding so far
            attempts = 0
            reach_to_point_success = False
            try:
                while reach_to_point_success == False and attempts < 3:
                    rospy.loginfo(f"funmap reaching to grasp point attempt={attempts}")
                    self.move_to_pose({'rotate_mobile_base': 0.2})
                    trigger_response = self.trigger_reach_to_point(FUNMAPReachToPointRequest(tf_wrt_map=tf_wrt_map))
                    if trigger_response.success == False:
                        reach_to_point_success = False
                        success = False
                        message = trigger_response.message
                    else:
                        reach_to_point_success = True
                        success = True
                        message = ""
                    attempts += 1
                    rospy.sleep(1)
            except:
                success = False
                message = "Exception while trying to reach to grasp point"

        # 4. Pick up hat
        if success: # succeeding so far
            self.move_to_pose({'joint_lift': 0.0115})
            rospy.sleep(1)
            self.move_to_pose({'joint_gripper_finger_left': -0.288})
            rospy.sleep(2)
            self.move_to_pose({'joint_lift': 0.5})

        return TriggerResponse(
            success=success,
            message=message
        )

    def main(self):
        hm.HelloNode.main(self, 'grasp_off_floor', 'grasp_off_floor', wait_for_first_pointcloud=False)

        rospy.Service('/grasp_off_floor/trigger_grasp_marker',
                      Trigger,
                      self.trigger_grasp_marker_callback)
        self.trigger_get_marker_location = rospy.ServiceProxy(
            '/hat_locator/get_marker_location', GetMarkerLocation
        )
        self.trigger_marker_scan = rospy.ServiceProxy(
            '/hat_locator/marker_scan', Trigger
        )
        self.trigger_reach_to_point = rospy.ServiceProxy(
            '/funmap/trigger_reach_to_point', FUNMAPReachToPoint
        )
        self.trigger_add_obstacle = rospy.ServiceProxy(
            '/funmap/trigger_add_obstacle', AddObstacle
        )

        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    args, unknown = parser.parse_known_args()

    GraspOffFloor().main()
