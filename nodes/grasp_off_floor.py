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
from cyra.srv import GetMarkerLocation, GetMarkerLocationRequest, GetMarkerLocationResponse
from cyra.srv import FUNMAPReachToPoint, FUNMAPReachToPointRequest, FUNMAPReachToPointResponse


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

        try:
            rospy.loginfo("first attempt to find marker")
            gml_response = self.trigger_get_marker_location(GetMarkerLocationRequest())
            if gml_response.success == True:
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
                        success = True
                        message = ""
                        tf_wrt_map = gml_response2.tf_wrt_map
                message = "Couldn't find marker location"
        except:
            message = "Exception while searching for marker location."

        try:
            rospy.loginfo("funmap reaching to grasp point")
            trigger_response = self.trigger_reach_to_point(FUNMAPReachToPointRequest(tf_wrt_map=tf_wrt_map))
            if trigger_response.success == False:
                message = trigger_response.message
        except:
            message = "Exception while trying to reach to grasp point"

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
            '/marker_locator/get_marker_location', GetMarkerLocation
        )
        self.trigger_marker_scan = rospy.ServiceProxy(
            '/marker_locator/marker_scan', Trigger
        )
        self.trigger_reach_to_point = rospy.ServiceProxy(
            '/funmap/trigger_reach_to_point', FUNMAPReachToPoint
        )

        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    args, unknown = parser.parse_known_args()

    GraspOffFloor().main()
