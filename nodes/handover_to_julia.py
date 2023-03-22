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


class HandoverToJulia(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        self.rate = 10

    def trigger_handover_callback(self, request):
        success = False
        message = "unknown error"
        tf_wrt_map = None

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

        # 2. Plan and move to marker's grasp point
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

        return TriggerResponse(
            success=success,
            message=message
        )

    def main(self):
        hm.HelloNode.main(self, 'handover_to_julia', 'handover_to_julia', wait_for_first_pointcloud=False)

        rospy.Service('/handover_to_julia/trigger_handover',
                      Trigger,
                      self.trigger_handover_callback)
        self.trigger_get_marker_location = rospy.ServiceProxy(
            '/julia_locator/get_marker_location', GetMarkerLocation
        )
        self.trigger_marker_scan = rospy.ServiceProxy(
            '/julia_locator/marker_scan', Trigger
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

    HandoverToJulia().main()
