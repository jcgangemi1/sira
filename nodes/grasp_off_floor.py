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


class GraspOffFloor(hm.HelloNode):
    def __init__(self, marker_name):
        hm.HelloNode.__init__(self)
        self.rate = 10

    def trigger_grasp_marker_callback(self):
        success = False
        message = "unknown error"
        tf_wrt_map = None

        try:
            gml_response = self.trigger_get_marker_location(GetMarkerLocationRequest())
            if gml_response.success == True:
                success = True
                message = ""
                tf_wrt_map = gml_response.tf_wrt_map
            else:
                if "haven't seen" in gml_response.message:
                    # Robot hasn't searched for the marker yet
                    self.trigger_marker_scan(TriggerRequest())
                    gml_response2 = self.trigger_get_marker_location(GetMarkerLocationRequest())
                    if gml_response2.success == True:
                        success = True
                        message = ""
                        tf_wrt_map = gml_response2.tf_wrt_map
                message = "Couldn't find marker location"
        except:
            message = "Exception while searching for marker location."

        # TODO

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

        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    args = parser.parse_args()

    GraspOffFloor().main()
