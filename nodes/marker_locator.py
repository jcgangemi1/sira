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


class MarkerLocator(hm.HelloNode):
    def __init__(self, marker_name):
        hm.HelloNode.__init__(self)

        # Config
        self.rate = 10
        self.marker_name = marker_name
        self.marker_tf = None

    def marker_array_callback(self, marker_array_msg):
        rospy.logdebug("MarkerArray Message: ")
        rospy.logdebug(marker_array_msg)
        for marker in marker_array_msg.markers:
            if marker.text == self.marker_name:
                marker_transform_stamped_msg = self.get_tf('map', self.marker_name)
                rospy.logdebug("FOUND MARKER: ")
                rospy.logdebug(marker_transform_stamped_msg)
                self.marker_tf = marker_transform_stamped_msg

    def get_marker_location_callback(self, request):
        success = False
        tf_wrt_map = None
        message = "unknown error"

        if self.marker_tf is None:
            message = f"haven't seen {self.marker_name} yet"
        else:
            success = True
            tf_wrt_map = self.marker_tf
            message = ""

        return GetMarkerLocationResponse(
            success=success,
            tf_wrt_map=tf_wrt_map,
            message=message
        )

    def marker_scan_callback(self, request):
        success = False
        message = "unknown error"

        try:
            self.trigger_head_scan(TriggerRequest())
            success = True
            message = ""
        except:
            success = False
            message = "Head scanning via '/funmap/trigger_local_localization' failed"

        return TriggerResponse(
            success=success,
            message=message
        )

    def main(self):
        hm.HelloNode.main(self, 'marker_locator', 'marker_locator', wait_for_first_pointcloud=True)

        rospy.Subscriber('/aruco/marker_array', MarkerArray, self.marker_array_callback)
        rospy.Service('/marker_locator/get_marker_location',
                      GetMarkerLocation,
                      self.get_marker_location_callback)
        rospy.Service('/marker_locator/marker_scan',
                      Trigger,
                      self.marker_scan_callback)
        self.trigger_head_scan = rospy.ServiceProxy('/funmap/trigger_local_localization', Trigger)

        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("marker_name")
    args = parser.parse_args()

    MarkerLocator(args.marker_name).main()

