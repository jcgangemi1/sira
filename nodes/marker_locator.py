#!/usr/bin/env python3

# ROS
import rospy
import argparse
import hello_helpers.hello_misc as hm
from tf2_ros import StaticTransformBroadcaster

# Messages
from std_msgs.msg import Float32, Int8
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped

#Triggers
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from sira.srv import GetMarkerLocation, GetMarkerLocationRequest, GetMarkerLocationResponse


class MarkerLocator(hm.HelloNode):
    def __init__(self, marker_name):
        hm.HelloNode.__init__(self)
        self.rate = 10
        self.marker_name = marker_name
        self.marker_tf = None
        self.already_marker_scanned = False

    def marker_array_callback(self, marker_array_msg):
        rospy.logdebug("MarkerArray Message: ")
        rospy.logdebug(marker_array_msg)
        for marker in marker_array_msg.markers:
            if marker.text == self.marker_name:
                # publish static transform 2cm along x axis
                marker_offsetted = TransformStamped()
                marker_offsetted.header.stamp = rospy.Time.now()
                marker_offsetted.header.frame_id = self.marker_name
                marker_offsetted.child_frame_id = f"{self.marker_name}_grasp_point"

                marker_offsetted.transform.translation.x = 0.12
                marker_offsetted.transform.translation.y = 0.0
                marker_offsetted.transform.translation.z = 0.0
                marker_offsetted.transform.rotation.x = 0.0
                marker_offsetted.transform.rotation.y = 0.0
                marker_offsetted.transform.rotation.z = 0.0
                marker_offsetted.transform.rotation.w = 1.0
                self.br.sendTransform([marker_offsetted])

                # get grasp point
                marker_transform_stamped_msg = self.get_tf('map', f"{self.marker_name}_grasp_point")
                rospy.logdebug("FOUND MARKER: ")
                rospy.logdebug(marker_transform_stamped_msg)
                self.marker_tf = marker_transform_stamped_msg

    def get_marker_location_callback(self, request):
        success = False
        tf_wrt_map = None
        message = "unknown error"

        if self.marker_tf is None:
            if self.already_marker_scanned:
                message = f"didn't see {self.marker_name}"
            else:
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
            self.already_marker_scanned = True
        except:
            success = False
            message = "Head scanning via '/funmap/trigger_local_localization' failed"

        return TriggerResponse(
            success=success,
            message=message
        )

    def clear_saved_locations_callback(self, request):
        self.marker_tf = None
        self.already_marker_scanned = False

        return TriggerResponse(
            success=True,
            message=""
        )

    def main(self):
        hm.HelloNode.main(self, 'marker_locator', 'marker_locator', wait_for_first_pointcloud=True)

        self.br = StaticTransformBroadcaster()
        rospy.Subscriber('/aruco/marker_array', MarkerArray, self.marker_array_callback)
        rospy.Service(f'{rospy.get_name()}/get_marker_location',
                      GetMarkerLocation,
                      self.get_marker_location_callback)
        rospy.Service(f'{rospy.get_name()}/marker_scan',
                      Trigger,
                      self.marker_scan_callback)
        rospy.Service(f'{rospy.get_name()}/clear_saved_locations',
                      Trigger,
                      self.clear_saved_locations_callback)
        self.trigger_head_scan = rospy.ServiceProxy('/funmap/trigger_local_localization', Trigger)

        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("marker_name")
    args, unknown = parser.parse_known_args()

    MarkerLocator(args.marker_name).main()

