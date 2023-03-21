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
from cyra.srv import GetMarkerLocation, GetMarkerLocationRequest, GetMarkerLocationResponse


class MarkerLocator(hm.HelloNode):
    def __init__(self, marker_name):
        hm.HelloNode.__init__(self)

        # Config
        self.rate = 10
        self.marker_name = marker_name
        self.marker_tf = None

    def marker_callback(self, marker_array_msg):
        rospy.logdebug("MarkerArray Message: ")
        rospy.logdebug(marker_array_msg)
        for marker in marker_array_msg.markers:
            if marker.text == self.marker_name:
                marker_transform_stamped_msg = self.get_tf('map', self.marker_name)
                rospy.logdebug("FOUND MARKER: ")
                rospy.logdebug(marker_transform_stamped_msg)
                self.marker_tf = marker_transform_stamped_msg


    def main(self):
        hm.HelloNode.main(self, 'marker_locator', 'marker_locator', wait_for_first_pointcloud=True)

        self.marker_subscriber = rospy.Subscriber('/aruco/marker_array', MarkerArray, self.marker_callback)

        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("marker_name")
    args = parser.parse_args()

    MarkerLocator(args.marker_name).main()

