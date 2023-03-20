#!/usr/bin/env python3

# ROS
import rospy

# Messages
from std_msgs.msg import Float32, Int8

#Triggers
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

import hello_helpers.hello_misc as hm

class MarkerLocator(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)

        # Config
        self.rate = 10

    def main(self):
        hm.HelloNode.main(self, 'marker_locator', 'marker_locator', wait_for_first_pointcloud=True)

        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    MarkerLocator().main()