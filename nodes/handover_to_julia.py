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

    def trigger_handover_callback(self):
        success = False
        message = "unknown error"

        return TriggerResponse(
            success=success,
            message=message
        )

    def main(self):
        hm.HelloNode.main(self, 'handover_to_julia', 'handover_to_julia', wait_for_first_pointcloud=False)

        rospy.Service('/handover_to_julia/trigger_handover',
                      Trigger,
                      self.trigger_handover_callback)

        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    args, unknown = parser.parse_known_args()

    HandoverToJulia().main()
