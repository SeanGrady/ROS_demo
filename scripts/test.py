#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan


class LaserRepeater():
    def __init__(self):
        rospy.init_node('laser_repeater')
        self.laser_sub = rospy.Subscriber(
                "scan",
                LaserScan,
                self.handle_incoming_laser
        )
        self.laser_pub = rospy.Publisher(
                "/repeater/scan",
                LaserScan,
                queue_size = 1
        )
        rospy.spin()

    def handle_incoming_laser(self, message):
        self.laser_pub.publish(message)


if __name__ == "__main__":
    lr = LaserRepeater()
