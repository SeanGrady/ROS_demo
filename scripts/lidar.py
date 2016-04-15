#!/usr/bin/env python

import rospy
import math
import random
from pprint import pprint
from sensor_msgs.msg import LaserScan


class LaserScanner():
    def __init__(self):
        rospy.init_node('laser_scanner')
        self.laser_sub = rospy.Subscriber(
                "scan",
                LaserScan,
                self.handle_incoming_laser
        )
        rospy.spin()

    def handle_incoming_laser(self, message):
        point_list = self.process_laser(message)
        pprint(point_list)

    def process_laser(self, laser_data):
        point_list = []
        for index, distance in enumerate(laser_data.ranges):
            x, y = self.transform_to_point(
                    laser_data.angle_min,
                    laser_data.angle_increment,
                    index,
                    distance
            )
            point = (x, y, distance)
            point_list.append(point)
        return point_list

    def transform_to_point(self, start_angle, angle_increment, index, distance):
        angle = start_angle + (angle_increment * index)
        x = distancei * math.cos(angle)
        y = distance * math.sin(angle)
        return x, y

if __name__ == "__main__":
    ls = LaserScanner()
