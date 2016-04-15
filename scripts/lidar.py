#!/usr/bin/env python

import rospy
import math
from pprint import pprint
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class LaserScanner():
    def __init__(self):
        rospy.init_node('laser_scanner')
        self.laser_sub = rospy.Subscriber(
                "scan",
                LaserScan,
                self.handle_incoming_laser
        )
        self.collision_pub = rospy.Publisher(
                '/lidar/collision',
                Bool,
                queue_size = 10
        )
        rospy.spin()

    def handle_incoming_laser(self, message):
        self.point_list = self.process_laser(message)
        collision = self.check_points(self.point_is_dangerous)
        self.collision_pub.publish(collision)

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
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        return x, y

    def point_is_dangerous(self, point):
        y_buffer = 0.4
        x_buffer = 1.0
        return (abs(point[1]) < y_buffer) and (point[2] < x_buffer)

    def check_points(self, expression):
        hit = any(map(expression, self.point_list))
        return hit


if __name__ == "__main__":
    ls = LaserScanner()
