#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from assignment_3.srv import TurnService
import random


class RoombaExplorer():
    def __init__(self):
        rospy.init_node('roomba_explorer')
        self.lidar_sub = rospy.Subscriber(
                '/lidar/data',
                LaserPoints,
                self.handle_incoming_laser
        )
        self.drive_command_pub = rospy.Publisher(
                'wheels/commands',
                Twist,
                queue_size = 10
        )
        self.turn_requester = rospy.ServiceProxy(
                'turn_service',
                TurnService
        )
        self.rate = rospy.Rate(1)
        self.control_loop()

    def handle_incoming_laser(self, message):
        self.raw_data = message.raw
        self.point_list = message.point_list

    def control_loop(self):
        while not rospy.is_shutdown():
            collsion = self.check_points(point_is_dangerous)
            if collision:
                self.turn_random_ammount()
            else:
                self.drive_forward()
                self.rate.sleep()

    def point_is_dangerous(self, point):
        y_buffer = 0.2
        x_buffer = 0.375
        return (abs(point[1]) < y_buffer) and (point[2] < x_buffer)

    def check_points(self, expression):
        hit = any(map(expression, self.point_list))
        return hit

    def turn_random_ammount(self):
        time = random.uniform(0, 1) + 1

        drive_command = Twist()
        drive_command.angular.z = 1.0

        turn_request = TurnServiceRequest()
        turn_request.time = time
        turn_request.twist = drive_command

        success = self.turn_requester(turn_request)

    def drive_forward(self):
        drive_command = Twist()
        drive_command.linear.x = 0.3
        self.drive_command_pub.publish(drive_command)


if __name__ == "__main__":
    rex = RoombaExplorer()
