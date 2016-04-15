#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from assignment_3.msg import Lidar
from assignment_3.srv import TurnService, TurnServiceRequest
from std_msgs.msg import Bool
import random


class RoombaExplorer():
    def __init__(self):
        rospy.init_node('roomba_explorer')
        self.collision_sub = rospy.Subscriber(
                '/lidar/collision',
                Bool,
                self.handle_incoming_collision
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
        self.collision = False
        self.control_loop()

    def handle_incoming_collision(self, message):
        self.collision = message.data

    def control_loop(self):
        while not rospy.is_shutdown():
            if self.collision:
                self.turn_random_ammount()
            else:
                self.drive_forward()
                self.rate.sleep()


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
        drive_command.linear.x = .5
        self.drive_command_pub.publish(drive_command)


if __name__ == "__main__":
    rex = RoombaExplorer()
