#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from assignment_3.msg import Lidar
from assignment_3.srv import TurnService, TurnServiceRequest, CollisionService
from std_msgs.msg import Bool
import random


class RoombaExplorer():
    def __init__(self):
        rospy.init_node('roomba_explorer')
        self.collision_service = rospy.Service(
                'collision_service',
                CollisionService,
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
        rospy.sleep(5)
        self.drive_forward()
        rospy.spin()

    def handle_incoming_collision(self, request):
        print "incoming collision"
        if request.collision == True:
            self.turn_random_ammount()
            self.drive_forward()
        return 'Collision handled.'

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
