#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from assignment_3.srv import TurnService, TurnServiceResponse


class Wheels():
    def __init__(self):
        rospy.init_node('wheels')
        self.drive_pub = rospy.Publisher(
                'mobile_base/commands/velocity',
                Twist,
                queue_size = 10
        )
        self.drive_sub = rospy.Subscriber(
                'wheels/commands',
                Twist,
                self.handle_incoming_drive_command
        )
        self.turn_service = rospy.Service(
                'turn_service',
                TurnService,
                self.handle_incoming_turn
        )
        self.drive_command = Twist()
        self.drive_rate = rospy.Rate(.5)
        self.drive_loop()

    def drive_loop(self):
        while not rospy.is_shutdown():
            self.drive_pub.publish(self.drive_command)
            self.drive_rate.sleep()

    def handle_incoming_turn(self, request):
        print "Received request to turn for ", request.time, " seconds."
        self.drive_command = request.twist
        return TurnServiceResponse(True)
    
    def handle_incoming_drive_command(self, request):
        print "Received request to drive forwart at ", request.linear.x
        self.drive_command = request


if __name__ == "__main__":
    ws = Wheels()
