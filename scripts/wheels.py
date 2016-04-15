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
        rospy.spin()

    def handle_incoming_turn(self, request):
        print "Received request to turn for ", request.time, " seconds."
        self.drive_pub.publish(request.twist)
        rospy.sleep(request.time)
        stop_message = Twist()
        self.drive_pub.publish(stop_message)
        return TurnServiceResponse(True)
    
    def handle_incoming_drive_command(self, request):
        print "Received request to drive forwart at ", request.linear.x
        self.drive_pub.publish(request)


if __name__ == "__main__":
    ws = Wheels()
