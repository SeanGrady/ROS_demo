#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


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
                CommandService,
                self.handle_incoming_turn
        )

    def handle_incoming_command(self, request):
        print "Received request to turn for ", request.time, " seconds."
        self.drive_pub.publish(request.twist)
        self.sleep(request.time)
        stop_message = Twist()
        self.drive_pub.publish(stop_message)
        return CommandServiceResponse(True)
    
    def handle_incoming_drive_command(self, request):
        print "Received request to drive forwart at ", request.twist.linear.x
        self.drive_pub.publish(request.twist)


if __name__ == "__main__":
    ws = Wheels()
