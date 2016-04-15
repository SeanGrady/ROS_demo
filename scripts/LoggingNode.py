#!/usr/bin/env python

import rospy
from assignment_3.msg import LogMessage


class Logger():
    def __init__(self):
        rospy.init_node('logger')
        self.log_sub = rospy.Subscriber(
                '/lidar/log',
                LogMessage,
                self.handle_log_message
        )
        rospy.spin()

    def handle_log_message(self, message):
        if message.collision == True:
            print "Imminent collision!"
            print min(message.raw_scan.ranges)
            pass


if __name__ == '__main__':
    lg = Logger()
