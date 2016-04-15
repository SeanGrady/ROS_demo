#!/usr/bin/env python

import math
from code import interact
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from assignment_3.msg import Wall
import random as rnd

"""
def callback(data):
    wmsg = Wall()
    wmsg.start = data.angle_min
    wmsg.end = data.angle_max
    pub.publish(wmsg)
"""

class Roomba_Explorer():
    def __init__(self, drive_mode="random"):
        self.seen_wall = 0   
        self.turn_count = 0
        rospy.init_node('roomba_explorer')
        self.r = rospy.Rate(1)
        rospy.Subscriber("scan", LaserScan, self.receive_laser)
        self.pub = rospy.Publisher("walls", Wall, queue_size=10)
        self.vpub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.drive_mode = drive_mode
        rospy.spin()        

    def receive_laser(self, data):
        self.point_cloud = self.process_laser(data)
        self.drive_robot()

    def drive_robot(self):
        if self.drive_mode == "follow_wall":
            self.follow_wall()
        if self.drive_mode == "random":
            self.drive_randomly()

    def drive_randomly(self):
        if self.check_range(self.near_wall):
            vmsg = Twist()
            if not self.seen_wall:
                self.turn_count = rnd.randint(4,12)
                self.seen_wall = True
                self.angle_sign = rnd.choice([-1, 1])
            vmsg.angular.z = (self.angle_sign * 3)
            self.vpub.publish(vmsg)
        else:
            if self.turn_count > 0:
                dmsg = Twist()
                dmsg.angular.z = 3*self.angle_sign
                self.vpub.publish(dmsg)
                self.turn_count -= 1
            else:
                self.seen_wall = False
                dmsg = Twist()
                dmsg.linear.x = 0.3
                self.vpub.publish(dmsg)

    def check_range(self, expression):
        return any(map(expression, self.point_cloud))
        """
        for point in self.point_cloud:
            if expression(point):
                return True
        return False
        """

    def imminent_collision(self, point):
        y_buffer = 0.2
        dist_buffer = 0.375
        return (abs(point[1]) < y_buffer) and (point[2] < dist_buffer)

    def near_wall(self, point):
        dist_buffer = 0.375
        return point[2] < dist_buffer

    def follow_wall(self):
        if self.check_range(self.imminent_collision):
            #rospy.loginfo('Collision warning!')
            vmsg = Twist()
            vmsg.angular.z = -1.0
            self.vpub.publish(vmsg)
        else:
            #rospy.loginfo('No collision')
            dmsg = Twist()
            dmsg.linear.x = 0.2
            dmsg.angular.z = 0.35
            self.vpub.publish(dmsg)

    '''
    def imminent_collision(self):
        y_buffer = 0.2
        dist_buffer = 0.375
        for point in self.point_cloud:
            if (abs(point[1]) < y_buffer) and (point[2] < dist_buffer):
                #output = "%f, %f, %f, %f" % (point[1], y_buffer, point[2], dist_buffer)
                #rospy.loginfo(output)
                return True
        #interact(local=locals())
        return False

    def near_wall(self):
        dist_buffer = 0.375
        for point in self.point_cloud:
            if point[2] < dist_buffer:
                #output = "%f, %f, %f, %f" % (point[1], y_buffer, point[2], dist_buffer)
                #rospy.loginfo(output)
                return True
        #interact(local=locals())
        return False
    '''

    def process_laser(self, data):
#        ind_max, val_max = max(enumerate(data.ranges),  key=operator.itemgetter(1))
#        ind_min, val_min = min(enumerate(data.ranges),  key=operator.itemgetter(1))

        start_angle = data.angle_min
        angle_increment = data.angle_increment
        point_cloud = []
        for index in range(len(data.ranges)):
            dist = data.ranges[index]
            x, y = self.convert_laser_point(start_angle, angle_increment, index, dist)
            point_cloud.append((x, y, dist))

        return point_cloud

    def convert_laser_point(self, start_angle, angle_increment, index, dist):
        angle = start_angle + (angle_increment * index)
        x = dist*math.cos(angle)
        y = dist*math.sin(angle)
        return x, y
        
"""
    def explore(self):
        dmsg = Twist()
        while not rospy.is_shutdown():
            dmsg.linear.x = 0.5
            self.vpub.publish(dmsg)
            self.r.sleep()
"""

if __name__ == '__main__':
    bot = Roomba_Explorer()

