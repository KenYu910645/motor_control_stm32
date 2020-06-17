#!/usr/bin/env python
import math
from math import sin, cos, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


class Twist2Odom():
    def __init__(self):
        rospy.init_node('odometry_publisher')
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)
        
        self.th = 0
        self.v = 0
        self.w = 0

        self.last_time = rospy.Time.now()
        rospy.Subscriber('/STM32_twist', Twist, self.get_cmd_vel)

        # while not rospy.is_shutdown():
            
        rospy.spin()

    def get_cmd_vel(self, cmd):
        self.current_time = rospy.Time.now()
        self.v = cmd.linear.x
        self.w = cmd.angular.z
        self.dt = (self.current_time - self.last_time).to_sec()
        self.delta_th = self.w * self.dt
        self.th += self.delta_th
        self.vx = self.v * cos(self.th)
        self.vy = self.v * sin(self.th)


        self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        self.odom = Odometry()
        self.odom.header.stamp = self.current_time
        self.odom.header.frame_id = "odom"
        self.odom.pose.pose = Pose(Point(0, 0, 0.), Quaternion(*self.odom_quat))
        self.odom.child_frame_id = "base_link"
        self.odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.w))
        self.odom_pub.publish(self.odom)
        rospy.loginfo(self.odom)
        self.last_time = self.current_time

if __name__ == '__main__':
    

    try:
        a = Twist2Odom()
        
    except KeyboardInterrupt:
        pass

    finally:
        pass