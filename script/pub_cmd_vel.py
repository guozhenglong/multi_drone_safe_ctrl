#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from aruco_navigation.msg import Navdata_aruco
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Vector3, Twist

def pub_pos():
    rospy.init_node('pub_navdata')
    pub_nav = rospy.Publisher('/navdata_uav', Navdata_aruco, queue_size=1)
    pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    r = rospy.Rate(20) 
    i = 0
    while not rospy.is_shutdown():
        print(i)
        cmd_input = Twist()
        cmd_input.linear.x = 0.0
        cmd_input.linear.y = 0.0
        cmd_input.linear.z = 0.2
        cmd_input.angular.x = 0.0
        cmd_input.angular.y = 0.0
        cmd_input.angular.z = 0.0

        msg_data = Navdata_aruco()
        msg_data.header = Header()
        msg_data.header.stamp = rospy.Time.now()
        msg_data.header.frame_id = 'pub_navdata'

        if i%80 < 40:  
            msg_data.pose.position.x  =  (i%40)*0.5
            msg_data.pose.position.y  =  0.0
        elif i%80 >39:
            msg_data.pose.position.x  =  0.0
            msg_data.pose.position.y  =  (i%40)*0.5

        
        msg_data.pose.position.z  =  0.0
        msg_data.pose.orientation.x  =  0.0
        msg_data.pose.orientation.y  =  0.0
        msg_data.pose.orientation.z  =  0.0
        msg_data.pose.orientation.w  =  0.0
        msg_data.velocity.x  =  0.0
        msg_data.velocity.y  =  0.0
        msg_data.velocity.z  =  0.0
        msg_data.euler_angle.x  =  0.0
        msg_data.euler_angle.y  =  0.0
        msg_data.euler_angle.z  =  0.0

        i = i+1
        pub_nav.publish(msg_data)
        pub_cmd.publish(cmd_input)
        r.sleep()

if __name__ == '__main__':
    try:
        pub_pos()
    except rospy.ROSInterruptException: pass