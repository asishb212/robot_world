#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf import transformations
import time
def posecallback(message):
    global x,y,yaw
    x=message.pose.pose.x
    y=message.pose.pose.y
    z=message.pose.pose.z
    w=message.pose.pose.w
    yaw=transformations.euler_from_quaternion(x,y,z,w)[0]
def pctrl(vel_pub,x_goal,y_goal):
    global x,y,yaw
    veloc_msg=Twist()
    while True:
        lin_const=0.5
        dist_rem=math.sqrt((x_goal-x)**2+(y_goal-y)**2)
        lin_speed=lin_const*dist_rem

        ang_const=4.0
        req_angle=math.atan2(y_goal-y,x_goal-x)
        ang_speed=ang_const*(req_angle-yaw)

        veloc_msg.linear.x=lin_speed
        veloc_msg.angular.z=ang_speed

        vel_pub.publish(veloc_msg)
        rospy.loginfo("current pos {0},{1} dist_remain {2}".format(x,y,dist_rem))
        if dist_rem<0.1: #permissable error 
            break

rospy.init_node('turtlesim_motion_pose', anonymous=True)

#declare velocity publisher
cmd_vel_topic='/cmd_vel'
velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

position_topic = "/odom"
pose_subscriber = rospy.Subscriber(position_topic, Odometry, posecallback) 
time.sleep(2)

x_goal=4
y_goal=4
pctrl(velocity_publisher,x_goal,y_goal)