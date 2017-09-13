#!/bin/env python
import rospy
import numpy as np
import roslib
import tf
import math
from math import cos, sin,tan
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix as nav
from geometry_msgs.msg import TwistWithCovarianceStamped as gvel
from GaussProjection import *
from nav_msgs.msg import Odometry as odom
# global ve,vn,vd
# global angx,angy,angz
# global ax,ay,az
# global vx,vy,vz
# global r,p,y

# def on_new_imu(data):
#     global ve,vn,vd
#     global angx,angy,angz
#     global ax,ay,az
#     global vx,vy,vz
#     global r,p,y
#     #print 'recv imu'   
#     quaternion=(data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w)
#     r,p,y = tf.transformations.euler_from_quaternion(quaternion)
#     vn2vb()
#     #heading need check
#     #print r,p,math.degrees(y)  
    
# def on_new_gvel(data):
#     global ve,vn,vd
#     global angx,angy,angz
#     global ax,ay,az
#     global vx,vy,vz
#     global r,p,y
#     #print 'recv gvel'
#     ve = data.twist.twist.linear.x
#     vn = data.twist.twist.linear.y
#     vd = data.twist.twist.linear.z
#     #print ve,-vn,-vd
    

# def vn2vb():
#     global ve,vn,vd
#     global vx,vy,vz
#     global r,p,y
    
#     v=np.array([[ve],[vn],[vd]])
#     e=np.array([[cos(y)*cos(p), sin(y)*cos(p), -sin(p)], 
#         [cos(y)*sin(p)*sin(r)-sin(y)*cos(r),sin(y)*sin(p)*sin(r)+cos(y)*cos(r), cos(p)*sin(r) ],
#         [cos(y)*sin(p)*cos(r)+sin(y)*sin(r), sin(y)*sin(p)*cos(r)-cos(y)*sin(r), cos(p)*cos(r)]])
#     vk=np.dot(e,v)
#     print vk

def on_new_imu(data):
    #print 'recv imu'   
    quaternion=(data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w)
    r,p,y = tf.transformations.euler_from_quaternion(quaternion)
    # print (r,p,y)
    # while True:
    #     i = 1
    #     i +=1

def on_new_fix(fix):
    global odom_pub
    #print data.latitude, data.longitude,data.altitude
    t = LBtoxy(fix.longitude,fix.latitude)
    t.calculateAll()
    new_odom = odom()
    new_odom.header.stamp = fix.header.stamp
    #new_odom.header.frame_id = fix.header.frame_id
    new_odom.header.frame_id = 'map'
    #new_odom.child_frame_id = fix.header.frame_id
    new_odom.pose.pose.position.x = t.myx-243934.489143
    new_odom.pose.pose.position.y = t.myy-2554139.91852
    new_odom.pose.pose.position.z = fix.altitude

    new_odom.pose.pose.orientation.x = 0
    new_odom.pose.pose.orientation.y = 0
    new_odom.pose.pose.orientation.z = 0
    new_odom.pose.pose.orientation.w = 1
    covariance = [fix.position_covariance[0],\
                    fix.position_covariance[1],\
                    fix.position_covariance[2],\
                    0,0,0,\
                    fix.position_covariance[3],\
                    fix.position_covariance[4],\
                    fix.position_covariance[5],\
                    0,0,0,\
                    fix.position_covariance[6],\
                    fix.position_covariance[7],\
                    fix.position_covariance[8],\
                    0,0,0,\
                    0,0,0, 9999,0,0,\
                    0,0,0,0,9999,0,\
                    0,0,0,0,0,9999\
                ]
    new_odom.pose.covariance = covariance
    odom_pub.publish(new_odom)
    #print float(t.myy-243934.489143),float(t.myx-2554139.91852), float(0.0)




rospy.init_node('imu2vel',anonymous=True)
#rospy.Subscriber('/imu/data',Imu,on_new_imu)
#rospy.Subscriber('/gps/vel',gvel,on_new_gvel)
odom_pub = rospy.Publisher('/odom/gps', odom, queue_size=10)
rospy.Subscriber('/imu/data',Imu,on_new_imu)
rospy.Subscriber('/gps/fix',nav,on_new_fix)
print ("init finish, listening /gps/fix")

rospy.spin()
