#! /usr/bin/env python

from tabnanny import check
import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

start_flag = False
i=0
x_start = 0
y_start = 0
x_odom = []
y_odom = []
y_path = []
qx_odom = []
qy_odom = []
qz_odom = []
qw_odom = []
#x_odom_raw=[]
#y_odom_raw=[]
def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]
def check_far_from_start(x,y,xs,ys):
    
    dist = np.sqrt((x-xs)**2 + (y-ys)**2)
    
    return dist

def callback(msg):
    global i, x_start, y_start, start_flag
    #x_odom_raw.append(msg.pose.pose.position.x)
    #y_odom_raw.append(msg.pose.pose.position.y)
    if i==0:
        x_start = msg.pose.pose.position.x
        y_start = msg.pose.pose.position.y
        x_odom.append(x_start)
        y_odom.append(y_start)
        #qx_odom.append(msg.pose.pose.orientation.x)
        #qy_odom.append(msg.pose.pose.orientation.y)
        #qz_odom.append(msg.pose.pose.orientation.z)
        #qw_odom.append(msg.pose.pose.orientation.w)
        i=i+1
        print('start')

    if check_far_from_start(msg.pose.pose.position.x,msg.pose.pose.position.y,x_start,y_start) > 0.5:
        start_flag = True

    if check_far_from_start(msg.pose.pose.position.x,msg.pose.pose.position.y,x_odom[-1],y_odom[-1]) > 0.2:
        x_odom.append(msg.pose.pose.position.x)#-962585.903566)
        y_odom.append(msg.pose.pose.position.y)#-1959238.24075)
        qx_odom.append(msg.pose.pose.orientation.x)
        qy_odom.append(msg.pose.pose.orientation.y)
        qz_odom.append(msg.pose.pose.orientation.z)
        qw_odom.append(msg.pose.pose.orientation.w)

    if start_flag == True and check_far_from_start(msg.pose.pose.position.x,msg.pose.pose.position.y,x_start,y_start) <0.5 and i==1:
    
        #p = np.polyfit(x_odom,y_odom,3)
        #x_path = np.linspace(x_odom[0],x_odom[-1],len(x_odom))
        #y_path = np.polyval(p,x_path)
        path = Path()
        path.header.frame_id = 'velodyne'
    
        for j in range(len(x_odom)-1):
            pose = PoseStamped()
            pose.pose.position.x = x_odom[j]
            pose.pose.position.y = y_odom[j]
            pose.pose.position.z = 0

            yaw = np.arctan2(y_odom[j+1]-y_odom[j],x_odom[j+1]-x_odom[j] )
            qx_,qy_,qz_,qw_ = get_quaternion_from_euler(0,0,yaw)
            #pose.pose.orientation.x = qx_odom[j]
            #pose.pose.orientation.y = qy_odom[j]
            #pose.pose.orientation.z = qz_odom[j]
            #pose.pose.orientation.w = qw_odom[j]
            pose.pose.orientation.x = qx_
            pose.pose.orientation.y = qy_
            pose.pose.orientation.z = qz_
            pose.pose.orientation.w = qw_

            path.poses.append(pose)


        path_pub.publish(path)
        print('end')
        #plt.plot(x_odom_raw,y_odom_raw)
        #plt.show()
        
        i=i+1




if __name__ == '__main__':

    rospy.init_node("odom path generator")
    rospy.Subscriber('/odom',Odometry,callback)
    path_pub = rospy.Publisher('/fastlap_path',Path,queue_size=1)

    rospy.spin()
