#! /usr/bin/env python

import rospy
import numpy as np
import math
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from std_msgs.msg import Float32
from darknet_ros_msgs.msg import BoundingBoxes
import matplotlib.pyplot as plt
import cv2
from cv_bridge import CvBridge




def callback2(msg):

    global cv_image
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, 'bgr8')

    for i in range(len(x)):
        cv2.line(cv_image,(int(path_func[i]),int(x[i])),(int(path_func[i]),int(x[i])),(0,0,255),5)


    for i in range(len(mid_path_points_x)):
        cv2.line(cv_image,(int(mid_path_points_x[i]),int(mid_path_points_y[i])),(int(mid_path_points_x[i]),int(mid_path_points_y[i])),(0,255,255),5)
    
    image_message = bridge.cv2_to_imgmsg(cv_image, "bgr8")

    pub.publish(image_message)


def callback(msg):
    blue_x=[]
    blue_y=[]
    yellow_x=[]
    yellow_y=[]
    mid_path_points_x=[]
    mid_path_points_y=[]
    weights=[]
    global x,path_func, mid_path_points_x, mid_path_points_y
    for i in range(len(msg.bounding_boxes)):

        if msg.bounding_boxes[i].Class =="blue":
            blue_x.append((msg.bounding_boxes[i].xmin + msg.bounding_boxes[i].xmax)/2)
            blue_y.append((msg.bounding_boxes[i].ymin + msg.bounding_boxes[i].ymax)/2)

        else:
            yellow_x.append((msg.bounding_boxes[i].xmin + msg.bounding_boxes[i].xmax)/2)
            yellow_y.append((msg.bounding_boxes[i].ymin + msg.bounding_boxes[i].ymax)/2)

    for i in range(len(blue_x)):
        for j in range(len(yellow_x)):

            mid_path_points_x.append(( blue_x[i] + yellow_x[j] ) / 2)
            mid_path_points_y.append(( blue_y[i] + yellow_y[j] ) / 2)
    # mid_path_points_x.insert(0,320)
    # mid_path_points_y.insert(0,359.5)
    # mid_path_points_x.insert(0,320)
    # mid_path_points_y.insert(0,359.6)
    # mid_path_points_x.insert(0,320)
    # mid_path_points_y.insert(0,359.7)
    # mid_path_points_x.insert(0,320)
    # mid_path_points_y.insert(0,359.8)
    # mid_path_points_x.insert(0,320)
    # mid_path_points_y.insert(0,359.9)
    mid_path_points_x.insert(0,320)
    mid_path_points_y.insert(0,360)

    for i in range(len(mid_path_points_x)):
            if i < 1:
                weights.append(22)
            else :
                weights.append(1)
    x=np.linspace(250,360,10)
    p = np.polyfit(mid_path_points_y,mid_path_points_x,2,w = weights)
    path_func = p[0] * x**2 + p[1] * x + p[2]
    



    steer = path_func[0] - 320
    steer= steer*-0.001
    print(steer)



if __name__ == '__main__':

    rospy.init_node('yolo slowlap')
    rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,callback)
    rospy.Subscriber("/darknet_ros/detection_image",Image,callback2)

    pub = rospy.Publisher("/detection_image",Image,queue_size=1)

    rospy.spin()