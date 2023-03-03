#!/usr/bin/env python

import rospy
import struct
import math
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import PoseArray
import sensor_msgs.point_cloud2 as pc2

import numpy as np

minimum_points =[]

def callback2(msg):

    global cone_centers
    cone_centers = PoseArray()
    for i in range(len(msg.poses)):
        if ((msg.poses[i].position.x>-0.8) and (msg.poses[i].position.x<0.8) and (msg.poses[i].position.y>-0.8) and (msg.poses[i].position.y<0.8)):
            continue
        else:
            cone_centers.poses.append(msg.poses[i])
            continue


def callback(msg):
    global cone_centers
    if msg.header.frame_id[0]=='b':
        cone_number = msg.header.frame_id[4]
    if msg.header.frame_id[0]=='y':
        cone_number = msg.header.frame_id[6]
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1),
            PointField('intensity', 16, PointField.FLOAT32, 1)]
    if cone_number == '0':
        # print('publish')
        if not len(minimum_points) == 0:
            sorted_point_pub.publish(pc2.create_cloud(sorted_points.header,fields,minimum_points))
            del minimum_points[:]


    cone_x = []
    cone_y = []
    cone_z = []

    cone_dist = []
    for p in pc2.read_points(msg , field_names = ("x","y","z"), skip_nans = True):
        cone_x.append(p[0])
        cone_y.append(p[1])
        cone_z.append(p[2])
    if (len(cone_centers.poses)) > 0 :
        for i in range(len(cone_centers.poses)):
            for j in range(len(cone_x)):
                if math.sqrt((cone_x[j]-cone_centers.poses[i].position.x)**2 + (cone_y[j]-cone_centers.poses[i].position.y)**2) < 0.2 :
                    pt = [cone_centers.poses[i].position.x,cone_centers.poses[i].position.y,0,0,0]
                    if msg.header.frame_id[0]=='b':
                        r = int(0 * 255.0)
                        g = int(0 * 255.0)
                        b = int(1 * 255.0)
                        a = 255
                    else:
                        r = int(1 * 255.0)
                        g = int(1 * 255.0)
                        b = int(0 * 255.0)
                        a = 255
                    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                    pt[3] = rgb
                    pt[4] = 1

                    minimum_points.append(pt)
                    
                    break

    # print(msg.header.frame_id)
    sorted_points.header = msg.header   
    sorted_points.header.stamp = rospy.Time.now() 
    sorted_points.header.frame_id = 'velodyne'

    



if __name__=='__main__':

    rospy.init_node('cone_distance')
    rospy.Subscriber("/id_points2",PointCloud2,callback)
    rospy.Subscriber("/adaptive_clustering/poses",PoseArray,callback2)
    sorted_point_pub = rospy.Publisher("/sorted_points1",PointCloud2,queue_size=1)
    
    cone_centers = PoseArray()
    sorted_points= PointCloud2()

    rospy.spin()
