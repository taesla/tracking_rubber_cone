#!/usr/bin/env python

import rospy
import struct
import math
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseArray
import sensor_msgs.point_cloud2 as pc2
import message_filters

import numpy as np

minimum_points =[]
frame='tmp'
frame_blue='tmp'


def callback(msg,msg_blue,cone_centers):

    global frame, frame_blue
    frame = msg.header.frame_id
    frame_blue = msg_blue.header.frame_id
    print(1)
    # fields = [PointField('x', 0, PointField.FLOAT32, 1),
    #         PointField('y', 4, PointField.FLOAT32, 1),
    #         PointField('z', 8, PointField.FLOAT32, 1),
    #         PointField('rgb', 12, PointField.UINT32, 1),
    #         PointField('intensity', 16, PointField.FLOAT32, 1)]
    # if msg.header.frame_id[-1]=='0' or msg_blue.header.frame_id[-1] == '0':
    #     # print(minimum_points)
    #     sorted_point_pub.publish(pc2.create_cloud(sorted_points.header,fields,minimum_points))
    #     del minimum_points[:]


    cone_x = []
    cone_y = []
    cone_z = []
    cone_frame=[]

    for p in pc2.read_points(msg , field_names = ("x","y","z"), skip_nans = True):
        if msg.header.frame_id[0]=='y':
            cone_x.append(p[0])
            cone_y.append(p[1])
            cone_z.append(p[2])
            cone_frame.append('y')

    for p in pc2.read_points(msg_blue , field_names = ("x","y","z"), skip_nans = True):
        if msg_blue.header.frame_id[0]=='b':
            cone_x.append(p[0])
            cone_y.append(p[1])
            cone_z.append(p[2])
            cone_frame.append('b')

    if (len(cone_centers.poses)) > 0 :
        for i in range(len(cone_centers.poses)):
            for j in range(len(cone_x)):
                
                if math.sqrt((cone_x[j]-cone_centers.poses[i].position.x)**2 + (cone_y[j]-cone_centers.poses[i].position.y)**2) < 0.5 :
                    pt = [cone_centers.poses[i].position.x,cone_centers.poses[i].position.y,0,0,0]
                    if cone_frame[j]=='b' :
                        r = int(0 * 255.0)
                        g = int(0 * 255.0)
                        b = int(1 * 255.0)
                        a = 255
                    elif cone_frame[j]=='y' :
                        r = int(1 * 255.0)
                        g = int(1 * 255.0)
                        b = int(0 * 255.0)
                        a = 255
                    else:
                        break

                    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                    pt[3] = rgb
                    pt[4] = 1

                    minimum_points.append(pt)
                    
                    break

    sorted_points.header = msg.header   
    sorted_points.header.stamp = rospy.Time.now() 
    sorted_points.header.frame_id = 'velodyne'

    



if __name__=='__main__':

    rospy.init_node('cone_distance')

    sorted_point_pub = rospy.Publisher("/sorted_points2",PointCloud2,queue_size=10)

    cone_centers = PoseArray()
    sorted_points= PointCloud2()

    while not rospy.is_shutdown(): 
        id_sub = message_filters.Subscriber("/id_points2",PointCloud2)
        id_blue_sub = message_filters.Subscriber("/id_points2_blue",PointCloud2)
        clus_sub = message_filters.Subscriber("/adaptive_clustering/poses",PoseArray)
        ts = message_filters.ApproximateTimeSynchronizer([id_sub,id_blue_sub,clus_sub], 100,100)
        ts.registerCallback(callback)

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('rgb', 12, PointField.UINT32, 1),
                PointField('intensity', 16, PointField.FLOAT32, 1)]
        if frame[-1]=='0' or frame_blue[-1] == '0':
            # print(minimum_points)
            sorted_point_pub.publish(pc2.create_cloud(sorted_points.header,fields,minimum_points))
            del minimum_points[:]

        rospy.sleep(0.1)
    

