#!/usr/bin/env python

from glob import glob
import rospy
import struct
import math
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseArray
import sensor_msgs.point_cloud2 as pc2

import numpy as np

minimum_points =[]
frame='tmp'
frame_blue='tmp'

def callback2(msg):

    global cone_centers
    cone_centers = PoseArray()
    cone_centers = msg




def callback3(msg):

    global minimum_points,frame_blue

    frame_blue = msg.header.frame_id[-1]
    
    # fields = [PointField('x', 0, PointField.FLOAT32, 1),
    #         PointField('y', 4, PointField.FLOAT32, 1),
    #         PointField('z', 8, PointField.FLOAT32, 1),
    #         PointField('rgb', 12, PointField.UINT32, 1),
    #         PointField('intensity', 16, PointField.FLOAT32, 1)]
    # if msg.header.frame_id[-1] == '0':
    #     sorted_point_pub.publish(pc2.create_cloud(sorted_points.header,fields,minimum_points))
    #     del minimum_points[:]


    cone_x = []
    cone_y = []
    cone_z = []

    for p in pc2.read_points(msg , field_names = ("x","y","z"), skip_nans = True):
        if msg.header.frame_id[0] == 'b':
            cone_x.append(p[0])
            cone_y.append(p[1])
            cone_z.append(p[2])

    if (len(cone_centers.poses)) > 0 :
        for i in range(len(cone_centers.poses)):
            for j in range(len(cone_x)):
                
                if math.sqrt((cone_x[j]-cone_centers.poses[i].position.x)**2 + (cone_y[j]-cone_centers.poses[i].position.y)**2) < 0.5 :
                    pt = [cone_centers.poses[i].position.x,cone_centers.poses[i].position.y,0,0,0]
                    r = int(0 * 255.0)
                    g = int(0 * 255.0)
                    b = int(1 * 255.0)
                    a = 255
                    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                    pt[3] = rgb
                    pt[4] = 1

                    minimum_points.append(pt)

                    break

    sorted_points.header = msg.header   
    sorted_points.header.stamp = rospy.Time.now() 
    sorted_points.header.frame_id = 'velodyne'
    
def callback(msg):

    global minimum_points,frame

    frame = msg.header.frame_id[-1]

    # fields = [PointField('x', 0, PointField.FLOAT32, 1),
    #         PointField('y', 4, PointField.FLOAT32, 1),
    #         PointField('z', 8, PointField.FLOAT32, 1),
    #         PointField('rgb', 12, PointField.UINT32, 1),
    #         PointField('intensity', 16, PointField.FLOAT32, 1)]
    # if msg.header.frame_id[-1] == '0':
    #     sorted_point_pub.publish(pc2.create_cloud(sorted_points.header,fields,minimum_points))
    #     del minimum_points[:]

    cone_x = []
    cone_y = []
    cone_z = []

    for p in pc2.read_points(msg , field_names = ("x","y","z"), skip_nans = True):
        if msg.header.frame_id[0] == 'y':
            cone_x.append(p[0])
            cone_y.append(p[1])
            cone_z.append(p[2])

    if (len(cone_centers.poses)) > 0 :
        for i in range(len(cone_centers.poses)):
            for j in range(len(cone_x)):
                
                if math.sqrt((cone_x[j]-cone_centers.poses[i].position.x)**2 + (cone_y[j]-cone_centers.poses[i].position.y)**2) < 0.5 :
                    pt = [cone_centers.poses[i].position.x,cone_centers.poses[i].position.y,0,0,0]
                    r = int(1 * 255.0)
                    g = int(1 * 255.0)
                    b = int(0 * 255.0)
                    a = 255
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

    sorted_point_pub = rospy.Publisher("/sorted_points2",PointCloud2,queue_size=1)
    cone_centers = PoseArray()
    sorted_points= PointCloud2()
    
    while not rospy.is_shutdown():
        rospy.Subscriber("/id_points2",PointCloud2,callback)
        rospy.Subscriber("/id_points2_blue",PointCloud2,callback3)
        rospy.Subscriber("/adaptive_clustering/poses",PoseArray,callback2)
        
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1),
            PointField('intensity', 16, PointField.FLOAT32, 1)]
        if frame == '0' or frame_blue == '0':
            sorted_point_pub.publish(pc2.create_cloud(sorted_points.header,fields,minimum_points))
            del minimum_points[:]

        rospy.sleep(0.5)




# #!/usr/bin/env python
# # PointCloud2 color cube
# import rospy
# import struct

# from sensor_msgs import point_cloud2
# from sensor_msgs.msg import PointCloud2, PointField
# from std_msgs.msg import Header


# rospy.init_node("create_cloud_xyzrgb")
# pub = rospy.Publisher("point_cloud2", PointCloud2, queue_size=2)

# points = []
# lim = 8
# for i in range(lim):
#     for j in range(lim):
#         for k in range(lim):
#             x = float(i) / lim
#             y = float(j) / lim
#             z = float(k) / lim
#             pt = [x, y, z, 0]
#             r = int(x * 255.0)
#             g = int(y * 255.0)
#             b = int(z * 255.0)
#             a = 255
#             print r, g, b, a
#             rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
#             print hex(rgb)
#             pt[3] = rgb
#             points.append(pt)

# fields = [PointField('x', 0, PointField.FLOAT32, 1),
#           PointField('y', 4, PointField.FLOAT32, 1),
#           PointField('z', 8, PointField.FLOAT32, 1),
#           PointField('rgb', 16, PointField.UINT32, 1),
#           ]

# header = Header()
# header.stamp = rospy.Time.now()
# header.frame_id = "map"
# pc2 = point_cloud2.create_cloud(header, fields, points)
# while not rospy.is_shutdown():
#     pc2.header.stamp = rospy.Time.now()
#     pub.publish(pc2)
#     rospy.sleep(1.0)