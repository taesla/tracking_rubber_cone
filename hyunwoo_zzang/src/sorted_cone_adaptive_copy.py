#!/usr/bin/env python

import rospy
import struct
import math
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseArray
import sensor_msgs.point_cloud2 as pc2

import numpy as np
import message_filters
from std_msgs.msg import Int32, Float32

minimum_points =[]
cone_centers = PoseArray()

def calposition(cone_pose,cone_pc_x,cone_pc_y,cone_pc_z,hd_msg):

    
    cone_centers.poses = cone_pose  
    cone_x = cone_pc_x
    cone_y = cone_pc_y
    cone_z = cone_pc_z
    for i in range(len(cone_centers.poses)):
        for j in range(len(cone_x)):
        
            if math.sqrt((cone_x[j]-cone_centers.poses[i].position.x)**2 + (cone_y[j]-cone_centers.poses[i].position.y)**2) < 0.5 :
                pt = [cone_centers.poses[i].position.x,cone_centers.poses[i].position.y,0,0,0]
                if len(hd_msg)==5:
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
                #print(len(cone_x), j )
    

                break
    return minimum_points
                
    

def callback(pose_sub,pc_sub):

    #cone_centers
    #cone_centers = PoseArray()
 
    sorted_points= PointCloud2()    
    
    
    cone_number = pc_sub.header.frame_id[-1]
    print(cone_number)
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1),
            PointField('intensity', 16, PointField.FLOAT32, 1)]
    if cone_number == '0':
        sorted_points.header = pc_sub.header   
        sorted_points.header.stamp = rospy.Time.now() 
        sorted_points.header.frame_id = 'velodyne'
        sorted_point_pub.publish(pc2.create_cloud(sorted_points.header,fields,minimum_points))

        del minimum_points[:]


    cone_x = []
    cone_y = []
    cone_z = []

    cone_centers = pose_sub
    cone_dist = []
    for p in pc2.read_points(pc_sub , field_names = ("x","y","z"), skip_nans = True):
        cone_x.append(p[0])
        cone_y.append(p[1])
        cone_z.append(p[2])
    #i=0
    cnt = len(cone_centers.poses)
    hd_msg = pc_sub.header.frame_id
    if (cnt):
        calposition(cone_centers.poses,cone_x,cone_y,cone_z,hd_msg)
                    
                    #i=i+1
                    # continue



    
    #print("publish:",minimum_points)


    # print("[minimum_points_header]:", hd_msg)
    # #print("[minimum_points_x]:",sorted_points.header)
    # print("[minimum_points_y]:",pc2.create_cloud(sorted_points.header,fields,minimum_points))
    # print("[minimum_points_z]:",minimum_points[0][2])
    # print("[minimum_points_rgb]:",minimum_points[0][3])

    



if __name__=='__main__':

    rospy.init_node('cone_distance')
    # Subscribers
    
    pose_sub = message_filters.Subscriber('/adaptive_clustering/poses', PoseArray)
    pc_sub = message_filters.Subscriber('/id_points2', PointCloud2)
    
    # Synchronize images
    rospy.loginfo("time sync start")
    ts = message_filters.ApproximateTimeSynchronizer([pose_sub, pc_sub], queue_size=10, slop=0.5)
    ts.registerCallback(callback)


    sorted_point_pub = rospy.Publisher("/sorted_points2",PointCloud2,queue_size=10)
    
 

    rospy.spin()


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

