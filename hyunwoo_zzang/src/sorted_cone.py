#!/usr/bin/env python

import rospy
import struct
import math
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

import numpy as np

minimum_points =[]

def callback(msg):
    
    cone_number = msg.header.frame_id[-1]
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1),
            PointField('intensity', 16, PointField.FLOAT32, 1)]
    if cone_number == '0':
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
        cone_dist.append(math.sqrt( p[0] ** 2 + p[1] ** 2 ))

    min_x=(cone_x[cone_dist.index(min(cone_dist))])
    min_y=(cone_y[cone_dist.index(min(cone_dist))])
    min_z=(cone_z[cone_dist.index(min(cone_dist))])

    print(min_x,min_y,min_z)
    pt = [min_x,min_y,min_z,0,0]
    if len(msg.header.frame_id)==5:
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

    sorted_points.header = msg.header   
    sorted_points.header.stamp = rospy.Time.now() 
    sorted_points.header.frame_id = 'velodyne'


    



if __name__=='__main__':

    rospy.init_node('cone_distance')
    rospy.Subscriber("/id_points2",PointCloud2,callback)
    sorted_point_pub = rospy.Publisher("/sorted_points2",PointCloud2,queue_size=10)
    
    sorted_points= PointCloud2()

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

