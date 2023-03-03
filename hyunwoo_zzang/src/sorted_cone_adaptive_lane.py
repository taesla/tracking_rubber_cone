#!/usr/bin/env python

import rospy
import struct
import math
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseArray , Point
import sensor_msgs.point_cloud2 as pc2

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import numpy as np

minimum_points =[]

def callback2(msg):

    global cone_centers
    cone_centers = PoseArray()
    cone_centers = msg

def callback(msg):
    cone_number = msg.header.frame_id[-1]
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1),
            PointField('intensity', 16, PointField.FLOAT32, 1)]
    if cone_number == '0':
        print('publish')
        sorted_point_pub.publish(pc2.create_cloud(sorted_points.header,fields,minimum_points))
        del minimum_points[:]


    cone_x = []
    cone_y = []
    cone_z = []

    cone_dist = []

    collsion = MarkerArray()
    collsion.markers = []
    collision_distance = Marker()
    collision_distance.header.frame_id = "/velodyne"
    collision_distance.ns = "collision_distance"
    #marker_est.id = 42+i
    collision_distance.type = Marker.TEXT_VIEW_FACING
    collision_distance.action = Marker.ADD


    yellow_cone_lines = MarkerArray()
    yellow_cone_lines.markers = []
    yellow_cone_line = Marker()
    yellow_cone_line.header.frame_id = "/velodyne"
    yellow_cone_line.ns = "yellow_cone_line"
    #marker_est.id = 42+i
    yellow_cone_line.type = Marker.LINE_STRIP
    yellow_cone_line.action = Marker.ADD

    blue_cone_lines = MarkerArray()
    blue_cone_lines.markers = []
    blue_cone_line = Marker()
    blue_cone_line.header.frame_id = "/velodyne"
    blue_cone_line.ns = "blue_cone_line"
    #marker_est.id = 42+i
    blue_cone_line.type = Marker.LINE_STRIP
    blue_cone_line.action = Marker.ADD
    for p in pc2.read_points(msg , field_names = ("x","y","z"), skip_nans = True):
        cone_x.append(p[0])
        cone_y.append(p[1])
        cone_z.append(p[2])

    if (len(cone_centers.poses)) > 0 :
        for i in range(len(cone_centers.poses)):
            for j in range(len(cone_x)):
                
                if math.sqrt((cone_x[j]-cone_centers.poses[i].position.x)**2 + (cone_y[j]-cone_centers.poses[i].position.y)**2) < 0.5 :
                    pt = [cone_centers.poses[i].position.x,cone_centers.poses[i].position.y,0,0,0]
                    if len(msg.header.frame_id)==5:
                        r = int(0 * 255.0)
                        g = int(0 * 255.0)
                        b = int(1 * 255.0)
                        a = 255

                        blue_cone_line.points.append(Point(cone_centers.poses[0].position.x,cone_centers.poses[0].position.y,0.0))
                        blue_cone_line.points.append(Point(cone_centers.poses[1].position.x,cone_centers.poses[1].position.y,0.0))
                        blue_cone_line.pose.position.z = 0
                        blue_cone_line.pose.orientation.w = 1
                        blue_cone_line.color.r, blue_cone_line.color.g, blue_cone_line.color.b = (0.0, 0.0, 1.0)
                        blue_cone_line.color.a = 1.0
                        blue_cone_line.scale.x, blue_cone_line.scale.y, blue_cone_line.scale.z = (0.07, 0.07, 0.07)
                        blue_cone_line.lifetime = rospy.Time.now()
                        blue_cone_lines.markers.append(blue_cone_line)
                        blue_cone_lines_pub.publish(blue_cone_lines)
                    else:
                        r = int(1 * 255.0)
                        g = int(1 * 255.0)
                        b = int(0 * 255.0)
                        a = 255

                        yellow_cone_line.points.append(Point(cone_centers.poses[0].position.x,cone_centers.poses[0].position.y,0.0))
                        yellow_cone_line.points.append(Point(cone_centers.poses[1].position.x,cone_centers.poses[1].position.y,0.0))
                        yellow_cone_line.pose.position.z = 0
                        yellow_cone_line.pose.orientation.w = 1
                        yellow_cone_line.color.r, yellow_cone_line.color.g, yellow_cone_line.color.b = (1.0, 1.0, 0.0)
                        yellow_cone_line.color.a = 1.0
                        yellow_cone_line.scale.x, yellow_cone_line.scale.y, yellow_cone_line.scale.z = (0.07, 0.07, 0.07)
                        yellow_cone_line.lifetime = rospy.Time.now()
                        yellow_cone_lines.markers.append(yellow_cone_line)
                        yellow_cone_lines_pub.publish(yellow_cone_lines)
                    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                    pt[3] = rgb
                    pt[4] = 1

                    minimum_points.append(pt)
                    
                    break

    #print(msg.header.frame_id)
    sorted_points.header = msg.header   
    sorted_points.header.stamp = rospy.Time.now() 
    sorted_points.header.frame_id = 'velodyne'

    



if __name__=='__main__':

    rospy.init_node('cone_distance_lane')
    rospy.Subscriber("/id_points2",PointCloud2,callback)
    rospy.Subscriber("/adaptive_clustering/poses",PoseArray,callback2)
    sorted_point_pub = rospy.Publisher("/sorted_points2",PointCloud2,queue_size=1)
    
    blue_cone_lines_pub = rospy.Publisher('/blue_cone_lines_pub', MarkerArray, queue_size=1)
    yellow_cone_lines_pub = rospy.Publisher('/yellow_cone_lines_pub', MarkerArray, queue_size=1)

    cone_centers = PoseArray()
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

