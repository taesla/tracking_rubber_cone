#!/usr/bin/env python
import numpy
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import math


import struct
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

import pcl
import pcl_helper


import message_filters
from std_msgs.msg import Int32, Float32

def do_passthrough(pcl_data,filter_axis,axis_min,axis_max):
    '''
    Create a PassThrough  object and assigns a filter axis and range.
    :param pcl_data: point could data subscriber
    :param filter_axis: filter axis
    :param axis_min: Minimum  axis to the passthrough filter object
    :param axis_max: Maximum axis to the passthrough filter object
    :return: passthrough on point cloud
    '''
    passthrough = pcl_data.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    return passthrough.filter()

# Use RANSAC planse segmentation to separate plane and not plane points
# Returns inliers (plane) and outliers (not plane)
def do_ransac_plane_normal_segmentation(point_cloud, input_max_distance):
    segmenter = point_cloud.make_segmenter_normals(ksearch=50)
    segmenter.set_optimize_coefficients(True)
    segmenter.set_model_type(pcl.SACMODEL_NORMAL_PLANE)  #pcl_sac_model_plane
    segmenter.set_normal_distance_weight(0.1)
    segmenter.set_method_type(pcl.SAC_RANSAC) #pcl_sac_ransac
    segmenter.set_max_iterations(1000)
    segmenter.set_distance_threshold(input_max_distance) #0.03)  #max_distance
    indices, coefficients = segmenter.segment()

    inliers = point_cloud.extract(indices, negative=False)
    outliers = point_cloud.extract(indices, negative=True)

    return indices, inliers, outliers


    

def callback(pose_sub, pc_sub):
    # The callback processing the pairs of numbers that arrived at approximately the same time
    #rospy.loginfo("receiving frame")

    pub_pose = Pose()
    pub_pose_array = PoseArray()

    cloud = PointCloud2
    tmp_cloud = PointCloud2

    #cloud.header.frame_id = "/velodyne" 

    pose_array = pose_sub
    cloud = pcl_helper.ros_to_pcl(pc_sub)

    pc_x=[]
    pc_y=[]
    pc_z=[]
    
    pc_size = int(str(cloud)[15:17])

    for idx in range(pc_size):
        pc_x.append(cloud[idx][0])
        pc_y.append(cloud[idx][1])
        pc_z.append(cloud[idx][2])
        for i in range(len(pose_array.poses)):
            x = (pc_x[idx]-pose_array.poses[i].position.x)**2
            y = (pc_y[idx]-pose_array.poses[i].position.y)**2
            z = (pc_z[idx]-pose_array.poses[i].position.z)**2
            #print("final")
            # radius settings (choose range you want around each pose)
            if(x+y+z<=10):
                
                print("final_pose.x:",pose_array.poses[i].position.x)
                print("final_pose.y:",pose_array.poses[i].position.y)
                print("final_pose.z:",pose_array.poses[i].position.z)
                #pose_array.header.frame_id = pc_sub.header.frame_id
                pose_array.header.frame_id = "/velodyne"
                #tmp_cloud.header.frame_id = pc_sub.header.frame_id
                print(pc_sub.header.frame_id[0])
                if pc_sub.header.frame_id[0] == 'b':
                    print('b')
                    pose_array.poses[i].position.z = 0

                if pc_sub.header.frame_id[0] == 'y' :
                    print('y')
                    pose_array.poses[i].position.z = 1
                pub.publish(pose_array)
                # pub2.publish(pc_sub) 
        

    
        
if __name__=='__main__':
    # Init ROS
    rospy.init_node('final_clustering')
    # Subscribers
    
    pose_sub = message_filters.Subscriber('/adaptive_clustering/poses', PoseArray)
    pc_sub = message_filters.Subscriber('/id_points2', PointCloud2)
    
    # Synchronize images
    rospy.loginfo("time sync start")
    ts = message_filters.ApproximateTimeSynchronizer([pose_sub, pc_sub], queue_size=10, slop=0.5)
    ts.registerCallback(callback)
    
    
    # Publishers
    pub = rospy.Publisher('/final_clustering_pose', PoseArray, queue_size=1)
    # pub2 = rospy.Publisher('/final_clustering_id_points2', PointCloud2, queue_size=1)

    # Spin
    rospy.spin()
   
