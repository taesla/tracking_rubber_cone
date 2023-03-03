#!/usr/bin/env python
# coding: utf-8

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

import pcl
import pcl_helper

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
points = []
def callback(input_ros_msg):

    
    cloud = PointCloud2() 
    cloud.header.frame_id = "/velodyne" 

    cloud.header = input_ros_msg.header

    cloud = pcl_helper.ros_to_pcl(input_ros_msg)

    # 실행 코드 부분 
    filter_axis = 'x'
    axis_min = -0.5
    axis_max = 5
    cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

    filter_axis = 'y'
    axis_min = -2
    axis_max = 2
    cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

    filter_axis = 'z'
    axis_min = -0.4
    axis_max = 0.1
    cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)


    time = rospy.Time.now().to_sec()
    cloud_new = pcl_helper.pcl_to_ros(cloud) #PCL을 ROS 메시지로 변경
    cloud_new.header.stamp = input_ros_msg.header.stamp
    cloud_new.fields = input_ros_msg.fields
    cloud_new.is_dense = True
    pub.publish(cloud_new)

if __name__ == "__main__":
    rospy.init_node('tutorial', anonymous=True)

    rospy.Subscriber('velodyne_points', PointCloud2, callback)
    pub = rospy.Publisher("/velodyne_points_new", PointCloud2, queue_size=100)

    rospy.spin()
