#! /usr/bin/env python


import serial
import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32, Float64
from geometry_msgs.msg import TransformStamped

from scipy import signal


port = str(rospy.get_param("~imu_port","/dev/ttyUSB0"))




if __name__ == '__main__':
    rospy.init_node("get_imu")

    port = rospy.get_param("~GPS_PORT",port)
    ser = serial.serial_for_url(port,115200, timeout=0)
    imu_pub = rospy.Publisher("/imu_data", Imu, queue_size=10)
    
    
