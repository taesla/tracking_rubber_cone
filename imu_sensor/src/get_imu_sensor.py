#! /usr/bin/env python

import serial
import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32, Float64
from geometry_msgs.msg import TransformStamped

from scipy import signal

old_time=0
port = str(rospy.get_param("~imu_port","/dev/ttyUSB0"))
rpy=[0,0,0]
w_speed=[0,0,0]
accel=[0,0,0]
error_yaw=0 #non magnetic -201.5
#error_yaw=-92.6
def euler_to_quaternion(roll,pitch,yaw):

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

def imu_error_callback(data):
    global error_yaw, rpy
    error_yaw +=-rpy[2] + data.data
    print(error_yaw)


def pub_tf_transform(roll,pitch,yaw,w_speed,accel):


    t = TransformStamped()  # pose of turntable_frame w.r.t. turntable_base
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'base_link'
    t.child_frame_id  = 'imu_link'
    t.transform.translation.x = roll
    t.transform.translation.y = pitch
    t.transform.translation.z = yaw
    q = euler_to_quaternion(roll, pitch, yaw)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node("get_imu")

    port = rospy.get_param("~GPS_PORT",port)
    ser = serial.serial_for_url(port,115200, timeout=0)

    #rospy.Subscriber("yaw_error",Float32,imu_error_callback)
    imu_pub = rospy.Publisher("/imu_data", Imu, queue_size=10) # /imu_data
    yaw_pub = rospy.Publisher("/imu_yaw",Float64, queue_size=10)
    #yaw_z_pub = rospy.Publisher("/imu_yaw_z",Float64, queue_size=10)
    br = tf2_ros.TransformBroadcaster()
    mag_pub = rospy.Publisher("/imu_mag", MagneticField, queue_size=10)
    mag_yaw_pub = rospy.Publisher("/imu_mag_yaw",Float64, queue_size=10)
    vel_pub = rospy.Publisher("imu_vel",Float64, queue_size=1)
    #r=rospy.Rate(20)
    mag=MagneticField()
    imu=Imu()
    global yaw_z
    yaw_z=0

    while not rospy.is_shutdown():
        IMU_message=ser.readline()

        if (len(IMU_message)>55):
            imu.header.stamp = rospy.Time.now()
            imu.header.frame_id = "map"
            data=IMU_message.split(",")
            #print(data)
            rpy[0]=round(float(data[1]),3)
            rpy[1]=round(float(data[2]),3)
            rpy[2]=round(float(data[3]),3)+error_yaw
            #print(rpy[2])
            if (rpy[2] >= 180):
                rpy[2] = rpy[2] - 2*180
            elif (rpy[2] <= -180):
                rpy[2] = rpy[2] + 2*180

            roll=rpy[0]*np.pi/180
            pitch=-rpy[1]*np.pi/180
            yaw=-rpy[2]*np.pi/180
            qx,qy,qz,qw = euler_to_quaternion(roll, pitch, yaw)

            imu.orientation.x = qx
            imu.orientation.y = qy
            imu.orientation.z = qz
            imu.orientation.w = qw

            w_speed[0]=round(float(data[4]),3)
            w_speed[1]=round(float(data[5]),3)
            w_speed[2]=round(float(data[6]),3)
	    
	    #high pass filter
	    
	    
            imu.angular_velocity.x = w_speed[0]*np.pi/180
            imu.angular_velocity.y = w_speed[1]*np.pi/180
            imu.angular_velocity.z = w_speed[2]*np.pi/180

            accel[0]=round(float(data[7]),3)
            accel[1]=round(float(data[8]),3)
            accel[2]=round(float(data[9]),3)

            imu.linear_acceleration.x = accel[0]
            imu.linear_acceleration.y = accel[1]
            imu.linear_acceleration.z = accel[2]
	    
            velocity = accel[0]
        
            t=rospy.Time.now()
            time=t.to_sec()
            delta_time=time-old_time
            old_time=time
            ang_z_rad=w_speed[2]*np.pi/180
            yaw_z+=ang_z_rad*delta_time
            if (yaw_z >= np.pi):
                yaw_z=yaw_z-2*np.pi
            elif (yaw_z <= -np.pi):
                yaw_z=yaw_z+2*np.pi

            velocity = accel[0] * delta_time

            mag.header=imu.header
            mag.magnetic_field.x=float(data[10])
            mag.magnetic_field.y=float(data[11])
            mag.magnetic_field.z=float(data[12])
        
            mag_x=-mag.magnetic_field.y
            mag_y=-mag.magnetic_field.x

            if mag_x == 0 :
                if mag_y > 0 :
                    mag_yaw=90
                else :
                    mag_yaw=-90
            else :
                mag_yaw=np.arctan( mag_y / mag_x ) *180/np.pi
                if mag_x < 0 :
                    mag_yaw=mag_yaw+180
                    mag_yaw=mag_yaw*np.pi/180
                    battery=round(float(data[13]),3)
                    imu_pub.publish(imu)
                    vel_pub.publish(velocity)
                    pub_tf_transform(roll,pitch,yaw,w_speed,accel)
                    print("roll",roll, "pitch",pitch, "yaw",yaw ,"w_speed",w_speed, "accel",accel)
                    #print("yaw:",rpy[2], "battery:",battery, "yaw_radian:",yaw , "yaw_from_ang.z:",yaw_z, "delta_time: ",delta_time)
                    #r.sleep()
                    #yaw_pub.publish(yaw) #yaw
                    #mag_pub.publish(mag)
                    #mag_yaw_pub.publish(mag_yaw)
                    #yaw_z_pub.publish(yaw_z)
                else:
                    pass
        else:
            pass
        #mainloop()
