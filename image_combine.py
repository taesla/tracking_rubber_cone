import numpy as np
import cv2
import glob
import rospy
import message_filters

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def callback_right(right):
    global cv_image_right
    bridge = CvBridge()
    cv_image_right = bridge.imgmsg_to_cv2(right, desired_encoding='rgb8')


def callback_left(left):
    global cv_image_right
    bridge = CvBridge()
    cv_image_left = bridge.imgmsg_to_cv2(left, desired_encoding='rgb8')
    cv_image_right = cv_image_right[0:360,0:640]
    cv_image_left = cv_image_left[0:360,0:640]
    cv_image_combined = cv2.hconcat([cv_image_left , cv_image_right])

    image_message = bridge.cv2_to_imgmsg(cv_image_combined, encoding="rgb8")
    image_message.header.stamp = rospy.Time.now()

    cam_pub.publish(image_message)


if __name__ == '__main__':

    rospy.init_node('image_combiner')
    cam_pub = rospy.Publisher("/combined_image",Image,queue_size =1)

    rospy.Subscriber("/camera_right/usb_cam/image_raw",Image,callback_right)
    rospy.Subscriber("/camera_left/usb_cam/image_raw",Image,callback_left)

    #right_sub = message_filters.Subscriber("/camera_right/usb_cam/image_raw",Image)
    #left_sub = message_filters.Subscriber("/usb_cam/image_raw",Image)

    #ts = message_filters.ApproximateTimeSynchronizer([right_sub,left_sub], 10,10)
    #ts.registerCallback(callback)

    rospy.spin()
