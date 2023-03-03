#!/usr/bin/env python

import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Point,PoseArray
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

def bounding_callback(msg):
    global color_area 
    color_area = BoundingBoxes()
    color_area = msg
    # global blue_color_area, yellow_color_area
    # blue_color_area = []
    # yellow_color_area = []

    # for i in range(len(msg.bounding_boxes)) :
        
    #     if msg.bounding_boxes[i].Class == 'blue' :
            
    #         blue_color_area[i][0] = msg.bounding_boxes[i].xmax * -0.00232559 + 1.22752
    #         blue_color_area[i][1] = msg.bounding_boxes[i].xmin * -0.00232559 + 1.22752
    #         blue_color_area[i][2] = msg.bounding_boxes[i].ymax * 0.015138 - 3.76716
    #         blue_color_area[i][3] = msg.bounding_boxes[i].ymin * 0.015138 - 3.76716

    #     else :
    #         print(i)
    #         yellow_color_area[i][0] = msg.bounding_boxes[i].xmax * -0.00232559 + 1.22752
    #         yellow_color_area[i][1] = msg.bounding_boxes[i].xmin * -0.00232559 + 1.22752
    #         yellow_color_area[i][2] = msg.bounding_boxes[i].ymax * 0.015138 - 3.76716
    #         yellow_color_area[i][3] = msg.bounding_boxes[i].ymin * 0.015138 - 3.76716




def cone_callback(msg):

    detected_cone = Marker()
    detected_cone.header.frame_id = 'velodyne'
    detected_cone.type = Marker.POINTS
    detected_cone.action = Marker.ADD

    for i in range(len(color_area.bounding_boxes)) :
        for j in range(len(msg.poses)) :
            print(msg.poses[j].position.y,color_area.bounding_boxes[i].Class,color_area.bounding_boxes[i].xmax * -0.00232559 + 1.2752,color_area.bounding_boxes[i].xmin * -0.00232559+1.2752)
            print(len(msg.poses))
            print(1)
            if color_area.bounding_boxes[i].Class == 'blue' :
                if msg.poses[j].position.y < color_area.bounding_boxes[i].xmax * -0.00232559 + 1.8 and msg.poses[j].position.y > color_area.bounding_boxes[i].xmin * -0.00232559 + 0.8 and msg.poses[j].position.z < color_area.bounding_boxes[i].ymax * 0.015138 - 3.8 and msg.poses[j].position.z > color_area.bounding_boxes[i].ymin * 0.015138 - 3.2 :
                    detected_cone.points.append(Point(msg.poses[j].position.x,msg.poses[j].position.y,0))
                    detected_cone.color = ColorRGBA(0,0,1,1)
                    detected_cone.scale.x = 0.1
                    detected_cone.scale.y = 0.1
                    detected_cone.scale.z = 0.1
                
                else :
                # if msg.poses[j].position.y < color_area.bounding_boxes[i].xmax * -0.00232559 + 1.55 and msg.poses[j].position.y > color_area.bounding_boxes[i].xmin * -0.00232559 + 0.4 and msg.poses[j].position.z < color_area.bounding_boxes[i].ymax * 0.015138 - 3.8 and msg.poses[j].position.z > color_area.bounding_boxes[i].ymin * 0.015138 - 3.2  :
                    detected_cone.points.append(Point(msg.poses[j].position.x,msg.poses[j].position.y,0))
                    detected_cone.color = ColorRGBA(1,1,0,1)
                    detected_cone.scale.x = 0.1
                    detected_cone.scale.y = 0.1
                    detected_cone.scale.z = 0.1
    detect_pub.publish(detected_cone)                

    
                
if __name__=='__main__':

    rospy.init_node('cone_fusion')
    rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,bounding_callback)
    rospy.Subscriber("/adaptive_clustering/poses",PoseArray,cone_callback)
    
    detect_pub = rospy.Publisher("/detected_cone",Marker,queue_size =1)

    rospy.spin()
