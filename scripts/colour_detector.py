#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from dynamic_reconfigure.server import Server
from robosot_race.cfg import ColourRangeConfig

H_min,S_min,V_min = 0,0,0
H_max,S_max,V_max = 180,255,255
    
def colour_callback(config, level):
    global H_min,S_min,V_min,H_max,S_max,V_max
    H_min = config.H_min
    S_min = config.S_min
    V_min = config.V_min
    H_max = config.H_max
    S_max = config.S_max
    V_max = config.V_max
    return config
       
def callback(data):
    global colour_xy, result
    global H_min,S_min,V_min,H_max,S_max,V_max
    
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data, "bgr8")   # capture one frame
    # do your OpenCV processing here
    # Convert BGR to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    #lower_blue = np.array([105,0,0])
    #upper_blue = np.array([179,255,255])
    lower_blue = np.array([H_min,S_min,V_min])
    upper_blue = np.array([H_max,S_max,V_max])
    #print lower_blue
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(img,img, mask= mask)

    # Calculate center of gravity 
    M = cv2.moments(mask)
    area = M['m00']
    if area > 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        #print cx,",",cy
        rospy.loginfo(rospy.get_caller_id() + " Colour detected at ("+str(cx)+","+str(cy)+")")
        cv2.circle(res,(cx,cy), 10, (0,0,255), -1)
        colour_xy.publish(str(cx)+","+str(cy))
        
    else:
        #print "No object found."
        rospy.loginfo(rospy.get_caller_id() + " Colour not found.")
        
    #cv2.imshow('ori',img)
    #cv2.imshow('mask',mask)
    #cv2.imshow('res',res)
    res_message = bridge.cv2_to_imgmsg(res, "bgr8")# encoding="passthrough")
    result.publish(res_message)
    
    cv2.waitKey(1) # must include this line

def listener():
    global colour_xy, result
    
    rospy.init_node('colour_detector', anonymous=True)

    rospy.Subscriber("/camera/image",Image, callback)
    colour_xy = rospy.Publisher('colour_xy', String, queue_size=1)
    result = rospy.Publisher('colour_result', Image, queue_size=1)
    srv1 = Server(ColourRangeConfig, colour_callback)
        
    rospy.spin()

if __name__ == '__main__':
    listener()
