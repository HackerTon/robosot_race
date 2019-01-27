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
hough_accum_resolution= 1.2
min_circle_dist = 5
canny_edge_th = 100
hough_accum_th = 28
min_radius = 0
max_radius = 100

def colour_callback(config, level):
    global H_min,S_min,V_min,H_max,S_max,V_max
    global hough_accum_resolution, min_circle_dist, canny_edge_th, hough_accum_th, min_radius, max_radius
    H_min = config.H_min
    S_min = config.S_min
    V_min = config.V_min
    H_max = config.H_max
    S_max = config.S_max
    V_max = config.V_max
    hough_accum_resolution  = config.hough_accum_resolution
    min_circle_dist         = config.min_circle_dist
    canny_edge_th           = config.canny_edge_th
    hough_accum_th          = config.hough_accum_th 
    min_radius              = config.min_radius 
    max_radius              = config.max_radius
    return config
       
def callback(data):
    global ball_xyr, output_colour, output_circle
    global H_min,S_min,V_min,H_max,S_max,V_max
    global hough_accum_resolution, min_circle_dist, canny_edge_th, hough_accum_th, min_radius, max_radius
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data, "bgr8")   # capture one frame
    
    # do your OpenCV processing here
    # Convert BGR to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # define range of blue color in HSV
    lower_hsv = np.array([H_min,S_min,V_min])
    upper_hsv = np.array([H_max,S_max,V_max])
    # Threshold the HSV image to get only wanted colors
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    # cv2.imshow("mask", mask)
    # Bitwise-AND mask and original image
    res_colour = cv2.bitwise_and(img,img, mask= mask)
    res_circle = res_colour.copy()
    # cv2.imshow("res_colour", res_colour)
    # To do circle detection, first gray and find the edges
    gray = cv2.cvtColor(res_colour, cv2.COLOR_BGR2GRAY)
    canny_edge = cv2.Canny(gray, 50, 240)
    kernel = np.ones((3,3),np.uint8)
    dilation = cv2.dilate(canny_edge,kernel,iterations = 1)
    # cv2.imshow("dilation", dilation)
    # cv2.imshow("canny_edge", canny_edge)
    # Then detect circles in the image
    circles = cv2.HoughCircles(dilation,cv2.HOUGH_GRADIENT,hough_accum_resolution,min_circle_dist,param1=canny_edge_th,param2=hough_accum_th,minRadius=min_radius,maxRadius=max_radius)
    # print circles
    # ensure at least 1 circle was found
    if circles is not None:
        rospy.loginfo(rospy.get_caller_id() + " Ball(s) found.")
        # convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")
        circles_sorted = sorted(circles, key=lambda x: x[2])
        # print circles_sort
        # loop over the (x, y) coordinates and radius of the circles
        for (x, y, r) in circles:
            # draw the circle in the output image, then draw a rectangle
            # corresponding to the center of the circle
            cv2.circle(res_circle, (x, y), r, (128, 255, 0), 2)
            cv2.rectangle(res_circle, (x - 2, y - 2), (x + 2, y + 2), (0, 128, 255), -1)
    
    else:
        #print "No object found."
        rospy.loginfo(rospy.get_caller_id() + " Ball not found.")
        circles_sorted = [[0,0,0]] # dummy numbers

    # Convert the resulting images to ROS Images
    res_colour_message = bridge.cv2_to_imgmsg(res_colour, "bgr8")
    res_circle_message = bridge.cv2_to_imgmsg(res_circle, "bgr8")
    # And then publish them
    output_circle.publish(res_circle_message)
    output_colour.publish(res_colour_message)
    # Get the largest ball's coordinate,
    # measured from the center of the image
    _cx = 1.0*circles_sorted[-1][0]/img.shape[1]
    _cy = 1.0*circles_sorted[-1][1]/img.shape[0]
    cx = 0.5 - _cx
    cy = 0.5 - _cy 
    cr = 1.0*circles_sorted[-1][2]/img.shape[1]

    # and publish its coordinate and radius size
    ball_xyr.publish(str(cx)+","+str(cy)+","+str(cr))
        
    cv2.waitKey(1) # must include this line

def listener():
    global ball_xyr, output_circle, output_colour
    # Init the node
    rospy.init_node('ball_detector', anonymous=True)
    # Input image topic from camera
    rospy.Subscriber("/camera/image",Image, callback)
    # Output: coordinate of the largest ball found
    ball_xyr = rospy.Publisher('ball_xyr', String, queue_size=1)
    # Output: processed images showing the filtered coulour and detected ball
    output_colour = rospy.Publisher('output_colour', Image, queue_size=1)
    output_circle = rospy.Publisher('output_circle', Image, queue_size=1)
    # Dynamic reconfigure server
    srv1 = Server(ColourRangeConfig, colour_callback)
    # Just spin and wait
    rospy.spin()

if __name__ == '__main__':
    listener()
