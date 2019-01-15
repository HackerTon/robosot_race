#!/usr/bin/env python
# license removed for brevity
import rospy
import cv2
import numpy as np
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

topic_name = "webcam" # put your topic name here
bridge = CvBridge()
image_size = 300
image = np.zeros((image_size,image_size,3), np.uint8)


def tbcallback(self):
    pass
    
def callback(data):
    global bridge
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")   # capture one frame
    
def range_pub():
    global image
    global image_size
    global topic_name
    image_sub = rospy.Subscriber(topic_name,Image, callback)
    rate = rospy.Rate(100) # 10hz

    # Create the trackbars
    cv2.namedWindow("HSV Range",0)
    for j in 'HSV':
        for i in ["MIN", "MAX"]:
            vmax = 255
            if i == "MIN":
                v = 0 
            elif j == "H": 
                v = 179
                vmax = 179
            else: 
                v = 255 
            cv2.createTrackbar("%s_%s" % (j, i), "HSV Range", v, vmax, tbcallback)
    
    # Process the image based on trackbars
    while not rospy.is_shutdown():
        # get the trackbar values
        values = []
        for i in ["MIN", "MAX"]:
            for j in 'HSV':
                v = cv2.getTrackbarPos("%s_%s" % (j, i), "HSV Range")
                values.append(v)
        v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = values
        
        # first resize the image to make it smaller
        r = image_size*1.0 / image.shape[1]
        dim = (image_size, int(image.shape[0] * r))
        image = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
        # then convert to HSV colour space
        frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # finally filter the colour out
        try:
            thresh = cv2.inRange(frame_to_thresh, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))
            # Bitwise-AND mask and original image
            res = cv2.bitwise_and(image,image, mask= thresh)
            #cv2.imshow("Original", image)
            #cv2.imshow("Thresh", thresh)
            #cv2.imshow('Result',res)
            cv2.imshow("HSV Range", np.hstack([image, res]))
        except:
            pass
            
        rate.sleep()
        cv2.waitKey(1)
        

if __name__ == '__main__':
    try:
        rospy.init_node('colour_range', anonymous=True)
        range_pub()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        pass
        
        
