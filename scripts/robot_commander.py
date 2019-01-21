#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

red_xyr = [-1,-1,-1]
blue_xyr = [-1,-1,-1]
yellow_xyr = [-1,-1,-1]
black_xyr = [-1,-1,-1]
white_xyr = [-1,-1,-1]
xyr = [red_xyr, blue_xyr, yellow_xyr, black_xyr, white_xyr]
red_timer = 0

# Max speed and PID constants
# angular rotation:
max_a = 0.5 # rad/s
kp_a = 1.2
max_l = 0.15 # m/s
kp_l = 0.6
lost_ball_cnt = 0
slow_approach_cnt = 0
offset_y_gnd =  -0.45 #0.12 # offset of y-axis 

def red_xyr_callback(data):
    global red_xyr
    red_xyr_str = data.data.split(',')
    # print red_xyr[0], red_xyr[1], red_xyr[2]
    red_xyr[0] = float(red_xyr_str[0])
    red_xyr[1] = float(red_xyr_str[1])
    red_xyr[2] = float(red_xyr_str[2])
    red_timer = 0
    
def blue_xyr_callback(data):
    global blue_xyr
    blue_xyr_str = data.data.split(',')
    # print blue_xyr[0], blue_xyr[1], blue_xyr[2]
    blue_xyr[0] = float(blue_xyr_str[0])
    blue_xyr[1] = float(blue_xyr_str[1])
    blue_xyr[2] = float(blue_xyr_str[2])

def yellow_xyr_callback(data):
    global yellow_xyr
    yellow_xyr_str = data.data.split(',')
    # print yellow_xyr[0], yellow_xyr[1], yellow_xyr[2]
    yellow_xyr[0] = float(yellow_xyr_str[0])
    yellow_xyr[1] = float(yellow_xyr_str[1])
    yellow_xyr[2] = float(yellow_xyr_str[2])

def black_xyr_callback(data):
    global black_xyr
    black_xyr_str = data.data.split(',')
    # print black_xyr[0], black_xyr[1], black_xyr[2]
    black_xyr[0] = float(black_xyr_str[0])
    black_xyr[1] = float(black_xyr_str[1])
    black_xyr[2] = float(black_xyr_str[2])

def white_xyr_callback(data):
    global white_xyr
    white_xyr_str = data.data.split(',')
    # print white_xyr[0], white_xyr[1], white_xyr[2]
    white_xyr[0] = float(white_xyr_str[0])
    white_xyr[1] = float(white_xyr_str[1])
    white_xyr[2] = float(white_xyr_str[2])


def robot_commander():
    global kp_a, kp_l, max_a, max_l
    global red_xyr, blue_xyr, yellow_xyr, black_xyr, white_xyr, xyr 
    global lost_ball_cnt, slow_approach_cnt

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.Subscriber("red/ball_xyr", String, red_xyr_callback)
    rospy.Subscriber("blue/ball_xyr", String, blue_xyr_callback)
    rospy.Subscriber("yellow/ball_xyr", String, yellow_xyr_callback)
    rospy.Subscriber("black/ball_xyr", String, black_xyr_callback)
    rospy.Subscriber("white/ball_xyr", String, white_xyr_callback)
    rospy.init_node('commander', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    command = Twist()
    # state 0 - turn around
    # state 1 - get nearest ball
    # state 2 - go to ball
    # state 3 - slow approach
    # state 4 - go to goal
    # state 5 - reverse from goal and turn
    STATE = 1
    # BALL_COLOUR
    # 0 - red 
    # 1 - blue 
    # 2 - yellow 
    # 3 - black 
    # 4 - white 
    BALL_COLOUR = 0
    while not rospy.is_shutdown():
        # # 1. if radius > 0
        # # 2. set angular to kpa*ball_x 
        # # 3. set linear to kpl*ball_y
        # # 4. move
        # if red_xyr[2] > 0:
        #     command.angular.z = kp_a * red_xyr[0]
        #     command.linear.x = kp_l * (red_xyr[1] - offset_y_gnd)
        # else:
        #     command.angular.z = 0.2
        #     command.linear.x = 0.0
        # rospy.loginfo(command)
        # pub.publish(command)
        if STATE == 0: # turn around
            command.angular.z = 0.2
            command.linear.x = 0.05
            STATE = 1

        elif STATE == 1: # get nearest ball
            BALL_COLOUR = 0
            rospy.sleep(0.5) # wait a bit
            # Loop over all the colours; 
            # if ball is found (r > 0), then get its y. 
            # Store the smallest y i.e. nearest ball 
            min_y = 500
            min_y_i = -1
            for i, c_xyr in enumerate(xyr):
                if c_xyr[2] > 0 and c_xyr[1] < min_y:
                    min_y = c_xyr[1]
                    min_y_i = i 
            # print "---------"
            # print xyr
            # print min_y_i, min_y
            if min_y_i > -1: 
                BALL_COLOUR = min_y_i 
                lost_ball_cnt = 0
                STATE = 2
            else:
                STATE = 0

        elif STATE == 2: # move towards ball
            command.angular.z = kp_a * xyr[BALL_COLOUR][0]
            command.linear.x = kp_l * (xyr[BALL_COLOUR][1] - offset_y_gnd)
            # limit the speeds
            if command.linear.x > max_l:
                command.linear.x = max_l
            if command.angular.z > max_a:
                command.angular.z = max_a
            if command.angular.z < -max_a:
                command.angular.z = -max_a
            
            # check if ball is near enough
            if xyr[BALL_COLOUR][1] < offset_y_gnd and -0.05<xyr[BALL_COLOUR][0]<0.05:
                STATE = 3

            # check if ball is lost
            # if yes, start looking again
            if xyr[BALL_COLOUR][2] == 0:
                lost_ball_cnt+=1
            if lost_ball_cnt > 20:
                STATE = 1

        elif STATE == 3: # Slow approach ball
            command.angular.z = 0.0
            command.linear.x = 0.1
            slow_approach_cnt+=1
            if slow_approach_cnt > 30:
                slow_approach_cnt = 0
                STATE = 6


        elif STATE == 6: # Stop
            command.angular.z = 0.0
            command.linear.x = 0.0
            # STATE = 1

        print("--------------------------")
        print("STATE: " + str(STATE))
        print("BALL_COLOUR: " + str(BALL_COLOUR))
        print("COMMAND: x:" + str(command.linear.x) + "   z: " + str(command.angular.z))
        pub.publish(command) 
        rate.sleep()

if __name__ == '__main__':
    try:
        robot_commander()
    except rospy.ROSInterruptException:
        pass
