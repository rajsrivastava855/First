#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3 
from nav_msgs.msg import Odometry
bridge=CvBridge()

global flag
KP = 0.0025
KI= 0.000001
KD= 0.0003
differror =0
intgerror =0
cmd_msg = Twist()
odom = Odometry()

def image_callback(data):
    print("Received an image!")
    
    # Convert your ROS Image message to OpenCV2
    cv2_img = bridge.imgmsg_to_cv2(data, "rgb8")
    cv2.imshow("frame",cv2_img)
    img_grey = cv2.cvtColor(cv2_img,cv2.COLOR_BGR2HSV)
    frame_threshold = cv2.inRange(img_grey, (0,0,0), (360,250,50))
    cv2.imshow('frame_thresold',frame_threshold)
    contours, hierarchy = cv2.findContours(frame_threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # draw all contours
    image = cv2.drawContours(cv2_img, contours, -1, (0, 255, 0), 2)
    rgb = cv2.cvtColor(image, cv2.COLOR_HSV2RGB)	
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)	
    # create a binary thresholded image
    _,thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)
   
    # cv2.putText(image ,'o', (240, 330),cv2.FONT_HERSHEY_SIMPLEX,5,(255,0,0) , 10 ,cv2.LINE_AA , True)
    # cv2.imshow('Contour image', image)
    # cv2.waitKey(5)
    M = cv2.moments(frame_threshold)
    if M["m00"] != 0:
        centroid_x = int(M["m10"] / M["m00"])
        centroid_y = int(M["m01"] / M["m00"])
    else:
        centroid_x = 0
        centroid_y = 0
    cv2.putText(img=image, text='o', org=(centroid_x,centroid_y), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=3, color=(255, 0, 0),thickness=1)
    cv2.imshow('Contour', image)
    cv2.waitKey(5)
    # extract the centroid x and y coordinates
    
    # display the centroid coordinates
    print("Centroid X: ", centroid_x)
    print("Centroid Y: ", centroid_y)
    rospy.Rate(1)
      
    # assume that 'frame' is the variable containing the camera frame
    # get dimensions of image
    dimensions = rgb.shape
    # height, width, number of channels in image
    frame_height = rgb.shape[0]
    frame_width = rgb.shape[1]
    _ = rgb.shape[2]
    # calculate the center of the camera frame
    center_x = int(frame_width / 2)
    center_y = int(frame_height / 2)
    # calculate the error
    error_x = centroid_x - center_x
    error_y = centroid_y - center_y
    #  display the error coordinates
    print("Error X: ", error_x)
    print("Error Y: ", error_y)
    
    # assume that 'threshold' is the maximum allowable error
    #threshold = 20
    # determine the direction based on the error
    if error_x > 20:
        print("Turn right")
        
    elif error_x < -20:
        print("Turn left")
      
    else:
        print("Continue straight")

    if round(odom.pose.pose.position.x,2)==2.21:
        cmd_msg.angular.z=0
        cmd_msg.linear.x=0
    else:
        if error_x>1:
            cmd_msg.angular.z=-PID(error_x)/8
            cmd_msg.linear.x=0.09
        # cmd_msg.angular.z=0
        elif error_x<0:
            cmd_msg.angular.z=PID(error_x)/8
            cmd_msg.linear.x=0.12
        # cmd_msg.angular.z=0
        else :
            cmd_msg.angular.z=0
            cmd_msg.linear.x = 0.3
    
    
    
    cmd_vel_pub.publish(cmd_msg)

def PID(error_x):

    global intgerror , differror 
    intgerror += error_x 
    differror = error_x -intgerror
    intgerror =error_x
    balance = error_x * KP + intgerror * KI + differror * KD 
    
    print(balance)
    return balance 
   
        

def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/camera/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    global cmd_vel_pub
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    #odom_sub = rospy.Subscriber(odom, Odometry, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()

#cv2.imshow('grey',img_grey)
    #cv2.namedWindow('Track')
    #cv2.resizeWindow('frame',700,512)
    # def track(x):
    #     pass
    # cv2.createTrackbar('hue min','frame',0,179,track)
    # cv2.createTrackbar('hue max','frame',179,179,track)
    # cv2.createTrackbar('sat min','frame',0,255,track)
    # cv2.createTrackbar('sat max','frame',255,255,track)
    # cv2.createTrackbar('val min','frame',0, 255,track)
    # cv2.createTrackbar('val max','frame',255,255,track)

    # while True:
    #     h_min = cv2.getTrackbarPos('hue min','frame')
    #     h_max = cv2.getTrackbarPos('hue min','frame')
    #     s_min = cv2.getTrackbarPos('hue min','frame')
    #     s_max = cv2.getTrackbarPos('hue min','frame')
    #     val_min = cv2.getTrackbarPos('hue min','frame')
    #     val_max = cv2.getTrackbarPos('hue min','frame')
    #     print(f'HUE MIN : {h_min} HUE MAX : {h_max} SAT MIN :{s_min} SAT MAX : {s_max} VAL MIN :{val_min} VAL MAX :{val_max}')
        
    #     if cv2.waitkey(1) &0xFF == ord('q'):
    #         break