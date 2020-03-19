#!/usr/bin/env python
'''

* Team Id               : eYRC#86
* Author List           : Krut Chitre, Yashi Pandit, Ninad Choksi,Aditya Narayan Bhattacharya
* Filename              : eYRC#86_IP_task4.py
* Theme                 : Pollinator Bee - eYRC Specific
* Functions             : __init__, color_detect, show_pollination
* Global Variables      : None

'''

from std_msgs.msg import Int32
from std_msgs.msg import Float64
import rospy
import time
import array as arr
import numpy as np
import cv2,cv_bridge
from sensor_msgs.msg import Image


class ImgProc():
        '''

        #Function Name:__init__
        Input:        None
        Output:       Initiates all the variables in the class ImgProc and creates subscribers 
        Logic:        initializes the value of the variables to predefined values
        Example Call: It is called automatically when an object is created

        ''' 
 	def __init__(self):
                
		rospy.init_node('image_processing')

		rospy.Subscriber('/usb_cam/image_rect_color', Image, self.color_detect)
                rospy.Subscriber('/whycon/image_out', Image, self.show_pollination)
                #rospy.Subscriber('/result', Int32, self.print_result)

		self.image_pub=rospy.Publisher('/image_pub', Image, queue_size=10)
		#self.visit=rospy.Publisher('/visit', Int32, queue_size=1)

		self.ros_bridge = cv_bridge.CvBridge()

        '''
        Function Name:color_detect()
        Input:        Image from /usb_cam/image_rect_color
        Output:       Finds the newly pollinated plants and stores its cordinates in single coordinate 
        Logic:        Finds contours on the input image and excludes that partition while checking for contours in next iteration
        Example Call: It is subscriber function so is called automatically and continously
        
        '''

	def color_detect(self,msg):

                self.frame = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                h,w,c = self.frame.shape
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
                cv2.erode(self.frame,kernel,iterations = 1)
                self.frame = cv2.cvtColor(self.frame,cv2.COLOR_BGR2HSV)

                self.visit.publish(self.i)

        '''
        Function Name:show_pollination()
        Input:        Image from whycon/image_out
        Output:       Draws rectangles on pollinated plant of same colour as that of led light 
        Logic:        The coordinates of pollinated plants are stored in an array and rectangles are drawn taking these points in consideration 
        Example Call: It is subscriber function so is called automatically and continously
                
        '''

        def show_pollination(self,msg):

                self.image = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

                
'''
Function Name:  main
Input:          None
Output:         None
Logic:          initializes send_data and starts image processing while the drone is not shut down 
Example Call:   called automatically
                
'''

if __name__ == '__main__':
	test = ImgProc()
	while not rospy.is_shutdown():
		rospy.spin()
		sys.exit(1)
