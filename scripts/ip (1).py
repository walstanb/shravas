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
                rospy.Subscriber('/result', Int32, self.print_result)

		self.image_pub=rospy.Publisher('/image_pub', Image, queue_size=10)
		self.visit=rospy.Publisher('/visit', Int32, queue_size=1)

		self.ros_bridge = cv_bridge.CvBridge()

                # Holds the Image to work on
		self.image = 0
                self.frame = 0

                # Holds the lower and upper limits of HSV values of BGR colours
                self.redlow = np.array([160,140,140])
                self.redhigh = np.array([180,255,255])
                self.greenlow = np.array([20,150,150])
                self.greenhigh = np.array([70,255,255])
                self.bluelow = np.array([100,140,140])
                self.bluehigh = np.array([140,150,150])
                
                # Array containing coordinates of all pollinated plants
                self.pollinated = [[1407,352]]

                # Array containing coordinates of pollinated plants of respective colours
		self.red_pollinated = [[1407,352]]
                self.green_pollinated = [[1407,352]]
                self.blue_pollinated = [[1407,352]]

                # Counter for number of pollinations
                self.i = 0

                # Counter for number of pollinations of respective colours
                self.red_count=0
                self.blue_count=0
                self.green_count=0
		
                # Comparing variable with pollination status of last iteration
		self.prev_len = 0

                # Holds the coordinates of eyantra logo at the bottom of the flex sheet
                self.elogo_x = 383
                self.elogo_y = 275
                self.elogo_w = 393
                self.elogo_h = 436
                
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
                #self.frame = self.frame[:(h-30),50:(w-130),:]

                # Exluding the eyantra logo at the bottom of flex from processing it as RED coloured 
                #cv2.rectangle(self.frame,(self.elogo_x,self.elogo_y),(self.elogo_h,self.elogo_w),(0,0,0),-1)
                for j in (self.pollinated):
                	x1 = j[0]
                	y1 = j[1]
                	cv2.rectangle(self.frame,(x1-20,y1-15),(x1+25,y1+15),(0,0,0),-1)
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
                cv2.erode(self.frame,kernel,iterations = 1)
                self.frame = cv2.cvtColor(self.frame,cv2.COLOR_BGR2HSV)

                # Using the image seperately for BGR in order to apply respective hsv masking on respective variable
                self.framered = np.copy(self.frame)
                self.frameblue = np.copy(self.frame)
                self.framegreen = np.copy(self.frame)



                # Finding RED DAYLILY



                self.framered = cv2.inRange(self.framered,self.redlow,self.redhigh)
                python3,contours,hierarchy =cv2.findContours(self.framered,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
                plants = [[1,1]]

                # Avoiding processing if no red contours are found
                if len(contours) > 1 :
                	x = 0
                	y = 0
                	for crt in contours:
                		x1,y1,w1,h1 = cv2.boundingRect(crt)

                                #Removing the multiple contours in such a way that only 2 contours are displayed on 2 adjacent nodes
                                if (-80 > (x1-x) or (x1-x) > 80) or (-80 > (y1-y) or (y1-y) > 80):
                			x=x1
                			y=y1
                			plants.append([x,y])
                			self.i+=1
                                        self.red_count+=1
                			#print("RED DAYLILY POLLINATED")	   
                if(len(plants) >= 2):
                	self.pollinated.append(plants[1])
                        self.red_pollinated.append(plants[1])
                new_len = len(self.pollinated)-1   
                if(new_len != self.prev_len):
                	self.prev_len = new_len



                # Finding GREEN CARNATION                
                
                
                
                self.framegreen = cv2.inRange(self.framegreen,self.greenlow,self.greenhigh)
                python3,contours,hierarchy =cv2.findContours(self.framegreen,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
                plants = [[1,1]]

                # Avoiding processing if no green contours are found
                if len(contours) > 1 :
                	x = 0
                	y = 0
                	for crt in contours:
                		x1,y1,w1,h1 = cv2.boundingRect(crt)

                                #Removing the multiple contours in such a way that only 2 contours are displayed on 2 adjacent nodes
                		if (-75 > (x1-x) or (x1-x) > 75) or (-75 > (y1-y) or (y1-y) > 75):
                			x=x1
                			y=y1
                			plants.append([x,y])
                			self.i+=1
                                        self.green_count+=1
                			#print("GREEN CARNATION POLLINATED")	   
                if(len(plants) >= 2):
                	self.pollinated.append(plants[1])
                        self.green_pollinated.append(plants[1])
                new_len = len(self.pollinated)-1   
                if(new_len != self.prev_len):
                	self.prev_len = new_len



                # Finding BLUE DELPHINIUM



                self.frameblue = cv2.inRange(self.frameblue,self.bluelow,self.bluehigh)
                python3,contours,hierarchy =cv2.findContours(self.frameblue,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
                plants = [[1,1]]
                
                # Avoiding processing if no blue contours are found
                if len(contours) > 1 :
                	x = 0
                	y = 0
                	for crt in contours:
                		x1,y1,w1,h1 = cv2.boundingRect(crt)

                                #Removing the multiple contours in such a way that only 2 contours are displayed on 2 adjacent nodes
                		if (-75 > (x1-x) or (x1-x) > 75) or (-75 > (y1-y) or (y1-y) > 75):
                			x=x1
                			y=y1
                			plants.append([x,y])
                			self.i+=1
                                        self.blue_count+=1
                			#print("BLUE DELPHINIUM POLLINATED")	   
                if(len(plants) >= 2):
                	self.pollinated.append(plants[1])
                        self.blue_pollinated.append(plants[1])
                new_len = len(self.pollinated)-1   
                if(new_len != self.prev_len):
                	self.prev_len = new_len


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

                # Drawing red squares on plants having RED LEDs once pollinated
                for j in (self.red_pollinated):
                        x1 = j[0]
                        y1 = j[1]
                        cv2.rectangle(self.image,(x1-20,y1-15),(x1+25,y1+15),(0,0,255),3)
                
                # Drawing red squares on plants having GREEN LEDs once pollinated
                for j in (self.green_pollinated):
                        x1 = j[0]
                        y1 = j[1]
                        cv2.rectangle(self.image,(x1-20,y1-15),(x1+25,y1+15),(0,255,0),3)

                # Drawing red squares on plants having BLUE LEDs once pollinated        
                for j in (self.blue_pollinated):
                        x1 = j[0]
                        y1 = j[1]
                        cv2.rectangle(self.image,(x1-20,y1-15),(x1+25,y1+15),(255,0,0),3)


                self.image_pub.publish(self.ros_bridge.cv2_to_imgmsg(self.image, 'bgr8'))

        '''
        Function Name:  print_result
        Input:          Subscribed data high when all plants are pollinated
        Output:         Exact number of plants pollinated after visiting all plants
        Logic:          Prints the counted number of plants pollinated using counter 
        Example Call:   It is subscriber function so is called automatically and continously
                
        '''


        def print_result(self,msg):
                if(msg.data == 1):
                        print(str(self.red_count)+" RED DAYLILY "+str(self.green_count)+" GREEN CARNATION "+str(self.blue_count)+ " BLUE DELPHINIUM POLLINATED")

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
