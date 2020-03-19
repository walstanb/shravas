#!/usr/bin/env python

#Team id:       
#Author List:   Aditya Narayan Bhattacharya,Ninad Choksi, Krut Chitre, Walstan Baptista
#Filename:      engine.py
#Theme:         QuadDrop
#Functions:
#Global variables:

from pid_tune.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import String
import rospy
import tellopy
import time
import array as arr
import numpy as np
import cv2,cv_bridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray



class engine():
	"""docstring for request_data"""
        #Function Name:__init__
        #Input:        none
        #Output:       initiates all the variables in the class send data and creates subscribers 
        #Logic:        initializes the value of the variables to predefined values
        #Example Call: called automatically when an object is created
	
	def __init__(self):
		rospy.init_node('engine')

		self.drone_x = 0
		self.drone_y = 0
		self.drone_z = 0
		self.start = False
		self.drone = tellopy.Tello()

		rospy.Subscriber('whycon/poses', PoseArray, self.get_pose)
		rospy.Subscriber('/input_key', Int16, self.emergencycontrol)



    #Function Name:get_pose
    #Input:         takes poseArray message from whycon/poses
    #Output:        sets the value of drone_x, drone_y, drone_z
    #Logic:         subscribes to whycon/poses to get the coordinates of the whycon marker placed on the drone
    #Example Call:  get_pose(data)
	def get_pose(self, pose):
		#This is the subscriber function to get the whycon poses
		#The x, y and z values are stored within the drone_x, drone_y and the drone_z variables
		self.drone_x = pose.poses[0].position.x
		self.drone_y = pose.poses[0].position.y
		self.drone_z = pose.poses[0].position.z





	
#######################################################################################################################################################################################################	
#MANUAL CONTROL (To take ccontrol of the drone if it flies out of range)
		
    #Function Name:emergencycontrol
    #Input:         takes value of key input from keyboard
    #Output:        calls drone control function according to the input
    #Logic:         take the values and choose a function to call using conditional statements
    #Example Call:  emergencycontrol(data)
	def emergencycontrol(self, msg):
		self.key_value = msg.data

		if self.key_value == 105:  #i
			print("UP")
			#self.drone.up(100)
			#rospy.sleep(0.1)

		if self.key_value == 107:  #k
			print("DOWN")
			#self.drone.down(100)
			#rospy.sleep(0.1)

		if self.key_value == 106:  #j
			print("Connect and TAKEOFF")
			self.drone.connect()
	        self.drone.wait_for_connection(60.0)
	        self.drone.takeoff()
	        rospy.sleep(2)

		if self.key_value == 108:  #l
			print("LAND and disconnect. Thankyou for flying with SRAVAS")
			self.drone.down(50)
	        rospy.sleep(5)
	        self.drone.land()
	        rospy.sleep(5)
	        self.drone.quit()
			

		if self.key_value == 100:  #d
			print("RIGHT")
			#self.drone.right(100)
			#rospy.sleep(0.1)

		if self.key_value == 119:  #w
			print("FORWARD")
			#self.drone.forward(100)
			#rospy.sleep(0.1)

		if self.key_value == 115:  #s
			print("BACKWARD")
			#self.drone.backward(100)
			#rospy.sleep(0.1)

		if self.key_value == 97:  #a
			print("LEFT")
			#self.drone.left(100)
			#rospy.sleep(0.1)

		if self.key_value == 114:  #r
			print("QUIT!!!")
			self.drone.quit()

		if self.key_value == 116:  #t
			print("You pressed T")

		if self.key_value == 112:  #p
			print("Place your hand under tello within the next 5 seconds")
			#self.drone.palm_land()
			#rospy.sleep(5)

		if self.key_value == 109:  #m
			print("COLCKWISE")
			#self.drone.clockwise(100)
			#rospy.sleep(0.1)

		if self.key_value == 110:  #n
			print("COUNTER_COLCKWISE")
			#self.drone.counter_clockwise(100)
			#rospy.sleep(0.1)

		if self.key_value == 43:  #+
			print("You pressed +")

		if self.key_value == 49:  #1
			print("FLIP_FORWARD")
			#self.drone.flip_forward()
			#rospy.sleep(2)

		if self.key_value == 50:  #2
			print("FLIP_BACKWARD")
			#self.drone.flip_back()
			#rospy.sleep(2)

		if self.key_value == 51:  #3
			print("FLIP_RIGHT")
			#self.drone.flip_right()
			#rospy.sleep(2)

		if self.key_value == 52:  #4
			print("FLIP_LEFT")
			#self.drone.flip_left()
			#rospy.sleep(2)



#########################################################################################################################################################################################################
#MAIN
#Function Name:main
#Input:none
#Output:none
#Logic:initializes send_data and starts autocontrol if the drone is not shut down
#Example Call:called automatically

if __name__ == '__main__':
	test = engine()
	while not rospy.is_shutdown():
		rospy.spin()
		sys.exit(1)



