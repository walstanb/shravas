#!/usr/bin/env python

'''
Team id:       
Author List:   Aditya Narayan Bhattacharya,Ninad Choksi, Krut Chitre, Walstan Baptista
Filename:      engine.py
Theme:         Quaddrop - Automated Aerial Delivery System
Functions:		Base engine for drone control
Global variables:
'''


from std_msgs.msg import Int32
from std_msgs.msg import Float64
import rospy
import time
import array as arr
from geometry_msgs.msg import PoseArray
import numpy
import av
import cv2,cv_bridge
import time
import datetime
import os
import tellopy
from pyzbar import pyzbar
import imutils
import time
from std_msgs.msg import String
import csvio
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

class pilot():
		
	'''
	Function Name: __init__
	Input:         None
	Output:        Initiates all the variables in the class ImgProc and creates subscribers 
	Logic:         initializes the value of the variables to predefined values
	Example Call:  It is called automatically when an object is created
	'''

	def __init__(self):
		self.homelocation=1        
		rospy.init_node('drone_pilot')
		
		self.wppub = rospy.Publisher('/wp_cords', PoseArray, queue_size=60)
		self.msgpub = PoseArray()
		self.gui_status = rospy.Publisher('status_msg', String, queue_size=1, latch=True)
		self.takeoff = rospy.Publisher('activation', Int32, queue_size=1, latch=True)

		rospy.Subscriber('whycon/poses', PoseArray, self.get_pose)
		rospy.Subscriber('/qr', String, self.setqr)
		rospy.Subscriber('drone_init', Int32, self.set_guicommand)

		self.counter = 0
		self.takeoffland = -1
		self.cruize = 10.0
		self.drone_x = 0.0
		self.drone_y = 0.0
		self.drone_z = 24.0
		self.home_x = 0.0
		self.home_y = 0.0
		self.wp_x = 0.0
		self.wp_y = 0.0
		self.wp_z = 0.0

		self.coordinatespub=[0.0,0.0,30.0]
		pose=Pose()
		pose.position = Point(*self.coordinatespub)
		self.msgpub.poses.append(pose)

		self.startrun = 0
		self.startend = 2
		self.coordinates=csvio.csvread('/home/walst/catkin_ws/src/shravas/scripts/coords.csv')

		for index in range(len(self.coordinates)):
			self.coordinates[index]['x'] = float(self.coordinates[index]['x'])
			self.coordinates[index]['y'] = float(self.coordinates[index]['y'])
			self.coordinates[index]['z'] = float(self.coordinates[index]['z'])
			self.coordinates[index]['delivery'] = int(self.coordinates[index]['delivery'])
		
		#print(self.coordinates)
		#self.coordinates=[{"x":0,"y":0,"Z":0,"qr":0,"delivery":0},{"x":-8.0,"y":4.0,"Z":20,"qr":"QuadDrop","delivery":2},{"x":0.7,"y":-0.63,"Z":20,"qr":"WANK","delivery":1},{"x":0,"y":0,"Z":0,"qr":0,"delivery":-1}]
		self.qr_pub="no code"
		self.last_time = 0.0
		self.loop_time = 0.02

	'''
	Function Name: check_delta
	Input:         Error allowed
	Output:        none
	Logic:         checks if drone is at specified position
	Example Call:  check_delta(0.2,0.5)
	'''
	
	def check_delta(self,err_xy,err_z):
		#Checks wether the drone is at its destination
		if((self.drone_x<(self.wp_x+err_xy)) & (self.drone_x>(self.wp_x-err_xy)) & (self.drone_y<(self.wp_y+err_xy)) & (self.drone_y>(self.wp_y-err_xy)) & (self.drone_z>(self.wp_z-err_z)) & (self.drone_z<(self.wp_z+err_z))):			
			self.counter+=1
		else:
			self.counter=0
		
	'''
	Function Name:	gotoloc
	Input:			travelling coordinates(x,y,z) and delta value to check at destination coordinates
	Output:			None
	Logic:			Assign the values to coordinates and publish them
	Example Call:	gotoloc(0.0,0.0,18.0,0.5,0.5)
	'''

	def gotoloc(self,x1,y1,z1,deltaxy,deltaz):
		self.wp_x=x1
		self.wp_y=y1
		self.wp_z=z1
		self.coordinatespub=[self.wp_x,self.wp_y,self.wp_z]
		pose=Pose()
		pose.position = Point(*self.coordinatespub)
		self.msgpub.poses=[]
		self.msgpub.poses.append(pose)
		self.gui_status.publish("Travelling to new location")
		self.gui_status.publish(str(self.wp_x)+","+str(self.wp_y)+","+str(self.wp_z))
		self.counter=0
		while(self.counter < 100):
			self.check_delta(deltaxy,deltaz)


	'''
	Function Name:	land
	Input:			None
	Output:			None
	Logic:			Check for delta at cruize height over home location and land when delta satisfied
	Example call:	land(0,index)
	'''		

	def land(self,endrun,index):
		self.moveahead=0
		self.takeoffland=0
		self.wp_z=26.0
		self.coordinatespub=[self.wp_x,self.wp_y,self.wp_z]
		pose=Pose()
		self.msgpub.poses=[]
		pose.position = Point(*self.coordinatespub)
		self.msgpub.poses.append(pose)
		self.check_delta(0.5,1.5)
		self.takeoff.publish(self.takeoffland)
		if(endrun==0):
			self.gui_status.publish("Waiting for authentication")
			while(self.moveahead!=1):
				if(self.qr_pub == self.coordinates[index]['qr']):
					self.moveahead=1
					self.takeoffland=1
					self.takeoff.publish(self.takeoffland)
					self.gui_status.publish("Authenticated")
					self.gui_status.publish("Qrcode Data :"+self.qr_pub)

			rospy.sleep(5)
			self.gui_status.publish("Taking off for next destination ")
			self.gotoloc(self.wp_x,self.wp_y,self.wp_z,1.0,2.0)
		else:
			self.takeoffland == -1
			self.takeoff.publish(self.takeoffland)	

	

	'''
	Function Name: 	fly
	Input:			Nil
	Output:			Next Delivery location
	Logic:			For delivery=1 Go for delivery and set the coordinates and for delivery=0 travel for no fly zone avoidance
	Example call:	fly()

	'''
	def fly(self):
		#rospy.sleep(10)
		while(self.startend != 1):
			rospy.sleep(0.0001)
		print("HERE !!")
		for index in range(len(self.coordinates)):
			if(self.coordinates[index]['delivery'] == 0):
				print("hello")
				self.takeoffland=1
				self.takeoff.publish(self.takeoffland)
				rospy.sleep(1)
				self.gotoloc(self.home_x,self.home_y,self.cruize,1.5,3.0)
			elif(self.coordinates[index]['delivery'] > 0):
				self.gotoloc(self.coordinates[index]['x'],self.coordinates[index]['y'],self.cruize,0.3,3.0)
				self.land(0,index)
			elif(self.coordinates[index]['delivery']== -2):
				self.gotoloc(self.coordinates[index]['x'],self.coordinates[index]['y'],self.cruize,0.3,3.0)
			elif(self.coordinates[index]['delivery'] == -1):
				self.gui_status.publish("ALL DELIVERIES COMPLETED GOING BACK HOME")
				self.gotoloc(self.home_x,self.home_y,self.cruize,0.3,3.0)
				self.land(1,index) 
			else:
				self.gui_status.publish("BAD COORDINATES GOING BACK HOME")
				self.gotoloc(self.home_x,self.home_y,self.cruize,0.3,3.0)
				self.land(1,index) 





########################   SUBSCRIBER FUNCTIONS    ########################	






	'''
	Function Name: get_pose
	Input:         takes poseArray message from whycon/poses
	Output:        sets the value of drone_x, drone_y, drone_z
	Logic:         subscribes to whycon/poses to get the coordinates of the whycon marker placed on the drone
	Example Call:  get_pose(data)
	'''

	def get_pose(self, pose):
		if(self.homelocation==1):
			self.home_x = pose.poses[0].position.x
			self.home_y = pose.poses[0].position.y
			self.homelocation=0

		
		self.drone_x = pose.poses[0].position.x
		self.drone_y = pose.poses[0].position.y
		self.drone_z = pose.poses[0].position.z
		self.takeoff.publish(self.takeoffland)
		self.wppub.publish(self.msgpub)

	'''
	Function Name: setqr
	Input:         takes qr data for authentication
	Output:        sets the value of qr matcher variable
	Logic:         subscribes to qr to get the qr code date of the user
	Example Call:  setqr(msg)
	'''

	def setqr(self, msg):
		self.qr_pub=msg.data

	'''
	Function Name: set_guicommand
	Input:         takes status flag from GUI for start and emergency land
	Output:        make publisher ready to send command to drone
	Logic:         sets the flag value
	Example Call:  set_guicommand(msg)
	'''

	def set_guicommand(self,msg): 
		self.startend=msg.data 	# 1 for start , 0 for land 
		if(self.startend == 0):
			self.takeoffland=-1
			self.takeoff.publish(self.takeoffland)
			self.startend=2


########################   MAIN   #########################




'''
Function Name:	main
Input:			none
Output:			none
Logic:			initializes send_data and calls fly 
Example Call:	called automatically
'''	
if __name__ == '__main__':
	test = pilot()
	test.fly()		