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
from geometry_msgs.msg import PoseArray


class pilot():
		
	'''
	Function Name:__init__
	Input:        None
	Output:       Initiates all the variables in the class ImgProc and creates subscribers 
	Logic:        initializes the value of the variables to predefined values
	Example Call: It is called automatically when an object is created
	'''

	def __init__(self):
		self.homelocation=1        
		rospy.init_node('drone_pilot')
		self.x = rospy.Publisher('/wp/x', Float64, queue_size=1)
		self.y = rospy.Publisher('/wp/y', Float64, queue_size=1)
		self.z = rospy.Publisher('/wp/z', Float64, queue_size=1)
		self.takeoff = rospy.Publisher('activation', Int32, queue_size=1)
		rospy.Subscriber('whycon/poses', PoseArray, self.get_pose)
		self.counter=0
		self.takeoffland=-1
		self.cruize=15.0
		self.drone_x=0.0
		self.drone_y=0.0
		self.drone_z=24.0
		self.home_x=0.0
		self.home_y=0.0
		self.startrun=0
		self.coordinates={ 1:{"x":0,"y":0,"Z":0,"delivery":0,"id":0},2:{"x":1.5,"y":1.5,"Z":15,"delivery":1,"id":2},3:{"x":-0.5,"y":-0.5,"Z":15,"delivery":0,"id":1},4:{"x":0,"y":0,"Z":0,"delivery":0,"id":-1}}

	'''
	Function Name:check_delta
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
		print("Inside gotoloc")
		print(self.wp_x)
		print(self.wp_y)
		print(self.wp_z)
		
		self.counter=0
		while(self.counter < 100):
			self.check_delta(deltaxy,deltaz)


	'''
	Function Name:	land
	Input:			None
	Output:			None
	Logic:			Check for delta at cruize height over home location and land when delta satisfied
	Example call:	land(0)
	'''		

	def land(self,endrun):
		self.moveahead=0
		print("Landing now")
		#self.takeoffland=0
		self.wp_z=19.0
		rospy.sleep(6)
		#self.takeoff.publish(self.takeoffland)
		if(endrun==0):
			while(self.moveahead!=1):
				print("Waiting for authentication")
				rospy.sleep(5)
				self.moveahead=1 # Set due to QR code module is not ready. Remove once QR module is ready and is integrated
			rospy.sleep(2)
			#self.autopilot=True
			print("Taking off for next destination ")
			#self.takeoffland=1
			#self.takeoff.publish(self.takeoffland)
			self.gotoloc(self.wp_x,self.wp_y,self.wp_z,1.0,2.0)	

	

	'''
	Function Name: 	fly
	Input:			Nil
	Output:			Next Delivery location
	Logic:			For delivery=1 Go for delivery and set the coordinates and for delivery=0 travel for no fly zone avoidance
	Example call:	fly()

	'''
	def fly(self):
		for index in self.coordinates:
			if(self.coordinates[index]['id'] == 0):
				self.takeoffland=1
				rospy.sleep(2)
				self.gotoloc(self.home_x,self.home_y,self.cruize,1.5,3.0)
			elif(self.coordinates[index]['id'] > 0):
				self.gotoloc(self.coordinates[index]['x'],self.coordinates[index]['y'],self.cruize,0.3,3.0)
				self.land(0)
			elif(self.coordinates[index]['id']== -2):
				self.gotoloc(self.coordinates[index]['x'],self.coordinates[index]['y'],self.cruize,0.3,3.0)
			elif(self.coordinates[index]['id'] == -1):
				print("ALL DELIVERIES COMPLETED GOING BACK HOME")
				self.gotoloc(self.home_x,self.home_y,self.cruize,0.3,3.0)
				self.land(1) 
			else:
				print("BAD COORDINATES GOING BACK HOME")
				self.gotoloc(self.home_x,self.home_y,self.cruize,0.3,3.0)
				self.land(1) 

################################################ SUBSCRIBER FUNCTION ###################################################################################

	'''
	Function Name:get_pose
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
		self.x.publish(self.wp_x)
		self.y.publish(self.wp_y)
		self.z.publish(self.wp_z)
		
if __name__ == '__main__':
	test = pilot()
	test.fly()		