#!/usr/bin/env python

from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import rospy
import tellopy
import time
import datetime
import os
import csvio
import getpass
import nofly

class pilot():

	def create_pose_array(self):
		self.posepos.position = Point(*self.coordinatespub)
		self.msgpub.poses=[]
		self.msgpub.poses.append(self.posepos)
		
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
		self.msgpub = PoseArray()
		self.posepos = Pose()

		self.wppub = rospy.Publisher('/wp_cords', PoseArray, queue_size=60)
		self.gui_status = rospy.Publisher('status_msg', String, queue_size=1, latch=True)
		self.takeoff = rospy.Publisher('activation', Int32, queue_size=1, latch=True)
		self.progress = rospy.Publisher('/progbar', Int16, queue_size=1)
		
		rospy.Subscriber('whycon/poses', PoseArray, self.get_pose)
		rospy.Subscriber('/qr', String, self.setqr)
		rospy.Subscriber('drone_init', Int32, self.set_guicommand)

		self.cruize = 15.0
		self.delivery_z = 18.0

		self.counter = 0
		self.takeoffland = -100
		self.drone_x = 0.0
		self.drone_y = 0.0
		self.drone_z = 24.0
		self.home_x = 0.0
		self.home_y = 0.0
		self.startend = 2

		#self.authenticationflag = 0 # Do not remove to be uncommented when feature of invalid QR is to be used

		self.coordinatespub=[0.0,0.0,30.0]
		self.create_pose_array()
		self.coordinates=csvio.csvread('/home/'+getpass.getuser()+'/catkin_ws/src/shravas/src/coordinates.csv')
		self.coordinates1=nofly.main(self.coordinates)
		print(self.coordinates)
		print(self.coordinates1)
		for index in range(len(self.coordinates)):
			self.coordinates[index]['x'] = float(self.coordinates[index]['x'])
			self.coordinates[index]['y'] = float(self.coordinates[index]['y'])
			self.coordinates[index]['z'] = float(self.coordinates[index]['z'])
			self.coordinates[index]['delivery'] = int(self.coordinates[index]['delivery'])
		
		self.qr_pub="no code"

	'''
	Function Name: check_delta
	Input:         Error allowed
	Output:        none
	Logic:         checks if drone is at specified position
	Example Call:  check_delta(0.2,0.5)
	'''
	
	def check_delta(self,wp_x,wp_y,wp_z,err_xy,err_z):
		self.counter=0
		while(self.counter < 100):
			if((self.drone_x<(wp_x+err_xy)) & (self.drone_x>(wp_x-err_xy)) & (self.drone_y<(wp_y+err_xy)) & (self.drone_y>(wp_y-err_xy)) & (self.drone_z>(wp_z-err_z)) & (self.drone_z<(wp_z+err_z))):			
				self.counter+=1
			else:
				self.counter=0
			if(self.startend!=1):
				self.counter=200
		self.progress.publish(0)

	'''
	Function Name:	gotoloc
	Input:			travelling coordinates(x,y,z) and delta value to check at destination coordinates
	Output:			None
	Logic:			Assign the values to coordinates and publish them
	Example Call:	gotoloc(0.0,0.0,18.0,0.5,0.5)
	'''

	def gotoloc(self,wp_x,wp_y,wp_z,deltaxy,deltaz):
		self.gui_status.publish("Travelling to new location")
		self.coordinatespub=[wp_x,wp_y,wp_z]
		self.create_pose_array()
		self.gui_status.publish(str(wp_x)+","+str(wp_y)+","+str(wp_z))
		self.check_delta(wp_x,wp_y,wp_z,deltaxy,deltaz)
	
	'''
	Function name : check_qr
	Logic	      : To check and match the qr code shown by customer
	'''
	
	def check_qr(self,index):
		moveahead=0
		start_time = time.time()
		while(moveahead!=1):
			end_time = time.time()
			if((end_time - start_time) < 12):
				if(self.qr_pub == self.coordinates[index]['qr']):
					#self.authenticationflag = 1	# Do not remove to be uncommented when feature of invalid QR is to be used
					moveahead=1
					self.gui_status.publish("Customer Authenticated")
				#else:						# Do not remove to be uncommented when feature of invalid QR is to be used
					#self.authenticationflag = 0
			else:
				moveahead=1
				self.gui_status.publish("No Customer found, taking package back to home")
			if(self.startend!=1):
				moveahead=1

	'''
	Function Name:	land
	Input:			None
	Output:			None
	Logic:			Check for delta at cruize height over home location and land when delta satisfied
	Example call:	land(0,index)
	'''		

	def land(self,endrun,index):
		if(endrun == 1):
			self.takeoffland = -1
		else:
			self.takeoffland = 0
			self.coordinatespub.pop()
			self.coordinatespub.append(self.delivery_z)
			self.create_pose_array()
			self.check_delta(self.coordinatespub[0],self.coordinatespub[1],self.coordinatespub[2],0.5,1.5)	
			self.gui_status.publish("Waiting for authentication")			
			self.check_qr(index)
			rospy.sleep(3)
			self.takeoffland=1
			self.gui_status.publish("Taking off for next destination ")
				
	'''
	Function Name: 	fly
	Input:			Nil
	Output:			Next Delivery location
	Logic:			For delivery=1 Go for delivery and set the coordinates and for delivery=0 travel for no fly zone avoidance
	Example call:	fly()

	'''
	def fly(self):
		while(self.startend != 1):
			rospy.sleep(0.0001)
		self.home_x = self.drone_x
		self.home_y = self.drone_y

		for index in range(len(self.coordinates)):
			
			if(self.startend!=1):
				break
			elif(self.coordinates[index]['delivery'] == 0):
				self.gui_status.publish("Takeoff")
				self.takeoffland=1
				rospy.sleep(3)
				self.gotoloc(self.home_x,self.home_y,self.cruize,1.5,3.0)
			
			elif(self.coordinates[index]['delivery'] > 0):
				self.gotoloc(self.coordinates[index]['x'],self.coordinates[index]['y'],self.cruize,0.3,3.0)
				self.land(0,index)
				self.gotoloc(self.coordinates[index]['x'],self.coordinates[index]['y'],self.cruize,0.3,3.0)
			
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
		self.startend=msg.data 	# 1 for start , 0 for land ,-1 for call back
		if(self.startend == 0):
			rospy.sleep(1)
			self.takeoffland=-1
			self.takeoff.publish(self.takeoffland)
		elif(self.startend == -1):
			rospy.sleep(1)
			self.takeoffland=1
			self.coordinatespub = [self.home_x,self.home_y,self.cruize]
			self.create_pose_array()
			self.check_delta(self.home_x,self.home_y,self.cruize,0.5,1.5)
			self.takeoffland=-1


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
	time.sleep(20)	