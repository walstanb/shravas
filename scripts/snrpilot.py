#!/usr/bin/env python
from std_msgs.msg import Int32
from std_msgs.msg import Float64
import rospy
import array as arr
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

class ippilot():
	'''
	Function Name:__init__
	Input:        None
	Output:       Initiates all the variables in the class ImgProc and creates subscribers 
	Logic:        initializes the value of the variables to predefined values
	Example Call: It is called automatically when an object is created
	'''

	def __init__(self):
		rospy.init_node('snr_pilot')
		self.homelocation = 1
		self.wppub = rospy.Publisher('/wp_cords', PoseArray, queue_size=60)
		self.msgpub = PoseArray()

		self.coordinatespub=[0.0,0.0,30.0]
		pose=Pose()
		pose.position = Point(*self.coordinatespub)
		self.msgpub.poses.append(pose)



		rospy.Subscriber('whycon/poses', PoseArray,self.get_pose)
		rospy.Subscriber('/snr/stat', Int16, self.check_stuck)
		self.takeoff = rospy.Publisher('activation', Int32, queue_size=1)
		self.activate_ipflag = rospy.Publisher('ip_activation', Int32, queue_size=1)
		self.activate_ip_status = 0
		self.home_x = 0.0
		self.home_y = 0.0
		self.person_stuck = 0
		self.wp_x=0.0
		self.wp_y=0.0
		self.wp_z=0.0
		self.drone_x=0.0
		self.drone_y=0.0
		self.drone_z=0.0

		#Change all below value according to PC coordinates
		self.cruize = 10.0
		self.start_point_x = 0.0
		self.start_point_y = 0.0
		self.start_point_z = 13.0
		self.traverse_x = 0.4
		self.traverse_y = 0.0
		self.traverse_z = 0.2
		self.top_z = 10.0 	
		self.drone_wait=1

	def check_delta(self,err_xy,err_z):
		if((self.drone_x<(self.wp_x+err_xy)) & (self.drone_x>(self.wp_x-err_xy)) & (self.drone_y<(self.wp_y+err_xy)) & (self.drone_y>(self.wp_y-err_xy)) & (self.drone_z>(self.wp_z-err_z)) & (self.drone_z<(self.wp_z+err_z))):
			self.counter+=1
		else:
			self.counter=0
	
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

	def WaitAtPoint(self):
		temp_x=self.wp_x
		temp_y=self.wp_y
		temp_z=self.wp_z
		gotoloc(drone_x,drone_y,drone_z,1.0,0.3)
		rospy.sleep(5)
		gotoloc(temp_x,temp_y,temp_z,1.0,0.3


	def search(self):
		self.takeoffland=1
		rospy.sleep(2)
		self.wp_z=14.0
		self.counter=0
		while(self.counter < 100):
			# if(self.person_stuck == 1 and self.drone_wait == 1):
			# 	self.drone_wait = 0
			# 	rospy.sleep(5)
			# elif(self.person_stuck == 0 and self.drone_wait == 0):
			# 	self.drone_wait = 1
			if(self.person_stuck == 1):
				WaitAtPoint()
			self.check_delta(1.0,0.3)
		self.wp_z=20.0
		self.counter=0
		while(self.counter < 100):
			self.check_delta(1.0,0.3)
		self.wp_z=14.0
		self.counter=0
		while(self.counter < 100):
			self.check_delta(1.0,0.3)
		self.takeoffland=-1

	## SUBSCRIBER FUNCTIONS ##

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
		self.activate_ipflag.publish(self.activate_ip_status)
		self.wppub.publish(self.msgpub)

	def check_stuck(self, data):
		#self.person_stuck = data.data
		stuck = data.data
		if(stuck == 0 and self.drone_wait == 1):
			self.person_stuck = 0
		elif(stuck == 0 and self.drone_wait == 0):
			self.person_stuck = 0
			self.drone_wait = 1
		elif(stuck == 1 and self.drone_wait = 1):
			self.person_stuck = 1
			self.drone_wait = 0
		elif(stuck = 1 and self.drone_wait = 0):
			self.person_stuck = 0		
			


if __name__ == '__main__' :
	test = ippilot()
	test.search()
