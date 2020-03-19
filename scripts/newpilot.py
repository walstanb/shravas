#!/usr/bin/env python
from std_msgs.msg import Int32
from std_msgs.msg import Float64
import rospy
import time
import array as arr
from geometry_msgs.msg import PoseArray

class ippilot():
	'''
	Function Name:__init__
	Input:        None
	Output:       Initiates all the variables in the class ImgProc and creates subscribers 
	Logic:        initializes the value of the variables to predefined values
	Example Call: It is called automatically when an object is created
	'''

	def __init__(self):
		rospy.init_node('ip_pilot')
		self.homelocation = 1
		rospy.Subscriber('whycon/poses', PoseArray,self.get_pose)
		rospy.Subscriber('/snr_stat', Float64, self.check_stuck)
		self.takeoff = rospy.Publisher('activation', Int32, queue_size=1)
		self.activate_ipflag = rospy.Publisher('ip_activation', Int32, queue_size=1)
		self.activate_ip_status = 0
		self.home_x = 0.0
		self.home_y = 0.0
		self.person_stuck = 0
		self.wp_x=0.0
		self.wp_y=0.0
		self.wp_z=0.0

		#Change all below value according to PC coordinates
		self.cruize = 10.0
		self.start_point_x = 0.0
		self.start_point_y = 0.0
		self.start_point_z = 13.0
		self.traverse_x = 0.4
		self.traverse_y = 0.0
		self.traverse_z = 0.2
		self.top_z = 10.0 
		self.x = rospy.Publisher('/wp/x', Float64, queue_size=1)
		self.y = rospy.Publisher('/wp/y', Float64, queue_size=1)
		self.z = rospy.Publisher('/wp/z', Float64, queue_size=1)

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

	def traverse(self):
		#self.activate_ip_status = not(self.activate_ip_status)
		#rospy.sleep(2)
		self.wp_x += self.traverse_x

	def ascend(self):
		#self.activate_ip_status = not(self.activate_ip_status)
		#rospy.sleep(2)
		while(self.wp_z < self.top_z):
			self.wp_z += self.traverse_z
			if(self.person_stuck == 1):
				rospy.sleep(6)
			else:
				rospy.sleep(1)

	def decend(self):
		#self.activate_ip_status = not(self.activate_ip_status)
		#rospy.sleep(2)
		while(self.wp_z > self.start_point_z):
			self.wp_z -= self.traverse_z
			if(self.person_stuck == 1):
				rospy.sleep(6)
			else:
				rospy.sleep(1)

	def search(self):
		self.takeoffland=1
		rospy.sleep(2)
		self.gotoloc(self.home_x,self.home_y,self.cruize,1.5,3.0)
		self.gotoloc(self.start_point_x,self.start_point_y,self.start_point_z,1.0,1.5)
		self.ascend()
		self.traverse()
		self.descend()
		#self.traverse()
		#self.ascend()
		self.gotoloc(self.home_x,self.home_y,self.cruize,1.5,3.0)
		self.takeoffland=0

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
		self.x.publish(self.wp_x)
		self.y.publish(self.wp_y)
		self.z.publish(self.wp_z)

	def check_stuck(self, data):
		self.person_stuck = data.data

if __name__ == '__main__' :
	test = ippilot()
	test.search()