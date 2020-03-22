#!/usr/bin/env python

'''
Team id:       
Author List:   Aditya Narayan Bhattacharya,Ninad Choksi, Krut Chitre, Walstan Baptista
Filename:      engine.py
Theme:         Quaddrop - Automated Aerial Delivery System
Functions:		Base engine for drone control
Global variables:
'''


from pid_tune.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import String
from tello_driver.msg import TelloStatus
import rospy
import tellopy
import time
import array as arr
import numpy
import math
import cv2,cv_bridge
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
import av
import imutils
from pyzbar import pyzbar



class engine():
	"""docstring for request_data"""
	'''
	Function Name:__init__
	Input:        none
	Output:       initiates all the variables in the class send data and creates subscribers 
	Logic:        initializes the value of the variables to predefined values
	Example Call: called automatically when an object is created
	'''

	def __init__(self):
		rospy.init_node('snr_engine')
		self.flag=1

		#Variable to flag when the PID should take control of the drone
		self.autopilot  = False

		self.activate_takeoff = 1

		self.drone = tellopy.Tello()
		self.drone.connect()
		self.drone.wait_for_connection(10.0)

		rospy.Subscriber('whycon/poses', PoseArray, self.get_pose)
		rospy.Subscriber('/wp/x', Float64, self.getcoordx)
		rospy.Subscriber('/wp/y', Float64, self.getcoordy)
		rospy.Subscriber('/wp/z', Float64, self.getcoordz)
		rospy.Subscriber('/whycon/poses', PoseArray, self.autocontrol)
		rospy.Subscriber('activation', Int32, self.takeoffland)
		rospy.Subscriber('/input_key', Int16, self.manual)
		#self.drone.subscribe(self.drone.EVENT_FLIGHT_DATA, self.cb_status_log)
		#self.drone.subscribe(self.drone.EVENT_LOG_DATA, self.cb_data_log)

		self.droneobject = rospy.Publisher('/drone_ref', String, queue_size=1)
		self.qrcode = rospy.Publisher('/qr', String, queue_size=1)
		self.image_pub = rospy.Publisher('drone_feed', Image, queue_size=10)
		self.stats = rospy.Publisher('/stats', String, queue_size=1)
		
		#Holds the current coordinates of the drone (recieved from whycon/poses)
		self.drone_x = 0
		self.drone_y = 0
		self.drone_z = 0

		#Holds the coordinates to be achieved by the drone
		self.wp_x=0.0
		self.wp_y=0.0
		self.wp_z=20.0

		#Holds the z axis coordinate of the drone stage (beehive)
		self.ground=0

		#PID constants for Roll
		self.kp_roll = 15.0
		self.ki_roll = 0.01
		self.kd_roll = 4.0

		#PID constants for Pitch
		self.kp_pitch = 15.0
		self.ki_pitch = 0.02
		self.kd_pitch = 4.0

		#PID constants for Throttle
		self.kp_throt = 15.0
		self.ki_throt = 1.0
		self.kd_throt = 2.5

		#Variables to selectively activate PID for pitch roll throttle
		self.activ_roll = True
		self.activ_pitch = True
		self.activ_throt = True

		#Correction values after PID is computed
		self.correct_roll = 0.0
		self.correct_pitch = 0.0
		self.correct_throt = 0.0

		#Loop time for PID computation.
		self.last_time = 0.0
		self.loop_time = 0.02

		#Variables to store previous cycle errors to aid PID
		self.iterm_pitch=0.0
		self.preverr_pitch=0.0
		self.iterm_roll=0.0
		self.preverr_roll=0.0
		self.iterm_throt=0.0
		self.preverr_throt=0.0
		self.counter=0
		
		self.ros_bridge = cv_bridge.CvBridge()





########################   AUTOPILOT    ########################	





	'''
	Function Name: autocontrol
	Input:		   none
	Output:        none
	Logic:         hand over drone control to the PID
	Example Call:  autocontrol()
	'''

	def autocontrol(self,msg):
		if (self.autopilot):	
			self.calc_pid()

			pitch_value = int(0 - self.correct_pitch)
			pitch_value = self.limit(pitch_value, 69, -69)
															
			roll_value = int(0 + self.correct_roll)
			roll_value = self.limit(roll_value, 69, -69)
															
			throt_value = int(0 - self.correct_throt)
			throt_value = self.limit(throt_value, 42, -42)

			yaw_value = 0									
			self.rc(throt_value,pitch_value,roll_value,yaw_value)

	'''

	Function Name: limit
	Input:         value, upper, lower
	Output:        value in limits
	Logic:         simple if else logic to determine if value allowed(Use this function to limit the maximum and minimum values you send to your drone)
	Example Call:  limit(input, max, min)
	
	'''

	def limit(self, input_value, max_value, min_value):
		if input_value > max_value:
			return max_value
		if input_value < min_value:
			return min_value
		else:
			return input_value

			
	'''
	Function Name: calc_pid
	Input:         none
	Output:        none
	Logic:         calls the pid functions after a specific time duration
	Example Call:  calc_pid()
	'''

	def calc_pid(self):
		self.seconds = time.time()
		current_time = self.seconds - self.last_time
		if(current_time >= self.loop_time):
			if self.activ_roll:   
				self.pid_roll()
			if self.activ_pitch:
				self.pid_pitch()
			if self.activ_throt:
				self.pid_throt()
			self.last_time = self.seconds

	'''	
	Function Name: rc
	Input:         throttle, roll, pitch, yaw inputs in range -100 to 100
	Output:        send the corresponding commands to drone
	Logic:         uses drone functions to send values to tello lib
	Example Call:  pid_yaw()
	'''

	def rc(self, throttle, pitch, roll, yaw):
		self.drone.set_throttle(throttle/100.0)
		self.drone.set_pitch(pitch/100.0)
		self.drone.set_roll(roll/100.0)
		self.drone.set_yaw(yaw/100.0)

	'''
	Function Name:	pid_roll
	Input:			uses drone_x
	Output:        	set value of correct_roll
	Logic:         	uses PID algorithm to estimate the correction to be made in rcRoll to achieve required Y coordinate
	Example Call:  	pid_roll()
	'''
	
	def pid_roll(self):
		self.err_roll=self.wp_x-self.drone_x
		self.iterm_roll+=self.err_roll*self.loop_time
		dErr_roll=(self.err_roll-self.preverr_roll)/self.loop_time
		self.correct_roll=((self.kp_roll*self.err_roll)+(self.ki_roll*self.iterm_roll)+(self.kd_roll*dErr_roll))
		self.preverr_roll=self.err_roll
		self.stats.publish("Roll PID : "+str(self.correct_roll)) 

	'''
	Function Name:	pid_pitch
	Input:         	uses drone_y
	Output:        	set value of correct_pitch
	Logic:         	uses PID algorithm to estimate the correction to be made in rcPitch to achieve required X coordinate
	Example Call:  	pid_pitch()	
	'''	
	
	def pid_pitch(self):
		self.err_pitch=self.wp_y-self.drone_y
		self.iterm_pitch+=self.err_pitch*self.loop_time
		dErr_pitch=(self.err_pitch-self.preverr_pitch)/self.loop_time
		self.correct_pitch=((self.kp_pitch*self.err_pitch)+(self.ki_pitch*self.iterm_pitch)+(self.kd_pitch*dErr_pitch))
		self.preverr_pitch=self.err_pitch
		self.stats.publish("Pitch PID : "+str(self.correct_pitch))

	'''
	Function Name:	pid_throt
	Input:         	uses drone_z
	Output:        	set the value of correct_throt
	Logic:         	uses PID algorithm to estimate the correction to be made in rcThrot to achieve required Z coordinate
	Example Call:  	pid_throt()
	'''

	def pid_throt(self):
		self.err_throt=self.wp_z-self.drone_z
		self.iterm_throt+=self.err_throt*self.loop_time
		dErr_throt=(self.err_throt-self.preverr_throt)/self.loop_time
		self.correct_throt=((self.kp_throt*self.err_throt)+(self.ki_throt*self.iterm_throt)+(self.kd_throt*dErr_throt))
		self.preverr_throt=self.err_throt
		self.stats.publish("Throttle PID : "+str(self.correct_throt))







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
		
	
	'''
	Function Name: set_pid_alt
	Input:         the values of kp,ki and kd
	Output:        sets the values for kp_throt,ki_throt and kd_throt
	Logic:         copy the values from pid_val to the object's variables
	Example Call:  set_pid_alt(data)
	
	'''

	def set_pid_alt(self,pid_val):
		self.kp_throt = float(pid_val.Kp)/10
		self.ki_throt = float(pid_val.Ki)/10 
		self.kd_throt = float(pid_val.Kd)/10

	'''
	Function Name:	set_pid_roll
	Input: 			the values of kp,ki and kd
	Output: 		sets the values for kp_roll,ki_roll and kd_roll
	Logic:			copy the values from pid_val to the object's variables
	Example Call:	set_pid_roll(data)
	'''

	def set_pid_roll(self,pid_val):
		self.kp_roll = float(pid_val.Kp)/10
		self.ki_roll = float(pid_val.Ki)/10
		self.kd_roll = float(pid_val.Kd)/10 
		
	'''	
	Function Name:	set_pid_pitch
	Input: 			the values of kp,ki and kd
	Output: 		sets the values for kp_pitch,ki_pitch and kd_pitch
	Logic: 			copy the values from pid_val to the object's variables
	Example Call:	set_pid_pitch(data)
	'''

	def set_pid_pitch(self,pid_val):
		self.kp_pitch = float(pid_val.Kp)/10
		self.ki_pitch = float(pid_val.Ki)/10
		self.kd_pitch = float(pid_val.Kd)/10

	'''	
	Function Name:	getcoordx
	Input: 			x axis value from pilot
	Output: 		value assigned to x of waypoint
	Logic: 			copy the values from published x to x of waypoint
	Example Call:	getcoordx(x)
	'''

	def getcoordx(self,x):
		self.wp_x=x.data

	'''	
	Function Name:	getcoordy
	Input: 			y axis value from pilot
	Output: 		value assigned to y of waypoint
	Logic: 			copy the values from published y to y of waypoint
	Example Call:	getcoordy(y)
	'''

	def getcoordy(self,y):
		self.wp_y=y.data

	'''	
	Function Name:	getcoordz
	Input: 			z axis value from pilot
	Output: 		value assigned to z of waypoint
	Logic: 			copy the values from published z to z of waypoint
	Example Call:	getcoordz
	'''

	def getcoordz(self,z):
		self.wp_z=z.data

	'''	
	Function Name:	takeoffland
	Input: 			data flag from pilot for takeoff or land
	Output: 		Drone takeoff/land
	Logic: 			Trigger takeoff/land on respective flag status
	Example Call:	takeoffland(1)
	'''

	def takeoffland(self,ddata):
		if(ddata.data==1 and self.activate_takeoff==1):
			self.drone.takeoff()
			print("Takeoff")
			self.activate_takeoff=0
			self.autopilot = True
			self.flag=1
		elif((ddata.data == 0 or ddata.data == -1) and self.activate_takeoff == 0):
			if(ddata.data == -1):
				self.drone.land()
			print("Land")
			self.activate_takeoff=1
			self.flag=0
			#self.autopilot=False

	def feed(self):
		rospy.sleep(5)
		print("Starting feed")
		print("Starting feed")
		self.container = av.open(self.drone.get_video_stream())
		self.vid_stream = self.container.streams.video[0]
		for packet in self.container.demux((self.vid_stream,)):
			for frame in packet.decode():
				image = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)
				image = imutils.resize(image, width=400)
				
				# find the barcodes in the frame and decode each of the barcodes
				if(self.flag==0):
					print("Checking for barcodes")

					barcodes = pyzbar.decode(image)
					for barcode in barcodes:
						#self.flag=1
						barcodeData = barcode.data.decode("utf-8")
						print(barcodeData)
				

				# PUBLISH SOMETHING ELSE OR CHANGE THE BARCODE DATA ONCE AGAIN


				image = imutils.resize(image, width=720)

				x,y = 360,270
				image = cv2.line(image,(x,y-10),(x,y+10),(0,0,255),1)
				image = cv2.line(image,(x-10,y),(x+10,y),(0,0,255),1)

				self.image_pub.publish(self.ros_bridge.cv2_to_imgmsg(image, 'bgr8'))








########################   MANUAL CONTROL    ########################	





	'''
	Function Name:	rc
	Input:         throttle, roll, pitch, yaw inputs in range -100 to 100
	Output:        send the corresponding commands to drone
	Logic:         uses drone functions to send values to tello lib
	Example Call:  pid_yaw()
	'''

	def rc_manual(self, throttle, pitch, roll, yaw):
		self.drone.set_throttle(throttle/100.0)
		self.drone.set_pitch(pitch/100.0)
		self.drone.set_roll(roll/100.0)
		self.drone.set_yaw(yaw/100.0)
		rospy.sleep(0.1)
		self.drone.set_throttle(0)
		self.drone.set_pitch(0)
		self.drone.set_roll(0)
		self.drone.set_yaw(0)
		
	

	'''	
	Function Name:	manual
	Input:         	takes value of key input from keyboard
	Output:        	calls drone control function according to the input
	Logic:         	take the values and choose a function to call using conditional statements
	Example Call:  	manual(data)
	'''


	def manual(self, msg):
		self.key_value = msg.data

		if self.key_value == 114: #R
			try:
				self.autopilot = False
				self.drone.quit()
			except Exception as ex:
				print(ex)

		if self.key_value == 106: #J
			try:
				print("Takeoff")
				self.autopilot = True #Comment this if you want to manually control the drone (for testing purpose only)
				self.drone.takeoff()
			except Exception as ex:
				print(ex)

		if self.key_value == 108: #L
			try:
				self.autopilot = False
				print("Land")
				self.drone.land()
			except Exception as ex:
				print(ex)

		if self.key_value == 119: #W
			try:
				self.autopilot = False
				self.rc_manual(0,100,0,0)
			except Exception as ex:
				print(ex)

		if self.key_value == 115: #S
			try:
				self.autopilot = False
				self.rc_manual(0,-100,0,0)
			except Exception as ex:
				print(ex)

		if self.key_value == 97: #A
			try:
				self.autopilot = False
				self.rc_manual(0,0,-100,0)
			except Exception as ex:
				print(ex)

		if self.key_value == 100: #D
			try:
				self.autopilot = False
				self.rc_manual(0,0,100,0)
			except Exception as ex:
				print(ex)

		if self.key_value == 105: #I
			try:
				self.autopilot = False
				self.rc_manual(100,0,0,0)
			except Exception as ex:
				print(ex)

		if self.key_value == 107: #K
			try:
				self.autopilot = False
				self.rc_manual(-100,0,0,0)
			except Exception as ex:
				print(ex)

		if self.key_value == 109: #N
			try:
				self.autopilot = False
				self.rc_manual(0,0,0,-100)
			except Exception as ex:
				print(ex)

		if self.key_value == 110: #M
			try:
				self.autopilot = False
				self.rc_manual(0,0,0,100)
			except Exception as ex:
				print(ex)

		if self.key_value == 112: #P
			if self.autopilot == True:
				print("AutoPilot OFF")
				self.autopilot = False
			else:
				print("AutoPilot ON")
				self.autopilot = True

		if self.key_value == 49: #1
			if self.activ_pitch == True:
				self.activ_pitch = False
				self.correct_pitch = 0
			else:
				self.activ_pitch = True

		if self.key_value == 50: #2
			if self.activ_roll == True:
				self.activ_roll = False
				self.correct_roll = 0
			else:
				self.activ_roll = True

		if self.key_value == 51: #3
			if self.activ_throt == True:
				self.activ_throt = False
				self.correct_throt = 0
			else:
				self.activ_throt = True





########################   MAIN   #########################




'''
Function Name:	main
Input:			none
Output:			none
Logic:			initializes send_data and starts autocontrol if the drone is not shut down
Example Call:	called automatically
'''

if __name__ == '__main__':
	test = engine()
	test.feed()
	sys.exit(1)



