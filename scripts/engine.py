#!/usr/bin/env python2
import rospy
from std_msgs.msg import Empty, UInt8, Bool
from std_msgs.msg import UInt8
from pid_tune.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16,Int32, Float64, String
from sensor_msgs.msg import Image, CompressedImage, Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
from cv_bridge import CvBridgeError, CvBridgeError
import cv2,cv_bridge

import math
import numpy as np
import array as arr
import threading
import time
from tellopy._internal import tello
from tellopy._internal import error
from tellopy._internal import protocol
from tellopy._internal import logger

import datetime
import os
from pyzbar import pyzbar
import imutils
import sys

######################################BEGIN#######################################

#### Additional module imports (Jordy) ####

# Publish camera info to rectify camera images
from sensor_msgs.msg import CameraInfo

# To load camera calibration from '.yaml' format
import camera_info_manager as cim

# Access to all protocol constant variables
from tellopy._internal.protocol import *

# Add 'EVENT_VIDEO_FRAME_H264' to collect h264 images
from tellopy._internal import event

########################################END#######################################

class RospyLogger(logger.Logger):
	def __init__(self, header=''):
		super(RospyLogger, self).__init__(header)

	def error(self, s):
		if self.log_level < logger.LOG_ERROR:
			return
		rospy.logerr(s)

	def warn(self, s):
		if self.log_level < logger.LOG_WARN:
			return
		rospy.logwarn(s)

	def info(self, s):
		if self.log_level < logger.LOG_INFO:
			return
		rospy.loginfo(s)

	def debug(self, s):
		if self.log_level < logger.LOG_DEBUG:
			return
		rospy.logdebug(s)


def notify_cmd_success(cmd, success):
	if success:
		rospy.loginfo('%s command executed' % cmd)
	else:
		rospy.logwarn('%s command failed' % cmd)

#class TelloNode(tello.Tello):
class engine(tello.Tello):
#######################################BEGIN#######################################    

	## Add event variable(s) to leave 'TelloPy' package untouched (Jordy) ##
	EVENT_VIDEO_FRAME_H264 = event.Event('video frame h264')

#########################################END#######################################    
	def __init__(self):
		self.local_cmd_client_port = int(
			rospy.get_param('~local_cmd_client_port', 8890))
		self.local_vid_server_port = int(
			rospy.get_param('~local_vid_server_port', 6038))
		self.tello_ip = rospy.get_param('~tello_ip', '192.168.10.1')
		self.tello_cmd_server_port = int(
			rospy.get_param('~tello_cmd_server_port', 8889))
		self.connect_timeout_sec = float(
			rospy.get_param('~connect_timeout_sec', 10.0))
		self.stream_h264_video = bool(
			rospy.get_param('~stream_h264_video', False))
		self.bridge = CvBridgeError()
		self.frame_thread = None

		# Connect to drone
		self.log = RospyLogger('Tello')

#######################################BEGIN#######################################    
		# fast_mode attribute before inheritance from TelloPy object (see override __send_stick_command)
		self.fast_mode = False        
#########################################END#######################################    

		super(engine, self).__init__(port=9000)
		rospy.loginfo('Connecting to drone @ %s:%d' % self.tello_addr)
		self.connect()
		try:
			self.wait_for_connection(timeout=self.connect_timeout_sec)
		except error.TelloError as err:
			rospy.logerr(str(err))
			rospy.signal_shutdown(str(err))
			self.quit()
			return
		rospy.loginfo('Connected to drone')
		rospy.on_shutdown(self.cb_shutdown)

		# Setup dynamic reconfigure
		self.cfg = None
		self.srv_dyncfg = Server(TelloConfig, self.cb_dyncfg)

		# Setup topics and services
		# NOTE: ROS interface deliberately made to resemble bebop_autonomy
		#self.pub_status = rospy.Publisher('status', TelloStatus, queue_size=1, latch=True)
		if self.stream_h264_video:
			self.pub_image_h264 = rospy.Publisher(
				'image_raw/h264', CompressedImage, queue_size=10)
		else:
			self.pub_image_raw = rospy.Publisher(
				'camera/image_raw', Image, queue_size=10)

		self.sub_takeoff = rospy.Subscriber('takeoff', Empty, self.cb_takeoff)
		self.sub_manual_takeoff = rospy.Subscriber('manual_takeoff', Empty, self.cb_manual_takeoff)
		self.sub_throw_takeoff = rospy.Subscriber(
			'throw_takeoff', Empty, self.cb_throw_takeoff)
		self.sub_land = rospy.Subscriber('land', Empty, self.cb_land)
		self.sub_palm_land = rospy.Subscriber(
			'palm_land', Empty, self.cb_palm_land)
		self.sub_flattrim = rospy.Subscriber(
			'flattrim', Empty, self.cb_flattrim)
		self.sub_flip = rospy.Subscriber('flip', UInt8, self.cb_flip)
		self.sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, self.cb_cmd_vel)
		self.sub_fast_mode = rospy.Subscriber(
			'fast_mode', Empty, self.cb_fast_mode)

		self.subscribe(self.EVENT_FLIGHT_DATA, self.cb_status_log)

#########################################BEGIN#####################################

		# Reconstruction H264 video frames        
		self.prev_seq_id = None
		self.seq_block_count = 0        
		
		# Height from EVENT_FLIGHT_DATA more accurate than MVO (monocular visual odometry)
		self.height = 0

		#EVENT_LOG_DATA from 'TelloPy' package
		self.pub_odom = rospy.Publisher('odom', Odometry, queue_size=1, latch=True)
		self.pub_imu = rospy.Publisher('imu', Imu, queue_size=1, latch=True)

		self.subscribe(self.EVENT_LOG_DATA, self.cb_data_log)

		self.sub_zoom = rospy.Subscriber('video_mode', Empty, self.cb_video_mode, queue_size=1)

		if self.stream_h264_video:
			self.start_video()
			self.subscribe(self.EVENT_VIDEO_DATA, self.cb_video_data)            
			self.subscribe(self.EVENT_VIDEO_FRAME_H264, self.cb_h264_frame)
		else:
			#print("we're fucked")
			self.frame_thread = threading.Thread(target=self.framegrabber_loop)
			self.frame_thread.start()

		calib_path = rospy.get_param('~camera_calibration', '') 
		self.caminfo = cim.loadCalibrationFile(calib_path, 'camera_front')
		self.caminfo.header.frame_id = rospy.get_param('~camera_frame', rospy.get_namespace() + 'camera_front')
		self.pub_caminfo = rospy.Publisher('camera/camera_info', CameraInfo, queue_size=1, latch=True)                
		self.pub_caminfo.publish(self.caminfo)

		self.sub_emergency = rospy.Subscriber('emergency', Empty, self.cb_emergency, queue_size=1)


		#self.stats = rospy.Publisher('/stats', String, queue_size=1)
		#self.feed = rospy.Publisher('/dronefeed',Image,queue_size=1)
		self.xcord = rospy.Publisher('/wp/x',Float64,queue_size=1)
		self.ycord = rospy.Publisher('/wp/y',Float64,queue_size=1)
		self.zcord = rospy.Publisher('/wp/z',Float64,queue_size=1)
		
		rospy.Subscriber('/input_key', Int16, self.manual)
		#rospy.Subscriber('/pilot/x', Float64, self.xcoord)
		#rospy.Subscriber('/pilot/y', Float64, self.ycoord)
		#rospy.Subscriber('/pilot/z', Float64, self.zcoord)
		#sys.sleep(2)
		
		#rospy.Subscriber('/whycon/poses', PoseArray, self.feed)
		
		rospy.Subscriber('/pid_tuning_altitude', PidTune, self.set_pid_alt)
		rospy.Subscriber('/pid_tuning_roll', PidTune, self.set_pid_roll)
		rospy.Subscriber('/pid_tuning_pitch', PidTune, self.set_pid_pitch)
		rospy.Subscriber('whycon/poses', PoseArray, self.get_pose)

		self.p = rospy.Publisher('power/pitch', Float64, queue_size=1)
		self.r = rospy.Publisher('power/roll', Float64, queue_size=1)
		self.t = rospy.Publisher('power/throttle', Float64, queue_size=1)


		#Holds the coordinates to be achieved by the drone
		self.wp_x=0.0
		self.wp_y=0.0
		self.wp_z=21.0

		#self.send_tello.publish(self.drone)
		#self.actflag=0
		self.qrfl=0

		#Holds the current coordinates of the drone (recieved from whycon/poses)
		self.drone_x = 0.0
		self.drone_y = 0.0
		self.drone_z = 0.0

		
		#self.wp_pkg = 1 #Compartment no. containing the package for given wp

		#Holds the z axis coordinate of the drone stage (beehive)
		self.ground=0.0

		#Variable to flag when the PID should take control of the drone
		self.autopilot  = False

		#PID constants for Roll
		self.kp_roll = 17.0
		self.ki_roll = 0.01
		self.kd_roll = 2.0

		#PID constants for Pitch
		self.kp_pitch = 17.0
		self.ki_pitch = 0.02
		self.kd_pitch = 4.0

		#PID constants for Throttle
		self.kp_throt = 17.0
		self.ki_throt = 0.02
		self.kd_throt = 1.0

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
		
###########################################END#####################################

		rospy.loginfo('Tello driver node ready')        

###########################################BEGIN###################################

## Add 'Tello' compositions, leave 'TelloPy' package untouched (Jordy) ##        

	def autocontrol(self):
		if self.autopilot:
			self.ground=self.drone_z
			while self.autopilot:
				#print("**********************inside PID*************************")
				self.calc_pid()
				pitch_value = int(0 - self.correct_pitch)
				pitch_value = self.limit(pitch_value, 70, -70)
				
																
				roll_value = int(0 + self.correct_roll)
				roll_value = self.limit(roll_value, 70, -70)
				
																
				throt_value = int(0 - self.correct_throt)
				throt_value = self.limit(throt_value, 70, -70)
				yaw_value = 0.0

				self.p.publish(pitch_value)
				self.r.publish(roll_value)
				self.t.publish(throt_value)

				#Pushes command to drone                                        
				self.rc(throt_value,pitch_value,roll_value,yaw_value)
				



	#Function Name: check_delta
	#Input:         none
	#Output:        incrementing counter value
	#Logic:         if the drone is within error box around the required coordinates, increment counter; else reset counter
	#Example Call:  check_delta()
	def check_delta(self,error_xy,error_z):
		if((self.drone_x<(self.wp_x+error_xy)) & (self.drone_x>(self.wp_x-error_xy)) & (self.drone_y<(self.wp_y+error_xy)) & (self.drone_y>(self.wp_y-error_xy)) & (self.drone_z>(self.wp_z-error_z)) & (self.drone_z<(self.wp_z+error_z))):            
			self.counter+=1
		else:
			self.counter=0


	#Function Name:limit
	#Input:         value, upper, lower
	#Output:        value in limits
	#Logic:         simple if else logic to determine if value allowed.
	#Example Call:  limit(input, max, min)
	def limit(self, input_value, max_value, min_value):
		#Use this function to limit the maximum and minimum values you send to your drone
		#print("Entered Limit")
		if input_value > max_value:
			return max_value
		if input_value < min_value:
			return min_value
		else:
			return input_value

			

	#Function Name:calc_pid
	#Input:         none
	#Output:        none
	#Logic:         calls the pid functions after a specific time duration
	#Example Call:  calc_pid()
	def calc_pid(self):
		#print("Entered cal_pid")
		self.seconds = time.time()
		current_time = self.seconds - self.last_time
		if(current_time >= self.loop_time):
			#self.stats.publish("Calling PIDs")
			
			if self.activ_roll:   
				self.pid_roll()
			if self.activ_pitch:
				self.pid_pitch()
			if self.activ_throt:
				self.pid_throt()
			self.last_time = self.seconds
			#print("in calcpid")

		

	#Function Name:rc
	#Input:         throttle, roll, pitch, yaw inputs in range -100 to 100
	#Output:        send the corresponding commands to drone
	#Logic:         uses drone functions to send values to tello lib
	#Example Call:  pid_yaw()
	def rc(self, throttle, pitch, roll, yaw):
		self.set_throttle(throttle/100.0)
		self.set_pitch(pitch/100.0)
		self.set_roll(roll/100.0)
		self.set_yaw(yaw/100.0)
		#print("Inside RC Sent commands t drone")
		#self.stats.publish("RC autopilot commands sent")



	#Function Name:pid_roll
	#Output:        set value of correct_roll
	#Logic:         uses PID algorithm to estimate the correction to be made in rcRoll to achieve required Y coordinate
	#Example Call:  pid_roll()
	def pid_roll(self):
		#print("Entered pid_roll")
		
		self.err_roll=self.wp_x-self.drone_x
		self.iterm_roll+=self.err_roll*self.loop_time
		dErr_roll=(self.err_roll-self.preverr_roll)/self.loop_time
		self.correct_roll=((self.kp_roll*self.err_roll)+(self.ki_roll*self.iterm_roll)+(self.kd_roll*dErr_roll))
		self.preverr_roll=self.err_roll
		#print("in pidroll")
		#print (self.correct_roll)
		#self.stats.publish("Roll PID : "+str(self.correct_roll)) 

	#Function Name:pid_pitch
	#Input:         uses drone_y
	#Output:        set value of correct_pitch
	#Logic:         uses PID algorithm to estimate the correction to be made in rcPitch to achieve required X coordinate
	#Example Call:  pid_pitch() 
	def pid_pitch(self):
		#print("Entered pid_pitch")
		
		self.err_pitch=self.wp_y-self.drone_y
		self.iterm_pitch+=self.err_pitch*self.loop_time
		dErr_pitch=(self.err_pitch-self.preverr_pitch)/self.loop_time
		self.correct_pitch=((self.kp_pitch*self.err_pitch)+(self.ki_pitch*self.iterm_pitch)+(self.kd_pitch*dErr_pitch))
		self.preverr_pitch=self.err_pitch
		#print("in pidpitch")
		#print (self.correct_pitch)
		#self.stats.publish("Pitch PID : "+str(self.correct_pitch))

	#Function Name:pid_throt
	#Input:         uses drone_z
	#Output:        set the value of correct_throt
	#Logic:         uses PID algorithm to estimate the correction to be made in rcThrot to achieve required Z coordinate
	#Example Call:  pid_throt()
	def pid_throt(self):
		#print("Entered pid_throt")
		self.err_throt=self.wp_z-self.drone_z
		self.iterm_throt+=self.err_throt*self.loop_time
		dErr_throt=(self.err_throt-self.preverr_throt)/self.loop_time
		self.correct_throt=((self.kp_throt*self.err_throt)+(self.ki_throt*self.iterm_throt)+(self.kd_throt*dErr_throt))
		self.preverr_throt=self.err_throt
		#print (self.correct_throt)
		#print("in pidthrot")
		#self.stats.publish("Throttle PID : "+str(self.correct_throt))



####################################################################################################################################################################################################### 

#SUBSCRIBER FUNCTIONS



	#Function Name:get_pose
	#Input:         takes poseArray message from whycon/poses
	#Output:        sets the value of drone_x, drone_y, drone_z
	#Logic:         subscribes to whycon/poses to get the coordinates of the whycon marker placed on the drone
	#Example Call:  get_pose(data)
	def get_pose(self, pose):
		self.drone_x = pose.poses[0].position.x
		self.drone_y = pose.poses[0].position.y
		self.drone_z = pose.poses[0].position.z
		#self.stats.publish("Drone coordinates fetched")

		self.xcord.publish(self.wp_x)
		self.ycord.publish(self.wp_y)
		self.zcord.publish(self.wp_z)

	
		#Function Name:set_pid_alt
		#Input:         the values of kp,ki and kd
		#Output:        sets the values for kp_throt,ki_throt and kd_throt
		#Logic:         copy the values from pid_val to the object's variables
		#Example Call:  set_pid_alt(data)
	def set_pid_alt(self,pid_val):
		self.kp_throt = float(pid_val.Kp)/10
		self.ki_throt = float(pid_val.Ki)/10 
		self.kd_throt = float(pid_val.Kd)/10
		#print(self.kp_throt," / ",self.ki_throt," / ",self.kd_throt)
	

		#Function Name:set_pid_roll
		#Input:     the values of kp,ki and kd
		#Output:    sets the values for kp_roll,ki_roll and kd_roll
		#Logic:     copy the values from pid_val to the object's variables
		#Example Call:  set_pid_roll(data)
	def set_pid_roll(self,pid_val):
		self.kp_roll = float(pid_val.Kp)/10
		self.ki_roll = float(pid_val.Ki)/10
		self.kd_roll = float(pid_val.Kd)/10 
		#print self.kp_roll,self.ki_roll,self.kd_roll
		
		#Function Name:set_pid_pitch
		#Input:     the values of kp,ki and kd
		#Output:    sets the values for kp_pitch,ki_pitch and kd_pitch
		#Logic:     copy the values from pid_val to the object's variables
		#Example Call:  set_pid_pitch(data)
	def set_pid_pitch(self,pid_val):
		self.kp_pitch = float(pid_val.Kp)/10
		self.ki_pitch = float(pid_val.Ki)/10
		self.kd_pitch = float(pid_val.Kd)/10
		#print self.kp_pitch,self.ki_pitch,self.kd_pitch

	
	'''def feed(self):
		print("Starting feed")
		for packet in self.container.demux((self.vid_stream,)):
			for frame in packet.decode():
				#image=self.process_frame(frame)


				image = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)
				#image =_ self.write_hud(image)
				#image = imutils.resize(image, width=400)
				image = imutils.resize(image, width=400)
				
				# find the barcodes in the frame and decode each of the barcodes
				#if(self.flag==0):
				barcodes = pyzbar.decode(image)
				for barcode in barcodes:
					barcodeData = barcode.data.decode("utf-8")
					print("\n--------------------------------------------------------------\n")
					#print(colored("Barcode Data:  ",'green'),colored(barcodeData,'green'))
					print("Barcode Data:  "+barcodeData)
					print("\n--------------------------------------------------------------\n")
				# loop over the detected barcode
				#height, width = frame.shape[0], frame.shape[1]
				#x = int(width / 2)
				#y = int(height / 2)
				image = imutils.resize(image, width=720)
				#x, y = cv2.GetSize(frame)
				#x = numpy.size(frame, 0) / 2
				#y = numpy.size(frame, 0) / 2
				x,y = 360,270
				image = cv2.line(image,(x,y-10),(x,y+10),(0,0,255),1)
				image = cv2.line(image,(x-10,y),(x+10,y),(0,0,255),1)

				#self.image_pub.publish(self.ros_bridge.cv2_to_imgmsg(image, 'bgr8'))
				cv2.imshow("Qrcode Scanner", image)
				key = cv2.waitKey(1) & 0xFF              '''

	
####################################################################################################################################################################################################### 
#MANUAL CONTROL (To take ccontrol of the drone if it flies out of range)

	#Function Name:rc
	#Input:         throttle, roll, pitch, yaw inputs in range -100 to 100
	#Output:        send the corresponding commands to drone
	#Logic:         uses drone functions to send values to tello lib
	#Example Call:  pid_yaw()
	def rc_manual(self, throttle, pitch, roll, yaw):
		self.set_throttle(throttle/100.0)
		self.set_pitch(pitch/100.0)
		self.set_roll(roll/100.0)
		self.set_yaw(yaw/100.0)
		rospy.sleep(0.1)
		self.set_throttle(0)
		self.set_pitch(0)
		self.set_roll(0)
		self.set_yaw(0)
		#self.stats.publish("RC manual commands sent")

		
	#Function Name:emergencycontrol
	#Input:         takes value of key input from keyboard
	#Output:        calls drone control function according to the input
	#Logic:         take the values and choose a function to call using conditional statements
	#Example Call:  emergencycontrol(data)
	def manual(self, msg):
		self.key_value = msg.data

		if self.key_value == 114: #R
			try:
				self.autopilot = False
				self.quit()
			except Exception as ex:
				print(ex)

		if self.key_value == 106: #J
			try:
				print("Takeoff")
				self.autopilot = True #Comment this if you want to manually control the drone (for testing purpose only)
				
				self.takeoff()
			except Exception as ex:
				print(ex)

		if self.key_value == 113: #Q
			try:
				print("Throw Takeoff")
				self.autopilot = False #Comment this if you want to manually control the drone (for testing purpose only)
				self.cb_throw_takeoff()
			except Exception as ex:
				print(ex)

		if self.key_value == 108: #L
			try:
				self.autopilot = False
				print('Land')
				self.land()
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
				self.autopilot = False
			else:
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

	def set_fast_mode(self, enabled):
		self.fast_mode = enabled
		
	def reset_cmd_vel(self):
		self.left_x = 0.
		self.left_y = 0.
		self.right_x = 0.
		self.right_y = 0.
		self.fast_mode = False

	# scaling for velocity command
	def __scale_vel_cmd(self, cmd_val):
		return self.vel_cmd_scale * cmd_val
	
	# Override TelloPy '__send_stick_command' to add 'fast_mode' functionality
	def _Tello__send_stick_command(self):
		pkt = Packet(STICK_CMD, 0x60)

		axis1 = int(1024 + 660.0 * self.right_x) & 0x7ff
		axis2 = int(1024 + 660.0 * self.right_y) & 0x7ff
		axis3 = int(1024 + 660.0 * self.left_y) & 0x7ff
		axis4 = int(1024 + 660.0 * self.left_x) & 0x7ff
		axis5 = int(self.fast_mode) & 0x01
		self.log.debug("stick command: fast=%d yaw=%4d vrt=%4d pit=%4d rol=%4d" %
					   (axis5, axis4, axis3, axis2, axis1))

		'''
		11 bits (-1024 ~ +1023) x 4 axis = 44 bits
		fast_mode takes 1 bit
		44+1 bits will be packed in to 6 bytes (48 bits)
		 axis5      axis4      axis3      axis2      axis1
			 |          |          |          |          |
				 4         3         2         1         0
		98765432109876543210987654321098765432109876543210
		 |       |       |       |       |       |       |
			 byte5   byte4   byte3   byte2   byte1   byte0
		'''
		packed = axis1 | (axis2 << 11) | (
			axis3 << 22) | (axis4 << 33) | (axis5 << 44)
		packed_bytes = struct.pack('<Q', packed)
		pkt.add_byte(byte(packed_bytes[0]))
		pkt.add_byte(byte(packed_bytes[1]))
		pkt.add_byte(byte(packed_bytes[2]))
		pkt.add_byte(byte(packed_bytes[3]))
		pkt.add_byte(byte(packed_bytes[4]))
		pkt.add_byte(byte(packed_bytes[5]))
		pkt.add_time()
		pkt.fixup()
		self.log.debug("stick command: %s" %
					   byte(pkt.get_buffer()))
		return self.send_packet(pkt)        

	def manual_takeoff(self):
		# Hold max 'yaw' and min 'pitch', 'roll', 'throttle' for several seconds
		self.set_pitch(-1)
		self.set_roll(-1)
		self.set_yaw(1)
		self.set_throttle(-1)
		self.fast_mode = False

		return self._Tello__send_stick_command()

	def cb_video_data(self, event, sender, data, **args):
		now = time.time()
		
		# parse packet
		seq_id = byte(data[0])
		sub_id = byte(data[1])
		packet = data[2:]
		self.sub_last = False
		if sub_id >= 128: # MSB asserted
			sub_id -= 128
			self.sub_last = True
		
		#associate packet to (new) frame
		if self.prev_seq_id is None or self.prev_seq_id != seq_id:
			# detect wrap-arounds
			if self.prev_seq_id is not None and self.prev_seq_id > seq_id:
				self.seq_block_count += 1
			self.frame_pkts = [None]*128 # since sub_id uses 7 bits
			self.frame_t = now
			self.prev_seq_id = seq_id
		self.frame_pkts[sub_id] = packet
		
		# publish frame if completed
		if self.sub_last and all(self.frame_pkts[:sub_id+1]):
			if isinstance(self.frame_pkts[sub_id], str):
				frame = ''.join(self.frame_pkts[:sub_id+1])
			else:
				frame = b''.join(self.frame_pkts[:sub_id+1])
			self._Tello__publish(event=self.EVENT_VIDEO_FRAME_H264,
						   data=(frame, self.seq_block_count*256+seq_id, self.frame_t))        

	def send_req_video_sps_pps(self):
		"""Manually request drone to send an I-frame info (SPS/PPS) for video stream."""
		pkt = Packet(VIDEO_START_CMD, 0x60)
		pkt.fixup()
		return self.send_packet(pkt)

	def set_video_req_sps_hz(self, hz):
		"""Internally sends a SPS/PPS request at desired rate; <0: disable."""
		if hz < 0:
			hz = 0.
		self.video_req_sps_hz = hz
		
	# emergency command
	def emergency(self):
		""" Stop all motors """
		self.log.info('emergency (cmd=% seq=0x%04x)' % (EMERGENCY_CMD, self.pkt_seq_num))
		pkt = Packet(EMERGENCY_CMD)
		return self.send_packet(pkt)
		
	def flip(self, cmd):
		""" tell drone to perform a flip in directions [0,8] """
		self.log.info('flip (cmd=0x%02x seq=0x%04x)' % (FLIP_CMD, self.pkt_seq_num))
		pkt = Packet(FLIP_CMD, 0x70)
		pkt.add_byte(cmd)
		pkt.fixup()
		return self.send_packet(pkt)

##### Additions to 'tello_driver_node' (Jordy) #####
	
	def cb_video_mode(self, msg):
		if not self.zoom:
			self.set_video_mode(True)
		else:
			self.set_video_mode(False)

	def cb_emergency(self, msg):
		success = self.emergency()
		notify_cmd_success('Emergency', success)

##### Modifications to 'tello_driver_node' (Jordy) #####

	def cb_dyncfg(self, config, level):
		update_all = False
		req_sps_pps = False
		if self.cfg is None:
			self.cfg = config
			update_all = True

		if update_all or self.cfg.altitude_limit != config.altitude_limit:
			self.set_alt_limit(config.altitude_limit)
		if update_all or self.cfg.attitude_limit != config.attitude_limit:
			self.set_att_limit(config.attitude_limit)
		if update_all or self.cfg.low_bat_threshold != config.low_bat_threshold:
			self.set_low_bat_threshold(config.low_bat_threshold)
		if update_all or self.cfg.fixed_video_rate != config.fixed_video_rate:
			self.set_video_encoder_rate(config.fixed_video_rate)
			req_sps_pps = True
		if update_all or self.cfg.video_req_sps_hz != config.video_req_sps_hz:
			self.set_video_req_sps_hz(config.video_req_sps_hz)
			req_sps_pps = True
		if update_all or self.cfg.vel_cmd_scale != config.vel_cmd_scale:
			self.vel_cmd_scale = config.vel_cmd_scale
		if req_sps_pps:
			self.send_req_video_sps_pps()

		self.cfg = config
		return self.cfg

	def cb_status_log(self, event, sender, data, **args):
		speed_horizontal_mps = math.sqrt(
			data.north_speed*data.north_speed+data.east_speed*data.east_speed)/10.

		# TODO: verify outdoors: anecdotally, observed that:
		# data.east_speed points to South
		# data.north_speed points to East
		self.height = data.height/10.
		msg = TelloStatus(
			height_m=data.height/10.,
			speed_northing_mps=-data.east_speed/10.,
			speed_easting_mps=data.north_speed/10.,
			speed_horizontal_mps=speed_horizontal_mps,
			speed_vertical_mps=-data.ground_speed/10.,
			flight_time_sec=data.fly_time/10.,
			imu_state=data.imu_state,
			pressure_state=data.pressure_state,
			down_visual_state=data.down_visual_state,
			power_state=data.power_state,
			battery_state=data.battery_state,
			gravity_state=data.gravity_state,
			wind_state=data.wind_state,
			imu_calibration_state=data.imu_calibration_state,
			battery_percentage=data.battery_percentage,
			drone_fly_time_left_sec=data.drone_fly_time_left/10.,
			drone_battery_left_sec=data.drone_battery_left/10.,
			is_flying=data.em_sky,
			is_on_ground=data.em_ground,
			is_em_open=data.em_open,
			is_drone_hover=data.drone_hover,
			is_outage_recording=data.outage_recording,
			is_battery_low=data.battery_low,
			is_battery_lower=data.battery_lower,
			is_factory_mode=data.factory_mode,
			fly_mode=data.fly_mode,
			throw_takeoff_timer_sec=data.throw_fly_timer/10.,
			camera_state=data.camera_state,
			electrical_machinery_state=data.electrical_machinery_state,
			front_in=data.front_in,
			front_out=data.front_out,
			front_lsc=data.front_lsc,
			temperature_height_m=data.temperature_height/10.,
			cmd_roll_ratio=self.right_x,
			cmd_pitch_ratio=self.right_y,
			cmd_yaw_ratio=self.left_x,
			cmd_vspeed_ratio=self.left_y,
			cmd_fast_mode=self.fast_mode,
		)
		self.pub_status.publish(msg)

	def cb_data_log(self, event, sender, data, **args):
		time_cb = rospy.Time.now()

		odom_msg = Odometry()
		odom_msg.child_frame_id = rospy.get_namespace() + 'base_link'
		odom_msg.header.stamp = time_cb
		odom_msg.header.frame_id = rospy.get_namespace() + 'local_origin'        

		# Height from MVO received as negative distance to floor
		odom_msg.pose.pose.position.z = -data.mvo.pos_z #self.height #-data.mvo.pos_z
		odom_msg.pose.pose.position.x = data.mvo.pos_x
		odom_msg.pose.pose.position.y = data.mvo.pos_y
		odom_msg.pose.pose.orientation.w = data.imu.q0
		odom_msg.pose.pose.orientation.x = data.imu.q1
		odom_msg.pose.pose.orientation.y = data.imu.q2
		odom_msg.pose.pose.orientation.z = data.imu.q3
		# Linear speeds from MVO received in dm/sec
		odom_msg.twist.twist.linear.x = data.mvo.vel_y/10
		odom_msg.twist.twist.linear.y = data.mvo.vel_x/10
		odom_msg.twist.twist.linear.z = -data.mvo.vel_z/10
		odom_msg.twist.twist.angular.x = data.imu.gyro_x
		odom_msg.twist.twist.angular.y = data.imu.gyro_y
		odom_msg.twist.twist.angular.z = data.imu.gyro_z
				
		self.pub_odom.publish(odom_msg)
		
		imu_msg = Imu()
		imu_msg.header.stamp = time_cb
		imu_msg.header.frame_id = rospy.get_namespace() + 'base_link'
		
		imu_msg.orientation.w = data.imu.q0
		imu_msg.orientation.x = data.imu.q1
		imu_msg.orientation.y = data.imu.q2
		imu_msg.orientation.z = data.imu.q3        
		imu_msg.angular_velocity.x = data.imu.gyro_x
		imu_msg.angular_velocity.y = data.imu.gyro_y
		imu_msg.angular_velocity.z = data.imu.gyro_z
		imu_msg.linear_acceleration.x = data.imu.acc_x
		imu_msg.linear_acceleration.y = data.imu.acc_y
		imu_msg.linear_acceleration.z = data.imu.acc_z
		
		self.pub_imu.publish(imu_msg)

	def cb_cmd_vel(self, msg):
		self.set_pitch( self.__scale_vel_cmd(msg.linear.y) )
		self.set_roll( self.__scale_vel_cmd(msg.linear.x) )
		self.set_yaw( self.__scale_vel_cmd(msg.angular.z) )
		self.set_throttle( self.__scale_vel_cmd(msg.linear.z) )

	def cb_flip(self, msg):
		if msg.data < 0 or msg.data > 7: # flip integers between [0,7]
			rospy.logwarn('Invalid flip direction: %d' % msg.data)
			return
		success = self.flip(msg.data)
		notify_cmd_success('Flip %d' % msg.data, success)
		
##########################################END#########################################

	def cb_shutdown(self):
		self.quit()
		if self.frame_thread is not None:
			self.frame_thread.join()

	def cb_h264_frame(self, event, sender, data, **args):
		frame, seq_id, frame_secs = data
		pkt_msg = CompressedImage()
		pkt_msg.header.seq = seq_id
		pkt_msg.header.frame_id = self.caminfo.header.frame_id
		pkt_msg.header.stamp = rospy.Time.from_sec(frame_secs)


		####### IP Here   ######### 


		pkt_msg.data = frame
		self.pub_image_h264.publish(pkt_msg)

		self.caminfo.header.seq = seq_id
		self.caminfo.header.stamp = rospy.Time.from_sec(frame_secs)
		self.pub_caminfo.publish(self.caminfo)      

	def framegrabber_loop(self):
		import av #Import here as 'hack' to prevent troublesome install of PyAV when not used
		# Repeatedly try to connect
		vs = self.get_video_stream()
		while self.state != self.STATE_QUIT:
			try:
				container = av.open(vs)
				break
			except BaseException as err:
				rospy.logerr('fgrab: pyav stream failed - %s' % str(err))
				time.sleep(1.0)

		# Once connected, process frames till drone/stream closes
		while self.state != self.STATE_QUIT:
			try:
				# vs blocks, dies on self.stop
				for frame in container.decode(video=0):

					####### IP Here   #########
					'''
					#img = np.array(frame.to_image())

					img = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)
					
					x,y = 360,270
					img = cv2.line(img,(x,y-10),(x,y+10),(0,0,255),1)
					img = cv2.line(img,(x-10,y),(x+10,y),(0,0,255),1)
					img = imutils.resize(img, width=400)
					if(self.qrfl==0):
						barcodes = pyzbar.decode(img)
						for barcode in barcodes:
							barcodeData = barcode.data.decode("utf-8")
							print("\n--------------------------------------------------------------\n")
							#print(colored("Barcode Data:  ",'green'),colored(barcodeData,'green'))
							print("Barcode Data:  "+barcodeData)
							print("\n--------------------------------------------------------------\n")
							self.qrfl=1
					img = imutils.resize(img, width=720)


					#cv2.imshow("Qrcode Scanner", img)
					#key = cv2.waitKey(1) & 0xFF
					'''

					try:
						img_msg = self.bridge.cv2_to_imgmsg(img, 'rgb8')
						img_msg.header.frame_id = self.caminfo.header.frame_id
					except CvBridgeError as err:
						rospy.logerr('fgrab: cv bridge failed - %s' % str(err))
						continue
					self.pub_image_raw.publish(img_msg)
					self.pub_caminfo.publish(self.caminfo)                    
				break
			except BaseException as err:
				rospy.logerr('fgrab: pyav decoder failed - %s' % str(err)) 


	def cb_takeoff(self, msg):
		success = self.takeoff()
		notify_cmd_success('Takeoff', success)
	
	def cb_manual_takeoff(self, msg):
		success = self.manual_takeoff()
		notify_cmd_success('Manual takeoff', success)

	def cb_throw_takeoff(self, msg):
		success = self.throw_and_go()
		if success:
			rospy.loginfo('Drone set to auto-takeoff when thrown')
		else:
			rospy.logwarn('ThrowTakeoff command failed')

	def cb_land(self, msg):
		success = self.land()
		notify_cmd_success('Land', success)

	def cb_palm_land(self, msg):
		success = self.palm_land()
		notify_cmd_success('PalmLand', success)

	def cb_flattrim(self, msg):
		success = self.flattrim()
		notify_cmd_success('FlatTrim', success)

	def cb_fast_mode(self, msg):
		if self.fast_mode:
			self.set_fast_mode(False)
		elif not self.fast_mode:
			self.set_fast_mode(True)


def main():
	rospy.init_node('engine')
	robot = engine()
	while not rospy.is_shutdown():
		#print("Ph0A: Entered While Loop")
		while True:
			robot.autocontrol()
		rospy.spin()
		sys.exit(1)

if __name__ == '__main__':
	main()
