#!/usr/bin/env python

from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import String
import numpy
import cv2,cv_bridge
import time
import datetime
import os
import tellopy
import rospy
import array
#from imutils.video import VideoStream
from pyzbar import pyzbar
import imutils
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
#from termcolor import colored

class qrcode():

	def __init__(self):
		rospy.init_node('qrcode')
		self.qrfl=1
		rospy.Subscriber("/camera/image_raw", Image, self.process_frame)
		self.image_pub=rospy.Publisher('/shravas/feed', Image, queue_size=10)
		self.ros_bridge = cv_bridge.CvBridge()

	def process_frame(self, msg):
		print("Entered feed.py")
		img = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		#img = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)		
		
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
		x,y = 360,270
		img = cv2.line(img,(x,y-10),(x,y+10),(0,0,255),1)
		img = cv2.line(img,(x-10,y),(x+10,y),(0,0,255),1)
		self.image_pub.publish(self.ros_bridge.cv2_to_imgmsg(image, 'bgr8'))

if __name__ =="__main__":
	tellotrack = qrcode()    
	#test = qrcode()
	while not rospy.is_shutdown():
		rospy.spin()
		sys.exit(1)



		#stream.add_frame(msg.data)
		#frame = cv2.cvtColor(numpy.array(image.to_image()), cv2.COLOR_RGB2BGR)
		#container = av.open(stream)
		#rospy.loginfo('main: opened')
		#for frame in container.decode(video=0):
			#image = cv2.cvtColor(numpy.array(
				#frame.to_image()), cv2.COLOR_RGB2BGR)
			#v2.imshow('Frame', image)
			#cv2.waitKey(1)
		#self.frame = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		#self.image_pub.publish(self.ros_bridge.cv2_to_imgmsg(self.frame, 'bgr8'))


		'''
		#print("Entered FEED.PY")
		#def process_frame(self, frame):
		convert frame to cv2 image and show

				image = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)
				#image =_ self.write_hud(image)
				#image = imutils.resize(image, width=400)
				frame = imutils.resize(frame, width=400)
				
				# find the barcodes in the frame and decode each of the barcodes
				if(self.flag==0):
					barcodes = pyzbar.decode(frame)
					for barcode in barcodes:
						self.flag=1
						barcodeData = barcode.data.decode("utf-8")
						print("\n--------------------------------------------------------------\n")
						#print(colored("Barcode Data:  ",'green'),colored(barcodeData,'green'))
						print("Barcode Data:  "+barcodeData)
						print("\n--------------------------------------------------------------\n")
				# loop over the detected barcode
				#height, width = frame.shape[0], frame.shape[1]
				#x = int(width / 2)
				#y = int(height / 2)
				frame = imutils.resize(frame, width=720)
				#x, y = cv2.GetSize(frame)
				#x = numpy.size(frame, 0) / 2
				#y = numpy.size(frame, 0) / 2
				x,y = 360,270
				frame = cv2.line(frame,(x,y-10),(x,y+10),(0,0,255),1)
				frame = cv2.line(frame,(x-10,y),(x+10,y),(0,0,255),1)

				#self.image_pub.publish(self.ros_bridge.cv2_to_imgmsg(image, 'bgr8'))
				cv2.imshow("Qrcode Scanner", frame)
				key = cv2.waitKey(1) & 0xFF'''

