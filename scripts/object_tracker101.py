#!/usr/bin/env python
# USAGE
# python object_tracker.py --prototxt deploy.prototxt --model res10_300x300_ssd_iter_140000.caffemodel

# import the necessary packages
from pyimagesearch.centroidtracker import CentroidTracker
from imutils.video import VideoStream
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
import numpy as np
import argparse
import imutils
import rospy
import time
import cv2,cv_bridge
import sys


class objtracker():

	def __init__(self):
		self.stat = rospy.Publisher('snr/stat', Int32, queue_size=1)
		#rospy.Subscriber("/camera/image_raw", Image, self.process_frame)
		rospy.Subscriber("/whycon/image_out", Image, self.process_frame)
		self.image_pub=rospy.Publisher('/snr/image_out', Image, queue_size=10)
		self.ros_bridge = cv_bridge.CvBridge()
		self.ct = CentroidTracker()
		(self.H, self.W) = (None, None)
		self.confidence=0.78
		self.statfl=0
		print("[INFO] loading model...")
		self.net = cv2.dnn.readNetFromCaffe("deploy.prototxt", "res10_300x300_ssd_iter_140000.caffemodel")
		#print(type(self.net))
		print("[INFO] starting video stream...")

		time.sleep(2.0)

	def process_frame(self,msg):
		frame = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		frame = imutils.resize(frame, width=300)
		if self.W is None or self.H is None:
			(self.H, self.W) = frame.shape[:2]
		blob = cv2.dnn.blobFromImage(frame, 1.0, (self.W, self.H),
			(104.0, 177.0, 123.0))
		self.net.setInput(blob)
		detections = self.net.forward()
		rects = []
		objects=None
		for i in range(0, detections.shape[2]):
			if detections[0, 0, i, 2] > self.confidence:
				box = detections[0, 0, i, 3:7] * np.array([self.W, self.H, self.W, self.H])
				rects.append(box.astype("int"))
				(startX, startY, endX, endY) = box.astype("int")
				cv2.rectangle(frame, (startX, startY), (endX, endY),
					(0, 255, 0), 2)
				objects = self.ct.update(rects)
				if(bool(objects)):
					self.stat.publish(1)
		if(objects==None):
			self.stat.publish(0)
		self.image_pub.publish(self.ros_bridge.cv2_to_imgmsg(frame, 'bgr8'))

def main():
	rospy.init_node('snr')
	test = objtracker()
	while not rospy.is_shutdown():
		rospy.spin()
		sys.exit(1)

if __name__ == '__main__':
	main()

