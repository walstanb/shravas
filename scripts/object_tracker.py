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
		rospy.Subscriber("/camera/image_raw", Image, self.process_frame)
		#rospy.Subscriber("/whycon/image_out", Image, self.process_frame)
		self.image_pub=rospy.Publisher('/snr/image_out', Image, queue_size=10)
		self.ros_bridge = cv_bridge.CvBridge()
		
	
		# initialize our centroid tracker and frame dimensions
		self.ct = CentroidTracker()
		(self.H, self.W) = (None, None)
		self.confidence=0.75
		self.statfl=0

		# load our serialized model from disk
		print("[INFO] loading model...")
		self.net = cv2.dnn.readNetFromCaffe("/home/walst/catkin_ws/src/shravas/src/deploy.prototxt", "/home/walst/catkin_ws/src/shravas/src/res10_300x300_ssd_iter_140000.caffemodel")
		# initialize the video stream and allow the camera sensor to warmup
		print("[INFO] starting video stream...")
		#vs = VideoStream(src=0).start()

		time.sleep(2.0)
		# loop over the frames from the video stream
		
		#while True:
	def process_frame(self,msg):
		# read the next frame from the video stream and resize it
		frame = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		#frame = vs.read()
		frame = imutils.resize(frame, width=400)

		# if the frame dimensions are None, grab them
		if self.W is None or self.H is None:
			(self.H, self.W) = frame.shape[:2]

		# construct a blob from the frame, pass it through the network,
		# obtain our output predictions, and initialize the list of
		# bounding box rectangles
		blob = cv2.dnn.blobFromImage(frame, 1.0, (self.W, self.H),
			(104.0, 177.0, 123.0))
		self.net.setInput(blob)
		detections = self.net.forward()
		rects = []

		# loop over the detections
		for i in range(0, detections.shape[2]):
			# filter out weak detections by ensuring the predicted
			# probability is greater than a minimum threshold
			if detections[0, 0, i, 2] > self.confidence:
				# compute the (x, y)-coordinates of the bounding box for
				# the object, then update the bounding box rectangles list
				box = detections[0, 0, i, 3:7] * np.array([self.W, self.H, self.W, self.H])
				rects.append(box.astype("int"))

				# draw a bounding box surrounding the object so we can
				# visualize it
				(startX, startY, endX, endY) = box.astype("int")
				cv2.rectangle(frame, (startX, startY), (endX, endY),
					(0, 255, 0), 2)
				#self.statfl=1
				#self.stat.publish(self.statfl)
				self.stat.publish(1)
			#else:
			#	self.statfl=0
			#self.stat.publish(self.statfl)
		# update our centroid tracker using the computed set of bounding
		# box rectangles
		objects = self.ct.update(rects)
		# loop over the tracked objects
		for (objectID, centroid) in objects.items():
			# draw both the ID of the object and the centroid of the
			# object on the output frame
			text = "ID {}".format(objectID)
			cv2.putText(frame, text, (centroid[0] - 10, centroid[1] - 10),
				cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
			cv2.circle(frame, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)

		# show the output frame
		self.image_pub.publish(self.ros_bridge.cv2_to_imgmsg(frame, 'bgr8'))
		#cv2.imshow("Frame", frame)
		#key = cv2.waitKey(1) & 0xFF

		# if the `q` key was pressed, break from the loop
		#if key == ord("q"):
		#	sys.exit(1)
		# do a bit of cleanup
		#cv2.destroyAllWindows()
		#vs.stop()
def main():
	rospy.init_node('snr')
	test = objtracker()
	while not rospy.is_shutdown():
		rospy.spin()
		sys.exit(1)

if __name__ == '__main__':
	main()
