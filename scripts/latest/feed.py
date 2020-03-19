#!/usr/bin/env python
import numpy
import av
import cv2
import time
import datetime
import os
import tellopy
import rospy
#from imutils.video import VideoStream
from pyzbar import pyzbar
import imutils
from sensor_msgs.msg import Image

class qrcode():

    def __init__(self):
        rospy.init_node('qrcode')
        self.flag=0

        rospy.Subscriber('/dronefeed', Image, self.process_frame)

    def process_frame(self, image):
        print("Entered FEED.PY")
    #def process_frame(self, frame):
        """convert frame to cv2 image and show"""
        #image = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)
        #image =_ self.write_hud(image)
        
        #image = imutils.resize(image, width=400)
        
        # find the barcodes in the frame and decode each of the barcodes
        if(self.flag==0):
            barcodes = pyzbar.decode(image)
            for barcode in barcodes:
                self.flag=1
                barcodeData = barcode.data.decode("utf-8")
                print("\n--------------------------------------------------------------\n")
                print("Bracode Data:  "+barcodeData)
                print("\n--------------------------------------------------------------\n")
        # loop over the detected barcode
        image = imutils.resize(image, width=720)
        
       
        #self.image_pub.publish(self.ros_bridge.cv2_to_imgmsg(image, 'bgr8'))
        cv2.imshow("Barcode Scanner", image)
        key = cv2.waitKey(1) & 0xFF

if __name__ =="__main__":
    
    tellotrack = qrcode()    
    #test = qrcode()
    while not rospy.is_shutdown():
        rospy.spin()
        sys.exit(1)