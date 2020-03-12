#!/usr/bin/env python
from graphics import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import String
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

#These are the variables that store the information required for whycon to pixel conversion
wyc_x_low = -2 #Upper limit of whycon X
wyc_x_high = 2 #Lower limit of X
wyc_y_low = -1.5 #Upper limit of Y
wyc_y_high = 1.5 #Lower limit of Y
res_x = 960 #The Horizontal resolution of the window to be drawn
res_y = 720 #The Vertical resolution of the window to be drawn
scale_x = res_x / (wyc_x_high - wyc_x_low)
scale_y = res_y / (wyc_y_high - wyc_y_low)


class circle(Circle):

    current_x = 0
    current_y = 0

    def moveto(self, x, y):
        x = (x-wyc_x_low)*scale_x
        y = (y-wyc_y_low)*scale_y
        self.move(x-self.current_x, y-self.current_y)
        self.current_x = x
        self.current_y = y


class gooey():
	
    def __init__(self):
    	print("Started GUI")
        rospy.init_node('gooey')

        rospy.Subscriber('whycon/poses', PoseArray, self.get_pose)
        #Variables that hold the drone's WhyCon coordinates subscribed from ROS
        self.drone_x = 0
        self.drone_y = 0
        self.drone_z = 0

        rospy.Subscriber('wp/x', Float64, self.get_wpx)
        rospy.Subscriber('wp/y', Float64, self.get_wpy)
        #Variables to hold the current point the drone is headed (Published by the pilot.py)
        self.target_x=0.0
        self.target_y=0.0

        #Listens to the pilot's print requirements
        rospy.Subscriber('pilot/display_messages', String, self.get_message)

        #This is the window object that we are going to draw all our elements inside
        self.win = GraphWin("QuadDrop", res_x, res_y+20)

        #The target is a circle object that represents the current location the drone is trying to get to
        self.target = circle(Point(0,0), 15)
        self.target.setOutline(color_rgb(0,200,0))
        self.target.setWidth(2)

        #The target is a circle object that represents the current location the drone is trying to get to
        self.drone = circle(Point(0,0), 7)
        self.drone.setFill(color_rgb(0,120,255))
        self.drone.setWidth(0)

        #message is a Text object to show what the pilot was to print on console
        self.message = Text(Point(res_x/2, res_y+5), "")

        self.message.draw(self.win)
        self.target.draw(self.win)
        self.drone.draw(self.win)

        self.drawWaypoints()


    def drawWaypoints(self):
        '''The function to draw the delivery locations on the screen'''
        try:
            f = open("/home/walst/catkin_ws/src/shravas/src/coordinates.csv","r")
            line = f.readline()
            tgno = 1
            while(1):

                line = f.readline()

                if(line):
                    row = list(line.split(","))
                    if(int(row[4]) == 1):
                        waypoint = circle(Point(0,0), 15)
                        waypoint.setWidth(2)
                        waypoint.draw(self.win)
                        waypoint.moveto(float(row[0]), float(row[1]))
                        targetnumber = Text(waypoint.getCenter(),str(tgno))
                        targetnumber.draw(self.win)
                        tgno += 1

                else: break

            f.close()

        except:
            print("No Coordinates file found! Please create the coordinates.csv and maintain headers and format : x,y,x,qr,delivery")



    def updateElements(self):
    	'''The function that gets called from main infinitely to update the locations of the drone and its heading point on screen'''
        self.target.moveto(self.target_x, self.target_y)
        self.drone.moveto(self.drone_x, self.drone_y)

    
    #Function Name:get_pose
    #Input:         takes poseArray message from whycon/poses
    #Output:        sets the value of drone_x, drone_y, drone_z
    #Logic:         subscribes to whycon/poses to get the coordinates of the whycon marker placed on the drone
    #Example Call:  get_pose(data)
    def get_pose(self, pose):
        self.drone_x = pose.poses[0].position.x
        self.drone_y = pose.poses[0].position.y
        self.drone_z = pose.poses[0].position.z

    #Function Name:get_wpx
    #Input:         takes target coordinate X message from wp/x
    #Output:        sets the value of target_x
    #Logic:         subscribes to wp/x to get target X coordinate
    #Example Call:  get_wpx(data)
    def get_wpx(self, msg):
        self.target_x = msg.data

    #Function Name:get_wpy
    #Input:         takes target coordinate Y message from wp/y
    #Output:        sets the value of target_y
    #Logic:         subscribes to wp/y to get target Y coordinate
    #Example Call:  get_wpy(data)
    def get_wpy(self, msg):
        self.target_y = msg.data

	#Function Name:get_message
    #Input:         takes mesage from pilot
    #Output:        sets the value of message.setText
    #Logic:         subscribes to pilot/display_messgage to get message string over ROS
    #Example Call:  get_message(data)
    def get_message(self, msg):
        self.message.setText(str(msg.data))
        print(str(msg.data))




if __name__ == '__main__':
    gui = gooey()

    while True:
        gui.updateElements()