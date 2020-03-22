import Tkinter as tk
import numpy
import av
import cv2,cv_bridge
import time
import datetime
import os,subprocess
from subprocess import check_output
import rospy
import QuadDrop
import imutils
import PIL.Image, PIL.ImageTk
import csvio
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from tello_driver.msg import TelloStatus

drobj = rospy.Publisher('/drone_init', Int32, queue_size=1)
def start():
	drobj.publish(1)
def emstop():
	drobj.publish(0)

class Gui():

	def __init__(self,obj=None):
		global top
		top=obj
		rospy.init_node('gui')
		self.ros_bridge = cv_bridge.CvBridge()
		self.point=None
		self.draw_deliv()
		self.nxt=None
		rospy.Subscriber('whycon/poses', PoseArray, self.draw_point)
		rospy.Subscriber('drone_feed', Image, self.show_feed)
		rospy.Subscriber('tello/status', TelloStatus, self.tello_status)
		rospy.Subscriber('tello/odom', Odometry, self.tello_odom)
		rospy.Subscriber('tello/imu', Imu, self.tello_imu)
		rospy.Subscriber('status_msg', String, self.prnt_msg)
		rospy.Subscriber('/wp_cords', PoseArray, self.draw_nxt)

		Lphoto = PIL.ImageTk.PhotoImage(PIL.Image.open("/home/walst/catkin_ws/src/shravas/scripts/gui/logoba.jpeg").resize((200, 50), PIL.Image.ANTIALIAS))
		top.Logo.configure(image = Lphoto)
		top.Logo.image = Lphoto

		Cphoto = PIL.ImageTk.PhotoImage(PIL.Image.open("/home/walst/catkin_ws/src/shravas/scripts/gui/credits.png").resize((857, 408), PIL.Image.ANTIALIAS))
		top.CreditsSlide.configure(image = Cphoto)
		top.CreditsSlide.image = Cphoto

		Aphoto = PIL.ImageTk.PhotoImage(PIL.Image.open("/home/walst/catkin_ws/src/shravas/scripts/gui/about.png").resize((857, 408), PIL.Image.ANTIALIAS))
		top.AboutSlide.configure(image = Aphoto)
		top.AboutSlide.image = Aphoto

		Dphoto = PIL.ImageTk.PhotoImage(PIL.Image.open("/home/walst/catkin_ws/src/shravas/scripts/gui/dmode.png").resize((15, 15), PIL.Image.ANTIALIAS))
		top.dmodeButton.configure(image = Dphoto)
		top.dmodeButton.image = Dphoto

		fbarcol,bbarcol="#0078ff","#d9d9d9"
		fsccol,bsccol="#d9d9d9","#d9d9d9"
		top.style.configure('Horizontal.TProgressbar',foreground=fbarcol, background=fbarcol,troughcolor=bbarcol)
		top.style.configure('Horizontal.TScale',foreground=fsccol, background=fsccol,troughcolor=bsccol)

		top.StatusMesaages.configure(state='normal')
		top.StatusMesaages.insert('end', 'Status Messages\n- - - - - - - - - - - - - - - - - - - - - - - -')
		top.StatusMesaages.configure(state='disabled')
		top.StatusMesaages.see("end")

		top.FlightData.configure(state='normal')
		top.FlightData.insert('end', 'Flight Data\n- - - - - - - - - - - - - - - - - - - - - - - -\n')
		top.FlightData.configure(state='disabled')

		
		#self.draw_nxt()
		

		#root = tk.Tk()
		#self.top,self.win=QuadDrop.create_Toplevel1(tk.Tk())
		#QuadDrop.vp_start_gui()
		#root.Status.configure(text='''DisConnected''')
		#self.top.mainloop()


		#top.XYPositionalData.create_oval(x-15, y-15, x+15, y+15,outline="#64eb34", width=2)
		#top.XYPositionalData.create_oval(x-15, y-15, x+15, y+15,outline="#0078ff", width=2)
	
	#def create_circle():

	def scal(self,x,y):
		return x*top.sca+(top.fcan/2),(-1*y)*top.sca+(top.fcan/2)
		
	def draw_deliv(self):
		ls=csvio.csvread("/home/walst/catkin_ws/src/shravas/scripts/coords.csv")
		for i in range(len(ls)):
			x,y=self.scal(float(ls[i]['x']),float(ls[i]['y']))
			top.XYPositionalData.create_oval(x-15, y-15, x+15, y+15,outline="#0078ff", width=2)

		'''if self.nxt!=None:
		top.XYPositionalData.delete(self.nxt)
		self.nxt.configure(outline="#ff7b00")'''
	def draw_nxt(self, pose):
		self.prevx,self.prevy=x,y=self.scal(pose.poses[0].position.x, pose.poses[0].position.y)
		if self.nxt!=None:
			top.XYPositionalData.delete(self.nxt)
			self.nxt=top.XYPositionalData.create_oval(self.prevx-15, self.prevy-15, self.prevx+15, self.prevy+15, outline="#ff7b00", width=2)
		self.nxt=top.XYPositionalData.create_oval(x-15, y-15, x+15, y+15, outline="#64eb34", width=2)

	def draw_point(self,pose):
		#fcan_x=fcan_y=2000/2
		can_x,can_y=888,491
		#scalex=scaley=100
		if self.point!=None:
			top.XYPositionalData.delete(self.point)
		x,y=self.scal(pose.poses[0].position.x, pose.poses[0].position.y)
		self.point=top.XYPositionalData.create_oval(x-5, y-5, x+5, y+5, fill="#0078ff", outline="#0078ff", width=2)
		



	def get_pose(self,pose):
		top.Status.configure(text=str(pose.poses[0].position.x))

	def prnt_msg(self,msg):
		top.StatusMesaages.configure(state='normal')
		top.StatusMesaages.configure(foreground="white")
		top.StatusMesaages.insert('end', "\n"+msg.data)
		#print('\n'+str(msg.data))
		top.StatusMesaages.configure(state='disabled')
		top.StatusMesaages.see("end")

	def show_feed(self, image):
		frame = self.ros_bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
		frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		frame = imutils.resize(frame, width=656,height=405)
		
		photo = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(frame))
		#top.Status.configure(text="Hello")
		
		top.DroneLiveFeed.configure(image = photo)
		top.DroneLiveFeed.image = photo
	
	def tello_status(self,data):
		top.Status.configure(foreground="#2cbc00")
		top.Status.configure(text='''Connected''')

		if(data.battery_percentage<=30):
			top.style.configure('batt.Horizontal.TProgressbar',foreground='red', background='red')
		elif(data.battery_percentage<=80 and data.battery_percentage>30):
			top.style.configure('batt.Horizontal.TProgressbar',foreground='#0078ff', background='#0078ff')
		elif(data.battery_percentage>80):
			top.style.configure('batt.Horizontal.TProgressbar',foreground='#2cbc00', background='#2cbc00')

		top.Battery.configure(value=str(data.battery_percentage))
		top.Altitude.configure(text=str(100*data.height_m))
		top.AltitudeBar.configure(value=str(100*data.height_m))
		top.Speed.configure(text=str(100*data.speed_horizontal_mps))
		top.SpeedBar.configure(value=str(100*data.speed_horizontal_mps))

		
		top.Mode.configure(text=str(data.fly_mode))
		top.Wifi.configure(text=str(data.cmd_roll_ratio)+"%")

		top.FlightData.configure(state='normal')
		top.FlightData.delete('3.0', 'end')
		strr="Flight Time : "+str(data.flight_time_sec)
		strr+="\nBattery Percentage : "+str(data.battery_percentage)
		strr+="\nFlight time left : "+str(data.drone_fly_time_left_sec)
		strr+="\nTemperature : "+str(data.temperature_height_m)
		strr+="\nWind State : "+("Windy " if data.wind_state==1 else "Not Windy")
		top.FlightData.insert('3.0',strr)
		top.FlightData.update()
		top.FlightData.configure(state='disabled')

	def tello_odom(self,data):
		top.mvo.configure(state='normal')
		top.mvo.delete('1.0', 'end')
		top.mvo.update()
		strr=" Odometry Data\n   Position:\n   x: "+str(data.pose.pose.position.x)+"\ty: "+str(data.pose.pose.position.y)+"\n   z: "+str(data.pose.pose.position.z)
		strr+="\n   Orientation:\n   w: "+str(data.pose.pose.orientation.w)+"\tx: "+str(data.pose.pose.orientation.x)+"\n   y: "+str(data.pose.pose.orientation.y)+"\tz: "+str(data.pose.pose.orientation.z)
		top.mvo.insert('1.0',strr)
		top.mvo.configure(state='disabled')

	def tello_imu(self,data):
		top.imu.configure(state='normal')
		top.imu.delete('1.0', 'end')
		top.imu.update()
		strr=" Gyroscope and IMU Data\n   Angular Velocity:\n   x: "+str(data.angular_velocity.x)+"\ty: "+str(data.angular_velocity.y)+"\n   z: "+str(data.angular_velocity.z)
		strr+="\n   Linear Acceleration:\n   x: "+str(data.linear_acceleration.x)+"\ty: "+str(data.linear_acceleration.y)+"\n   z: "+str(data.linear_acceleration.z)
		top.imu.insert('1.0',strr)
		top.imu.configure(state='disabled')
		#top.odm.see("end")

	def tello_events(self,msg):
		top.TelloEvents.configure(state='normal')
		top.TelloEvents.configure(foreground="white")
		top.TelloEvents.insert('end', "\n"+msg.data)
		#print('\n'+str(msg.data))
		top.TelloEvents.configure(state='disabled')
		top.TelloEvents.see("end")
	#def tello_odm(self,data):