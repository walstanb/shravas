from Tkinter import *
import Tkinter as tk

root = Tk()

class RoundedButton(tk.Canvas):
	def __init__(self, parent, width, height, color, bg, text="", fill="#000000", font="font", command=None, cornerradius=0, padding=0):
		if height/width>=1:
			if cornerradius == 0:
				cornerradius=width/2
			else:
				cornerradius=cornerradius
		elif height/width<1:
			if cornerradius == 0:
				cornerradius=height/2
			else:
				cornerradius=cornerradius
		tk.Canvas.__init__(self, parent, borderwidth=0, 
			relief="flat", highlightthickness=0, bg=bg)
		self.command = command

		'''if cornerradius > 0.5*width:
			print("Error: cornerradius is greater than width.")
			return None

		if cornerradius > 0.5*height:
			print("Error: cornerradius is greater than height.")
			return None'''

		rad = 2*cornerradius
		#font=font+" "+
		def shape():
			self.create_polygon((padding,height-cornerradius-padding,padding,cornerradius+padding,padding+cornerradius,padding,width-padding-cornerradius,padding,width-padding,cornerradius+padding,width-padding,height-cornerradius-padding,width-padding-cornerradius,height-padding,padding+cornerradius,height-padding), fill=color, outline=color)
			self.create_arc((padding,padding+rad,padding+rad,padding), start=90, extent=90, fill=color, outline=color)
			self.create_arc((width-padding-rad,padding,width-padding,padding+rad), start=0, extent=90, fill=color, outline=color)
			self.create_arc((width-padding,height-rad-padding,width-padding-rad,height-padding), start=270, extent=90, fill=color, outline=color)
			self.create_arc((padding,height-padding-rad,padding+rad,height-padding), start=180, extent=90, fill=color, outline=color)
			self.create_text((width/2),(height/2),text=text,fill=fill,font=font, anchor='center')


		id = shape()
		(x0,y0,x1,y1)  = self.bbox("all")
		width = (x1-x0)
		height = (y1-y0)
		self.configure(width=width, height=height)
		self.bind("<ButtonPress-1>", self._on_press)
		self.bind("<ButtonRelease-1>", self._on_release)

	def _on_press(self, event):
		self.configure(relief="sunken")

	def _on_release(self, event):
		self.configure(relief="raised")
		if self.command is not None:
			self.command()

def test():
	print("Hello")

canvas = Canvas(root, height=300, width=500)
canvas.pack()

button = RoundedButton(root, 200, 20, 'red', '#d9d9d9', 'helloooo', command=test)
button.place(relx=.5, rely=.5)

root.mainloop()