#! /usr/bin/env python
#  -*- coding: utf-8 -*-
#
# GUI module generated by PAGE version 5.0.3
#  in conjunction with Tcl version 8.6
#    Mar 16, 2020 02:36:39 AM IST  platform: Linux

import sys
import os,subprocess
from subprocess import check_output
import gui

try:
    import Tkinter as tk
except ImportError:
    import tkinter as tk

try:
    import ttk
    py3 = False
except ImportError:
    import tkinter.ttk as ttk
    py3 = True

import QuadDrop_support
import os.path

def vp_start_gui():
    '''Starting point when module is the main routine.'''
    global val, w, root
    root = tk.Tk()
    QuadDrop_support.set_Tk_var()
    top = Toplevel1 (root)
    test=gui.Gui(top)
    QuadDrop_support.init(root, top)
    root.mainloop()

w = None
def create_Toplevel1(rt, *args, **kwargs):
    '''Starting point when module is imported by another module.
       Correct form of call: 'create_Toplevel1(root, *args, **kwargs)' .'''
    global w, w_win, root
    #rt = root
    root = rt
    w = tk.Toplevel (root)
    QuadDrop_support.set_Tk_var()
    top = Toplevel1 (w)
    QuadDrop_support.init(w, top, *args, **kwargs)
    return (w, top)

def destroy_Toplevel1():
    global w
    w.destroy()
    w = None

class Toplevel1:
    def __init__(self, top=None):
        #self.colorflag=0
        '''This class configures and populates the toplevel window.
           top is the toplevel containing window.'''
        _bgcolor = '#d9d9d9'  # X11 color: 'gray85'
        _fgcolor = '#000000'  # X11 color: 'black'
        _compcolor = '#d9d9d9' # X11 color: 'gray85'
        _ana1color = '#d9d9d9' # X11 color: 'gray85'
        _ana2color = '#ececec' # Closest X11 color: 'gray92'
        self.style = ttk.Style()
        if sys.platform == "win32":
            self.style.theme_use('winnative')
        self.style.configure('.',background=_bgcolor)
        self.style.configure('.',foreground=_fgcolor)
        self.style.configure('.',font="TkDefaultFont")
        
        self.style.map('.',background=
            [('selected', _compcolor), ('active',_ana2color)])

        top.geometry("1863x1050+57+24")
        top.minsize(1, 1)
        top.maxsize(1905, 1050)
        top.resizable(1, 1)
        top.title("QuadDrop")
        top.configure(highlightcolor="black")

        self.StatusMesaageFrame = ttk.Labelframe(top)
        self.StatusMesaageFrame.place(relx=0.016, rely=0.629, relheight=0.31
                , relwidth=0.499)
        self.StatusMesaageFrame.configure(relief='')
        self.StatusMesaageFrame.configure(text='''Status Messages''')

        self.StatusMesaages = tk.Text(self.StatusMesaageFrame)
        self.StatusMesaages.place(relx=0.022, rely=0.064, relheight=0.887
                , relwidth=0.957, bordermode='ignore')
        self.StatusMesaages.configure(background="#000000")
        self.StatusMesaages.configure(blockcursor="1")
        self.StatusMesaages.configure(cursor="arrow")
        self.StatusMesaages.configure(exportselection="0")
        self.StatusMesaages.configure(font="-family {Ubuntu} -size 13")
        self.StatusMesaages.configure(foreground="white")
        self.StatusMesaages.configure(selectbackground="#c4c4c4")
        self.StatusMesaages.configure(state='disabled')
        self.StatusMesaages.configure(wrap="word")

        #ScrollBar Functionality
        self.vsb = tk.Scrollbar(self.StatusMesaageFrame, orient="vertical", command=self.StatusMesaages.yview)
        self.StatusMesaages.configure(yscrollcommand=self.vsb.set)
        self.vsb.pack(side="right", fill="y")
        self.StatusMesaages.pack(side="left", fill="both", expand=True)

        self.style.configure('TNotebook.Tab', background=_bgcolor)
        self.style.configure('TNotebook.Tab', foreground=_fgcolor)
        self.style.map('TNotebook.Tab', background=
            [('selected', _compcolor), ('active',_ana2color)])
        self.TNotebook1 = ttk.Notebook(top)
        self.TNotebook1.place(relx=0.526, rely=0.515, relheight=0.422
                , relwidth=0.463)
        self.TNotebook1.configure(takefocus="")
        self.DeliveryDetailsPage = tk.Frame(self.TNotebook1)
        self.TNotebook1.add(self.DeliveryDetailsPage, padding=3)
        self.TNotebook1.tab(0, text="Delivery Details", compound="left"
                ,underline="-1", )
        self.DroneFlightDataPage = tk.Frame(self.TNotebook1)
        self.TNotebook1.add(self.DroneFlightDataPage, padding=3)
        self.TNotebook1.tab(1, text="Drone Flight Data", compound="left"
                ,underline="-1", )
        self.AboutPage = tk.Frame(self.TNotebook1)
        self.TNotebook1.add(self.AboutPage, padding=3)
        self.TNotebook1.tab(2, text="About",compound="none",underline="-1",)
        self.CreditsPage = tk.Frame(self.TNotebook1)
        self.TNotebook1.add(self.CreditsPage, padding=3)
        self.TNotebook1.tab(3, text="Credits",compound="none",underline="-1",)
        self.CreditsPage.configure(cursor="heart")

        self.TSeparator1 = ttk.Separator(self.DeliveryDetailsPage)
        self.TSeparator1.place(relx=0.298, rely=0.048, relheight=0.905)
        self.TSeparator1.configure(orient="vertical")

        self.DeliveryList = tk.Listbox(self.DeliveryDetailsPage)
        self.DeliveryList.place(relx=0.023, rely=0.048, relheight=0.914
                , relwidth=0.26)
        self.DeliveryList.configure(background="white")
        self.DeliveryList.configure(font="TkFixedFont")
        self.DeliveryList.configure(selectbackground="#c4c4c4")

        self.DeliveryDetails = tk.Text(self.DeliveryDetailsPage)
        self.DeliveryDetails.place(relx=0.314, rely=0.05, relheight=0.907
                , relwidth=0.67)
        self.DeliveryDetails.configure(background="white")
        self.DeliveryDetails.configure(blockcursor="1")
        self.DeliveryDetails.configure(cursor="arrow")
        self.DeliveryDetails.configure(exportselection="0")
        self.DeliveryDetails.configure(font="TkTextFont")
        self.DeliveryDetails.configure(selectbackground="#c4c4c4")
        self.DeliveryDetails.configure(state='disabled')
        self.DeliveryDetails.configure(wrap="word")

        self.TelloEventsFrame = ttk.Labelframe(self.DroneFlightDataPage)
        self.TelloEventsFrame.place(relx=0.012, rely=0.024, relheight=0.298
                , relwidth=0.974)
        self.TelloEventsFrame.configure(relief='')
        self.TelloEventsFrame.configure(text='''Tello Events''')

        self.TelloEvents = tk.Text(self.TelloEventsFrame)
        self.TelloEvents.place(relx=0.012, rely=0.164, relheight=0.754
                , relwidth=0.976, bordermode='ignore')
        self.TelloEvents.configure(background="white")
        self.TelloEvents.configure(blockcursor="1")
        self.TelloEvents.configure(cursor="arrow")
        self.TelloEvents.configure(font="TkTextFont")
        self.TelloEvents.configure(selectbackground="#c4c4c4")
        self.TelloEvents.configure(state='disabled')
        self.TelloEvents.configure(wrap="word")

        self.Canvas3 = tk.Canvas(self.DroneFlightDataPage)
        self.Canvas3.place(relx=0.488, rely=0.349, relheight=0.617
                , relwidth=0.501)
        self.Canvas3.configure(borderwidth="2")
        self.Canvas3.configure(relief="ridge")
        self.Canvas3.configure(selectbackground="#c4c4c4")

        self.FlightData = tk.Text(self.Canvas3)
        self.FlightData.place(relx=0.0, rely=0.0, relheight=0.996
                , relwidth=1.012)
        self.FlightData.configure(background="white")
        self.FlightData.configure(cursor="arrow")
        self.FlightData.configure(font="TkTextFont")
        self.FlightData.configure(selectbackground="#c4c4c4")
        self.FlightData.configure(state='disabled')
        self.FlightData.configure(wrap="word")

        self.DroneFlightDataFrame = ttk.Labelframe(self.DroneFlightDataPage)
        self.DroneFlightDataFrame.place(relx=0.012, rely=0.341, relheight=0.622
                , relwidth=0.466)
        self.DroneFlightDataFrame.configure(relief='')
        self.DroneFlightDataFrame.configure(text='''Drone Flight Data''')

        self.mvo = tk.Text(self.DroneFlightDataFrame)
        self.mvo.place(relx=0.025, rely=0.078, relheight=0.439, relwidth=0.938
                , bordermode='ignore')
        self.mvo.configure(background="#ffffff")
        self.mvo.configure(borderwidth="0")
        self.mvo.configure(cursor="arrow")
        self.mvo.configure(font="TkTextFont")
        self.mvo.configure(selectbackground="#c4c4c4")
        self.mvo.configure(state='disabled')
        self.mvo.configure(wrap="word")

        self.imu = tk.Text(self.DroneFlightDataFrame)
        self.imu.place(relx=0.025, rely=0.51, relheight=0.439, relwidth=0.938
                , bordermode='ignore')
        self.imu.configure(background="#ffffff")
        self.imu.configure(borderwidth="0")
        self.imu.configure(cursor="arrow")
        self.imu.configure(font="TkTextFont")
        self.imu.configure(selectbackground="#c4c4c4")
        self.imu.configure(state='disabled')
        self.imu.configure(wrap="word")

        self.XYPositionalDataFrame = ttk.Labelframe(top)
        self.XYPositionalDataFrame.place(relx=0.016, rely=0.057, relheight=0.558
                , relwidth=0.499)
        self.XYPositionalDataFrame.configure(relief='')
        self.XYPositionalDataFrame.configure(text='''XY Positional Data''')

        self.Scale = ttk.Scale(self.XYPositionalDataFrame, from_=0, to=1.0, style='Horizontal.TScale')
        self.Scale.place(relx=0.022, rely=0.947, relwidth=0.958, relheight=0.0
                , height=17, bordermode='ignore')
        self.Scale.configure(takefocus="")
        self.Scale.configure(cursor="sb_h_double_arrow")
        

        self.XYPositionalData = tk.Canvas(self.XYPositionalDataFrame)
        self.XYPositionalData.place(relx=0.022, rely=0.051, relheight=0.86, relwidth=0.959, bordermode='ignore')
        #self.XYPositionalData.configure(borderwidth="2")
        self.XYPositionalData.configure(cursor="crosshair")
        self.XYPositionalData.configure(relief="ridge")
        self.XYPositionalData.configure(background="#ffffff")
        self.XYPositionalData.configure(scrollregion=(0,0,2000,2000))
        #self.XYPositionalData.bind('<Configure>', QuadDrop_support.create_grid)

        #self.xsb.grid(row=1, column=0, sticky="ew")
        #self.ysb.grid(row=0, column=1, sticky="ns")
        #self.XYPositionalData.grid(row=200, column=200, sticky="nsew")
        #self.grid_rowconfigure(0, weight=1)
        #self.grid_columnconfigure(0, weight=1)
        self.XYPositionalData.bind("<ButtonPress-1>", self.scroll_start)
        self.XYPositionalData.bind("<B1-Motion>", self.scroll_move)

        self.Progressbar = ttk.Progressbar(top, style="Horizontal.TProgressbar")
        self.Progressbar.place(relx=0.236, rely=0.957, relwidth=0.733
                , relheight=0.0, height=20)
        self.Progressbar.configure(length="1365")


        self.Hud = ttk.Frame(top)
        self.Hud.place(relx=0.907, rely=0.067, relheight=0.433, relwidth=0.083)
        self.Hud.configure(relief='groove')
        self.Hud.configure(borderwidth="2")
        self.Hud.configure(relief="groove")
        self.Hud.configure(cursor="arrow")

        self.StatusLabel = ttk.Label(self.Hud)
        self.StatusLabel.place(relx=0.065, rely=0.022, height=16, width=49)
        self.StatusLabel.configure(background="#d9d9d9")
        self.StatusLabel.configure(foreground="#000000")
        self.StatusLabel.configure(font="TkDefaultFont")
        self.StatusLabel.configure(relief="flat")
        self.StatusLabel.configure(anchor='w')
        self.StatusLabel.configure(justify='left')
        self.StatusLabel.configure(text='''Status :''')

        self.Status = ttk.Label(self.Hud)
        self.Status.place(relx=0.455, rely=0.022, height=16, width=67)
        self.Status.configure(background="#d9d9d9")
        #self.Status.configure(foreground="#2cbc00")
        self.Status.configure(font="TkDefaultFont")
        self.Status.configure(relief="flat")
        self.Status.configure(anchor='w')
        self.Status.configure(justify='left')

        self.BatteryLabel = ttk.Label(self.Hud)
        self.BatteryLabel.place(relx=0.065, rely=0.11, height=16, width=59)
        self.BatteryLabel.configure(background="#d9d9d9")
        self.BatteryLabel.configure(foreground="#000000")
        self.BatteryLabel.configure(font="TkDefaultFont")
        self.BatteryLabel.configure(relief="flat")
        self.BatteryLabel.configure(anchor='w')
        self.BatteryLabel.configure(justify='left')
        self.BatteryLabel.configure(text='''Battery :''')

        self.WifiLabel = ttk.Label(self.Hud)
        self.WifiLabel.place(relx=0.065, rely=0.066, height=16, width=64)
        self.WifiLabel.configure(background="#d9d9d9")
        self.WifiLabel.configure(foreground="#000000")
        self.WifiLabel.configure(font="TkDefaultFont")
        self.WifiLabel.configure(relief="flat")
        self.WifiLabel.configure(anchor='w')
        self.WifiLabel.configure(justify='left')
        self.WifiLabel.configure(text='''Wifi :''')

        self.Battery = ttk.Progressbar(self.Hud, style="batt.Horizontal.TProgressbar")
        self.Battery.place(relx=0.455, rely=0.112, relwidth=0.39, relheight=0.0
                , height=14)
        self.Battery.configure(length="60")
        self.Battery.configure(value="69.0")
        

        self.AltitudeBar = ttk.Progressbar(self.Hud, style="Horizontal.TProgressbar")
        self.AltitudeBar.place(relx=0.195, rely=0.262, relwidth=0.195
                , relheight=0.0, height=296)
        self.AltitudeBar.configure(orient="vertical")
        self.AltitudeBar.configure(length="90")
        self.AltitudeBar.configure(value="70.2")

        self.SpeedBar = ttk.Progressbar(self.Hud, style="Horizontal.TProgressbar")
        self.SpeedBar.place(relx=0.578, rely=0.262, relwidth=0.195, relheight=0.0
                , height=296)
        self.SpeedBar.configure(orient="vertical")
        self.SpeedBar.configure(length="90")
        self.SpeedBar.configure(value="30")

        self.AltitudeLabel = ttk.Label(self.Hud)
        self.AltitudeLabel.place(relx=0.13, rely=0.934, height=16, width=57)
        self.AltitudeLabel.configure(background="#d9d9d9")
        self.AltitudeLabel.configure(foreground="#000000")
        self.AltitudeLabel.configure(font="TkDefaultFont")
        self.AltitudeLabel.configure(relief="flat")
        self.AltitudeLabel.configure(anchor='w')
        self.AltitudeLabel.configure(justify='left')
        self.AltitudeLabel.configure(takefocus="Altitude")
        self.AltitudeLabel.configure(text='''Altitude''')

        self.SpeedLabel = ttk.Label(self.Hud)
        self.SpeedLabel.place(relx=0.558, rely=0.934, height=16, width=47)
        self.SpeedLabel.configure(background="#d9d9d9")
        self.SpeedLabel.configure(foreground="#000000")
        self.SpeedLabel.configure(font="TkDefaultFont")
        self.SpeedLabel.configure(relief="flat")
        self.SpeedLabel.configure(anchor='w')
        self.SpeedLabel.configure(justify='left')
        self.SpeedLabel.configure(takefocus="Altitude")
        self.SpeedLabel.configure(text='''Speed''')

        self.ModeLabel = ttk.Label(self.Hud)
        self.ModeLabel.place(relx=0.065, rely=0.152, height=16, width=47)
        self.ModeLabel.configure(background="#d9d9d9")
        self.ModeLabel.configure(foreground="#000000")
        self.ModeLabel.configure(font="TkDefaultFont")
        self.ModeLabel.configure(relief="flat")
        self.ModeLabel.configure(anchor='w')
        self.ModeLabel.configure(justify='left')
        self.ModeLabel.configure(text='''Mode :''')

        self.Mode = ttk.Label(self.Hud)
        self.Mode.place(relx=0.455, rely=0.154, height=16, width=67)
        self.Mode.configure(background="#d9d9d9")
        self.Mode.configure(foreground="#000000")
        self.Mode.configure(font="TkDefaultFont")
        self.Mode.configure(relief="flat")
        self.Mode.configure(anchor='w')
        self.Mode.configure(justify='left')
        self.Mode.configure(text='''Normal''')

        self.Wifi = ttk.Label(self.Hud)
        self.Wifi.place(relx=0.455, rely=0.066, height=17, width=40)
        self.Wifi.configure(background="#d9d9d9")
        self.Wifi.configure(foreground="#000000")
        self.Wifi.configure(font="TkDefaultFont")
        self.Wifi.configure(relief="flat")
        self.Wifi.configure(anchor='w')
        self.Wifi.configure(justify='left')
        self.Wifi.configure(text='''50%''')
        self.Wifi.configure(cursor="fleur")

        self.Altitude = ttk.Label(self.Hud)
        self.Altitude.place(relx=0.195, rely=0.213, height=16, width=30)
        self.Altitude.configure(background="#d9d9d9")
        self.Altitude.configure(foreground="#000000")
        self.Altitude.configure(font="TkDefaultFont")
        self.Altitude.configure(relief="flat")
        self.Altitude.configure(anchor='w')
        self.Altitude.configure(justify='left')
        self.Altitude.configure(text='''70.2''')

        self.Speed = ttk.Label(self.Hud)
        self.Speed.place(relx=0.578, rely=0.213, height=16, width=30)
        self.Speed.configure(background="#d9d9d9")
        self.Speed.configure(foreground="#000000")
        self.Speed.configure(font="TkDefaultFont")
        self.Speed.configure(relief="flat")
        self.Speed.configure(anchor='w')
        self.Speed.configure(justify='left')
        self.Speed.configure(text='''70.2''')

        self.EmStopButton = tk.Button(top)
        self.EmStopButton.place(relx=0.032, rely=0.952, height=28, width=169)
        self.EmStopButton.configure(activebackground="#ff4545")
        self.EmStopButton.configure(activeforeground="white")
        self.EmStopButton.configure(background="#ff2424")   
        self.EmStopButton.configure(cursor="hand1")
        self.EmStopButton.configure(state='disabled')
        self.EmStopButton.configure(command=gui.emstop)
        self.EmStopButton.configure(text='''Emergency Stop''')

        self.Logo = tk.Label(top)
        self.Logo.place(relx=0.011, rely=0.003, height=50, width=200)
        self.Logo.configure(activebackground="#f9f9f9")
        self.Logo.configure(cursor="heart")
        self.Logo.configure(font="-family {Ubuntu} -size 24")
        self.Logo.configure(text='''QuadDrop''')

        self.DroneLiveFeedFrame = ttk.Labelframe(top)
        self.DroneLiveFeedFrame.place(relx=0.527, rely=0.058, relheight=0.443
                , relwidth=0.372)
        self.DroneLiveFeedFrame.configure(relief='')
        self.DroneLiveFeedFrame.configure(text='''Drone Live Feed''')

        self.DroneLiveFeed = tk.Label(self.DroneLiveFeedFrame)
        self.DroneLiveFeed.place(relx=0.029, rely=0.066, height=405, width=656
                , bordermode='ignore')
        #photo_location = os.path.join(prog_location,"../../TelloPy/files/joystick_and_video.png")
        #global _img0
        #_img0 = tk.PhotoImage(file=photo_location)
        #self.DroneLiveFeed.configure(image=_img0)

        self.StartButton = tk.Button(top)
        self.StartButton.place(relx=0.886, rely=0.019, height=28, width=189)
        self.StartButton.configure(activebackground="#45ed61")
        self.StartButton.configure(activeforeground="#ffffff")
        self.StartButton.configure(cursor="hand1")
        self.StartButton.configure(command=gui.start)
        self.StartButton.configure(text='''Start''')

        self.ConnectButton = tk.Button(top)
        self.ConnectButton.place(relx=0.886, rely=0.019, height=28, width=189)
        self.ConnectButton.configure(activebackground="#f9f9f9")
        self.ConnectButton.configure(command=QuadDrop_support.conn)
        self.ConnectButton.configure(text='''Connect''')

        

        self.EmLock = tk.Checkbutton(top)
        self.EmLock.place(relx=0.016, rely=0.955, relheight=0.019
                , relwidth=0.013)
        self.EmLock.configure(activebackground="#f9f9f9")
        self.EmLock.configure(justify='left', bd=0)
        self.EmLock.configure(command=QuadDrop_support.disable_enable_button)
        self.EmLock.configure(variable=QuadDrop_support.che48)

        self.Error = tk.Label(top)
        self.Error.place(relx=0.693, rely=0.025, height=18, width=337)
        self.Error.configure(anchor='e')
        self.Error.configure(foreground="#d9d9d9")
        self.Error.configure(text='''Connection Error ! Try Again''')


        self.AboutSlide = tk.Label(self.AboutPage)
        self.AboutSlide.place(relx=0.0, rely=0.0, height=408, width=857)
        self.AboutSlide.configure(relief="flat", borderwidth="0")

        self.CreditsSlide = tk.Label(self.CreditsPage)
        self.CreditsSlide.place(relx=0.0, rely=0.0, height=408, width=857)
        self.CreditsSlide.configure(relief="flat", borderwidth="0")
        #self.Error.place_forget()

        self.CallbackButton = tk.Button(top)
        self.CallbackButton.place(relx=0.134, rely=0.952, height=28, width=169)
        self.CallbackButton.configure(activebackground="#f9f9f9")
        self.CallbackButton.configure(text='''Callback''')
        self.CallbackButton.configure(state='disabled')
        self.colorflag=0
        self.dmodeButton = tk.Button(top)
        self.dmodeButton.place(relx=0.975, rely=0.952, height=28, width=29)
        self.dmodeButton.configure(command=QuadDrop_support.dmode)
        
        

        scanoutput = check_output(["iwlist", "wlp2s0", "scan"])
        ssid = "WiFi not found"
        for line in scanoutput.split():
          line = line.decode("utf-8")
          if line[:5]  == "ESSID":
            ssid = line.split('"')[1]

        if(ssid=="QuadDrop"):
            self.ConnectButton.place_forget()
            self.Status.configure(text='''Connected''', foreground="#2cbc00")
        else:
            self.Status.configure(text="Disconnected",foreground="#ff0000")


    def scroll_start(self, event):
        self.XYPositionalData.scan_mark(event.x, event.y)

    def scroll_move(self, event):
        self.XYPositionalData.scan_dragto(event.x, event.y, gain=1)

if __name__ == '__main__':
    vp_start_gui()





