#!/usr/bin/env python3
import serial                                   #Used for serial communication over RS232
import datetime                                 #Used for timing
import ephem                                    #Used for determining azimuth and elevation of celestial bodies
import time                                     #Used for timing
import numpy as np                              #Reuired for arrays
import cv2 as cv                                #Used for computer vision
import math                                     #Required for maths
import picamera                                 #Used for taking pictures
import smbus                                    #Used for I2C
import tflite_runtime.interpreter as tflite     #Used for tensorflow lite models
import tkinter as tk                            #Used for creating GUI
from tkinter import messagebox                  
from PIL import Image, ImageTk
from pa1010d import PA1010D                     #used for GPS

#Function Definitions -----------------------------------------------------------------------------

def twoscomp(data):                             #Return twos complement of 16 bit number
    if data >= 32768:
        return data - 65536
    else:
        return data

def GetMoonPos():                               #Get moon az and el
    Moon = ephem.Moon(Obs)    
    print("Moon Azimuth and Elevation: ", Moon.az, Moon.alt)
    Az = Moon.az * 180 / 3.14159
    El = Moon.alt * 180 / 3.14159
    return Az, El

def GetSunPos():                                #Get sun az and el
    Sun = ephem.Sun(Obs)
    print("Sun Azimuth and Elevation: ", Sun.az, Sun.alt)
    Az = Sun.az * 180 / 3.14159
    El = Sun.alt * 180 / 3.14159
    return Az, El

def SendCmd(Az, El):                            #Convert az and el to a command for the rotator
    global yaw
    Az = round(Az - yaw)
    El = round(El)
    print("Adjusted Azimuth: " + str(Az))
    print("Adjusted Elevetaion: " + str(El))
    if Az > 360:
        Az = Az - 360
    if Az < 0:
        Az = Az + 360

    if Az < 10 :
        Azs = "00" + str(Az)
    elif Az < 100 :
        Azs = "0" + str(Az)
    else:
        Azs = str(Az)
    
    if El > 180:
        Els = "180"
    if El < 0:
        Els = ("000")
    else:
        if El < 10 :
            Els = ("00" + str(El))
        elif El < 100 :
            Els = ("0" + str(El))        
        else:
            Els = str(El)
    print("Sending: w" + Azs + " " + Els + "\r")
    Cmd = bytes("w" + Azs + " " + Els + "\r", 'utf-8')
    Ser.write(Cmd)
    
def Recalibrate():                              #Gets new values for position
    print("Setting Offsets")
    global Lat, Long, Alt, Obs, yaw
    result = gps.update()
    if (gps.data['altitude'] is not None):
        Lat = gps.data['latitude']
        Long = gps.data['longitude']
        Alt = gps.data['altitude']
        Obs.long = Long
        Obs.lat = Lat
        Obs.elevation = Alt
    delta = datetime.datetime.now() - StartTime
    Obs.date = ephem.Date(Time0) + ((delta.seconds + delta.microseconds/1000000)*1.1574076779652387 / 100000)
    if (manual_calibration.get() == 1):
        bus.write_byte_data(add, 0x60, 0x80)
        bus.write_byte_data(add, 0x62, 0x01)
        val0 = bus.read_byte_data(add, 0x68)
        val1 = bus.read_byte_data(add, 0x69)
        val2 = bus.read_byte_data(add, 0x6A)
        val3 = bus.read_byte_data(add, 0x6B)
        x_raw = twoscomp(float(val1 << 8 | val0))
        y_raw = twoscomp(float(val3 << 8 | val2))
        yaw = math.atan2(y_raw, x_raw) * 180 / 3.14159
        if yaw < 0:
            yaw = yaw + 360 
    else:
        yaw = 0

def GetPhoto():                             #Take picture and save resized copies for the CNN
    print("Taking Picture")
    camera.capture('/home/pi/Desktop/Moon.jpg')
    img = cv.imread("/home/pi/Desktop/Moon.jpg")
    img = cv.resize(img, (256, 256))
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    cv.imwrite("/home/pi/Desktop/Moon.jpg", img)
    img = cv.resize(img, (64, 64))
    cv.imwrite("/home/pi/Desktop/Moon_CNN.jpg", img)
        
def GetTarget():                            #Gets position of selected body
    print("Getting Targets")
    TargAz = 0
    TargEl = 0
    if (Track.get() == "Moon"):
        TargAz, TargEl = GetMoonPos()
    if (Track.get() == "Sun"):
        TargAz, TargEl = GetSunPos()
    if (Track.get() == "Manual"):
        TargAz = ManAz
        TargEl = ManEl
    return TargAz, TargEl

def GetCentre():                            #If moon is selected, use CNN and mark returned centre on image
    cenx = 32
    ceny = 32
    if (Track.get() == "Moon"):
        if (Method.get() == Methods[1]):
            print("Using CNN")
            img = cv.imread("/home/pi/Desktop/Moon_CNN.jpg")
            img = cv.resize(img, (64, 64))
            img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            print(img.shape)
            input_data = np.expand_dims(img, axis=0)
            input_data = np.expand_dims(input_data, axis=3)
            input_data = input_data.astype(np.float32)
            input_data = (input_data/255) - 0.5
            interpreter.set_tensor(input_details[0]['index'], input_data)
            interpreter.invoke()
            output_data = interpreter.get_tensor(output_details[0]['index'])
            results = np.squeeze(output_data)
            cenx = results[0]
            ceny = results[1]
            cv.circle(img, (int(cenx), int(ceny)), 1 ,255, -1)
            img = cv.resize(img, (256, 256))
            cv.imwrite("/home/pi/Desktop/Moon.jpg", img)
##      if (Method.get() == Methods[2]):                                        #Experimental code for hough transform
##          print("Using Hough Transform")
##          try:
##              img = cv.imread('/home/pi/Desktop/Moon.jpg')
##              img = cv.medianBlur(img, 5)
##              circles = cv.HoughCircles(cimg, cv.HOUGH_GRADIENT,1,1000,param1=50 ,param2=50,minRadius=0,maxRadius=0)
##              circles = np.uint16(np.around(circles))
##              circ = circles[0][0]
##          except:
##              circ = [32, 32]
##          dx = (circ[0] - 720) * 3474.8 / (2 * circ[2])
##          dy = (circ[1] - 450) * 3474.8 / (2 * circ[2])
##          da = math.atan(dx/384400)
##          de = math.atan(dy/384400)
##          print(da * 180 / 3.14159)
##          print(de * 180 / 3.14159)
##          TargAz = TargAz + (da * 180 / 3.14159)
##          TargEl = TargEl + (de * 180 / 3.14159)
##          cenx = circ[0]
##          ceny = circ[1]
    print (cenx, ceny)
    return cenx, ceny            
        
def GUIUpdate(TargAz, TargEl):                                  #Updates values on GUI
    LatDisplay.configure(text = Lat)
    LongDisplay.configure(text = Long)
    AltDisplay.configure(text = Alt)
    AzDisplay.configure(text = TargAz)
    ElDisplay.configure(text = TargEl)
    AltDisplay.configure(text = Alt)
    image = Image.open("/home/pi/Desktop/Moon.jpg")
    photo = ImageTk.PhotoImage(image)
    picture.configure(image = photo)
    picture.image = photo
        
def close():                                #Asks if user wants to close window
    if messagebox.askokcancel("Quit", "Do you want to quit?"):
        Ser.close()
        root.destroy()

def enter():                                                    #Allows user to manually set Azimuth and Elevation
    global ManAz, ManEl
    ManAz = float(ManAzEntry.get())
    ManEl = float(ManElEntry.get())
    print(ManAz)

def settime():                                                  #Updates the time if user wishes to change it from initial input
    global Obs, Time0, StartTime
    NewTime = TimeEntry.get()
    print(type(NewTime))
    StartTime = datetime.datetime.now() 
    Time0 = NewTime + ':' + '00'
    print(Time0)
    Obs.date = ephem.Date(Time0)

def Loop():
    Recalibrate()                                               #Check location if GPS is connected
    GetPhoto()                                                  #Take Picture
    TargAz, TargEl = GetTarget()                                #Calculate position of selected body
    SendCmd(TargAz, TargEl)                                     #Move to location
    time.sleep(5)
    GUIUpdate(TargAz, TargEl) 
    cx, cy = GetCentre()                                        #If CNN is used, returns position of moon
    x = 0
    while (((abs(cx - 32) > 4) or (abs(cy - 32) > 4)) and x < 3): #Adjusts az and el up to 3 degrees if moon not centred in picture
        if ((cx - 32) > 4):
            TargAz = TargAz + 1
        if ((cx - 32) < 4):
            TargAz = TargAz - 1
        if ((cy - 32) > 4):
            TargEl = TargEl + 1
        if ((cy - 32) < 4):
            TargEl = TargEl - 1
        GUIUpdate(TargAz, TargEl) 
        SendCmd(TargAz, TargEl)
        x = x + 1
        GetPhoto() 
        cx, cy = GetCentre() 
        time.sleep(2)
    GUIUpdate(TargAz, TargEl)                                                 #Update GUI
    root.after(49000, Loop)                                     #Wait 30 seconds and repeat.

#GUI Building -------------------------------------------------------------------------------------
root = tk.Tk()
root.title("Moon Tracker")

#Column 0
LatLabel = tk.Label(root, text = "Latitude:")
LongLabel = tk.Label(root, text = "Longitude:")
AltLabel = tk.Label(root, text = "Altitude:")
AzLabel = tk.Label(root, text = "Target Azimuth")
ElLabel = tk.Label(root, text = "Target Elevation:")

TrackLabel = tk.Label(root, text = "Tracking:")
MethodLabel = tk.Label(root, text = "Detection Method:")
ManAzLabel = tk.Label(root, text = 'Manually Set Azimuth:')
ManElLabel = tk.Label(root, text = 'Manually Set Elevation:')
manual_calibration = tk.IntVar()
c1 = tk.Checkbutton(root, text='Automatically Calibrate Azimuth? Warning - Magnetometer is Inaccurate' , variable=manual_calibration, onvalue=1, offvalue=0)
ChangeTime = tk.Label(root, text = 'Edit Time (YYYY-MM-DD HH:MM):')
                      
LatLabel.grid(row = 0, column = 0)
LongLabel.grid(row = 1, column = 0)
AltLabel.grid(row = 2, column = 0)
AzLabel.grid(row = 3, column = 0)
ElLabel.grid(row = 4, column = 0)
TrackLabel.grid(row = 5, column = 0)
MethodLabel.grid(row = 6, column = 0)
ManAzLabel.grid(row = 7, column = 0)
ManElLabel.grid(row = 8, column = 0)
ChangeTime.grid(row = 9, column = 0)
c1.grid(row = 10, column = 0, columnspan = 3)

#Column 1
LatDisplay = tk.Label(root, text = '0')
LongDisplay = tk.Label(root, text = '0')
AltDisplay = tk.Label(root, text = "0")
AzDisplay = tk.Label(root, text = "0")
ElDisplay = tk.Label(root, text = "0")
Bodies = ["Moon", "Sun", "Manual"]
Track = tk.StringVar()
Track.set(Bodies[0])
TrackMenu = tk.OptionMenu(root, Track, *Bodies)
Methods = ["Lookup Table", "Neural Network (Moon Only)"]
Method = tk.StringVar()
Method.set(Methods[0])
MethodMenu = tk.OptionMenu(root, Method, *Methods)
ManAzEntry = tk.Entry(root)
ManElEntry = tk.Entry(root)
TimeEntry = tk.Entry(root)

LatDisplay.grid(row = 0, column = 1)
LongDisplay.grid(row = 1, column = 1)
AltDisplay.grid(row = 2, column = 1)
AzDisplay.grid(row = 3, column = 1)
ElDisplay.grid(row = 4, column = 1)
TrackMenu.grid(row = 5, column = 1)
MethodMenu.grid(row = 6, column = 1)
ManAzEntry.grid(row = 7, column = 1)
ManElEntry.grid(row = 8, column = 1)
TimeEntry.grid(row = 9, column = 1)

#Column 2
image = Image.open("/home/pi/Desktop/Moon.jpg")
photo = ImageTk.PhotoImage(image)
picture = tk.Label(root, image = photo)
picture.image = photo
EnterButton = tk.Button(root, text = 'Enter Manual Az/El', command = enter)
SetTime = tk.Button(root, text = 'Set Time', command = settime)
                      
picture.grid(row = 0, column = 2, rowspan = 7)
EnterButton.grid(row = 7, column = 2)
SetTime.grid(row = 9, column = 2)
                      

#Setup --------------------------------------------------------------------------------------------
gps = PA1010D()                                         #Setup the gps module
bus = smbus.SMBus(1)                                    #Setup I2C bus
add = 0x1E                                              #Address for magnetometer
yaw = 0                                                 #Initial yaw value
ManAz = 0                                               #Value for Manual Azimuth
ManEl = 0                                               #Value for Manual Elevation

#Send command to rotator to move to zero position
Ser = serial.Serial('/dev/ttyUSB_DEVICE1', 9600, bytesize = 8, timeout=1, stopbits = serial.STOPBITS_ONE, parity=serial.PARITY_NONE)
input("Switch on Rotator Computer Controller and then press enter")
Ser.write(b'\r')
time.sleep(1)
SendCmd(0, 0)
time.sleep(10)
print('Point Antenna North')
yaw = 0

camera = picamera.PiCamera()                                    #Setup pi camera

manual_input = input('Automatically Detect Location? Y/N ')     

if (manual_input == 'Y' or manual_input == 'y'):                #If automatic calibration is requried, read from PA1010D GPS
    result = gps.update()
    while (gps.data['altitude'] is None):
        print('Connecting to GPS')
        result = gps.update()
        time.sleep(1.0)
    Lat = gps.data['latitude']
    Long = gps.data['longitude']
    Alt = gps.data['altitude']
else:                                                           #Else ask for coordinates
    Lat = input('Latitude: ')   
    Long = input('Longitude: ')
    Alt = float(input('Altitude: '))
                       
year = input('Year(YYYY):')                                     
month = input('Month(MM) :')
day = input('Day(DD):')
hour = input('Hour(HH) :')
minute = input('Minute(MM) :')
StartTime = datetime.datetime.now()                             #Record start time for program
Time0 = year + '-' + month + '-' + day + ' ' + hour + ':' + minute + ':' + '00'
Obs = ephem.Observer()                                          #Setup Obs Object
Obs.long = Long
Obs.lat = Lat
Obs.elevation = Alt
Obs.date = ephem.Date(Time0)

cenx = 32                                                       #Centre of Image
ceny = 32

interpreter = tflite.Interpreter(model_path="/home/pi/Desktop/MoonSmall.tflite")    #Setup Neural Network
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
floating_model = input_details[0]['dtype'] == np.float32
height = input_details[0]['shape'][1]
width = input_details[0]['shape'][2]
depth = input_details[0]['shape'][3]

root.after(1000, Loop)                                                              #Begin GUI
root.protocol("WM_DELETE_WINDOW", close)
root.mainloop()
