import thread
import cv2
import numpy as np
import time


global coordinates=[]#holds the waypoints to navigate to, odd indexs should be end of straight section of sidewalk.
global state="drive"#the current state of the robot.
global stage=0#which section of sidewalk the robot is on. To compenstate for changes in environment.

class recognition:
	def __init__(self):
		#Begins camera connection
		self.cap = cv2.VideoCapture(1)
	def get_img(self):
		#update current image.
		while 1:
			ret, frame = self.cap.read()
			if ret==True:#check if cap.read() returned frame
				self.img=frame
				break
	def get_lines(self):
		#function that returns a series of lines in the current image. Possibly using an input region of interest of the image.
		pass
	def wh_det(img,x1=10,x2=640,y1=470,y2=0,xit=10,yit=-10):
		tot=0;
		l1=[]
		for x in range(x1,x2,xit):
			test=3#used to ignore noise in the image. if the values goes too low the system stops incrementing in that y direction
			for y in range(y1,y2,yit):
				bgr=self.img[y,x]
				if abs(int(bgr[1])-int(bgr[2]))<14 and ((bgr[0]*1.25)>(bgr[1]+bgr[2])/2):#works
					tot+=1
					if test<3:
						test+=1
					else:
						test-=1
					if test<2:
						break
			l1.append(ytot)
		return l1
                    

class GPS:
	def __init__(self):
		#start the gps connection
		#attribute of current gps position, current speed, and estimated direction vector of cumulative gps positions.
		u = mraa.Uart(0)
		path=u.getDevicePath()
.		self.f=open(path,'r')
	def read_data(self,data_str='$GPGGA',iterations=50):
		#wait for optional checksum and specified identifier, return data.
		for i in range(iterations):
			nmea=self.f.readline()
			segmented_data=nmea.split(',')
			if segmented_data[0]==data_str: #and checksum(data):
				return segmented_data,i
			else:
				return False,i
	def read_position_data(self,iterations=50):
		#calls read_data() for the position nmea, updates stage
		segmented_data,i=self.read_data('$GPGGA',iterations)
		#6th position 0=no fix, 1=gps, 2=dgps
		if segmented_data and segmented_data[6]!=0:
			#return lat, lon
			return segmented_data[2:6]
		else:
			return self.read_position_data((iterations-i-1))
	def read_utc_data(self,iterations=50):
		#calls read_data() for the utc nmea
		data=self.read_data('$GPZDA',iterations)
		segmented_data=data.split(',')
		print segmented_data
		return segmented_data
	def write_commands(self):
		pass
		#I thought I read you can change certain nmea outputs but havnt played around with it yet.

class car:
	def __init__(self):
		#attribute of current speed.
	def speed(self):
		#changes the speed of individual motors to a specified inputs using PWM.
	def stop(self):
		#called in emergency if the sensor activates at a close proximity.


def __main__()
	#create regocnition object, GPS object, and car instance.

	#continous funcion that uses the gps vector attributes and a camera to stay on sidewalk.
	while True:
		#use gps to determine if turn is coming up.
		#possibly use eigenvectors or some other vector algorithim to determine stage.
		#check sonar.
		#stage will be constant at 0 for now
		if stage==0:
			if state=='drive':
				#control system to keep vehicle close to right side of sidewalk.		
				pass	
			elif state=='turn':
				#keep moving until the edge of the camera detects white
				#when detected, turn in a constant circle until the edge of the side walk is detected within a specified distance. orientate and drive.
				pass
			elif state=='ob_det':				
				pass
