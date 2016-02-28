import thread
import cv2
import numpy as np
import time


global coordinates=[]  #holds the waypoints to navigate to, odd indexs should be end of straight section of sidewalk.

global state="drive"  #the current state of the robot.
global stage=0  #which section of sidewalk the robot is on. To compenstate for changes in environment. 0=engineering building sidewalk, 1= art building sidewalk.... etc

global gps_read_flag=0  #set to 0 while its being written to in the thread
global gps_position=[0,0]  #lat,lon

global control_system_reset=0  #if behavior changes the control systems must be reset.

global debug_mode=1  #more print statements and saving of pictures may result.

global object_detect=0 #thread the sonar

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
                    

class car:
	def __init__(self):
		#attribute of current speed.
	def speed(self):
		#changes the speed of individual motors to a specified inputs using PWM.
	def stop(self):
		#called in emergency if the sensor activates at a close proximity.


if __name__ == "__main__":
	#create regocnition, GPS, and car instance.
	rec=recognition()
	
	#Thread gps
	while True:
		if state=='drive':
			if stage==0 or stage==1 or stage==2:
				#follow compass while checking if path edges are too close.
				while True:
					#perform white detection
					rec.get_img()
					l1=rec.wh_det()
					#check sonar. if true change behavior and break
					#check gps for change to turn state
					#if too close turn away from edge
					
			pass
		elif state=='turn':
			if stage==0:
				#the first turn is circular. choose a starting distance from edge of sidewalk.
				#use picture region of interest around the edge of the sidewalk and PID control. The camera will update quick enough for stability.
				#if edge of sidewalk is lst use compass to turn towards exit of turn(direction of sidewalk paralell to art building). 
				pass
			if stage==1:
				#travel paralell to art building. stop when close to sidewalk, follow sidewalk then look for turn left into forested area
				pass
			if stage==2:
				#keep moving until the edge of the camera detects white.
				#when detected, turn in a constant circle(radius of about .6 m) until the edge of the side walk is detected within a specified distance. orientate and drive.
				pass
		elif state=='ob_det':
			pass
