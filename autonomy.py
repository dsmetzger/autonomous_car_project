import thread
import cv2
import numpy as np
import time
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO

coordinates=[]  #holds the waypoints to navigate to, odd indexs should be end of straight section of sidewalk.

state="drive"  #the current state of the robot.
stage=1  #which section of sidewalk the robot is on. To compenstate for changes in environment. 0=engineering building sidewalk, 1= art building sidewalk.... etc

gps_read_flag=0  #set to 0 while its being written to in the thread
gps_position=[0,0]  #lat,lon

debug_mode=1  #more print statements and saving of pictures may result.

object_detect=0 #thread the sonar

class recognition:
	def __init__(self):
		#Begins camera connection
		self.cap = cv2.VideoCapture(0)
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
	def wh_det(self,x1=10,x2=630,y1=470,y2=10,xit=10,yit=-10,slope_en=0,er_max=4,debug=0):    
		#create green pointers on sidewalk
		count=[]
		tot=0
		for x in range(x1,x2,xit):
			test=er_max#used to ignore noise in the image. if the values goes too low the system stops incrementing in that y direction
			for y in range(y1,y2,yit):
				bgr=self.img[y,x]
				if abs(int(bgr[1])-int(bgr[2]))<14 and ((bgr[0]*1.25)>(bgr[1]+bgr[2])/2):
					tot+=1
					if test<er_max:
		            			test+=1
				else:
					test-=1
					#draw where true
					if test<1:
						for xn in range(-1,2):
							for yn in range(-1,2):
								self.img[y+yn,x+xn]=[0,200,255]
						break
			count.append(tot)
			tot=0
		#calculate slope,
		if slope_en==1:
			slopes=[]
		    	c1=yit/xit
		    	points=6#even multiple of len(count). number of points to evaluate in one loop
			for x1 in range(0,len(count),points):
				ydiff=[]
				for y1 in range(0,points-1):
					ydiff.append((count[x1+y1]-count[x1+y1+1])*c1)
				ysum=0
				#print y1+x1+1
				for y2 in ydiff:
					ysum=y2+ysum
				slopes.append(float(ysum)/len(ydiff))
		if debug==1:
			print counts		
		return count
	
                    

class car:
	def __init__(self):
		#attribute of current speed.
		# Right side motors. Pin definitions for H-bridge control
		self.right_wheel = "P9_14"  # controls speed of right front motor via PWM
		self.right_wheel_dir = "P9_12"   # combine M1 pins on upper and lower motor controllers into a single pin
		# Left side motors. Pin definitions for H-bridge control
		self.left_wheel = "P9_16"  # controls speed of left front motor via PWM
		self.left_wheel_dir = "P8_11"  # combine M2 pins on upper and lower motor controllers into a single pin
		# set pin directions as output
		GPIO.setup(self.right_wheel, GPIO.OUT)
		GPIO.setup(self.right_wheel_dir, GPIO.OUT)
		GPIO.setup(self.left_wheel, GPIO.OUT)
		GPIO.setup(self.left_wheel_dir, GPIO.OUT)

		PWM.start(self.left_wheel, 0)
		PWM.start(self.right_wheel, 0)
	def forward(self):
		GPIO.output(self.right_wheel_dir, GPIO.HIGH)
		GPIO.output(self.left_wheel_dir, GPIO.HIGH)
	def speed(self, duty=45, offset=0):
		#changes the speed of individual motors to a specified inputs using PWM.
		PWM.set_duty_cycle(self.left_wheel, duty+offset)
		PWM.set_duty_cycle(self.right_wheel, duty-offset)
	def stop(self):
		PWM.stop(self.right_wheel)
		PWM.stop(self.left_wheel)
		PWM.cleanup()

if __name__ == "__main__":
	#create regocnition, GPS, and car instance.
	rec=recognition()
	car1=car()
	
	#Thread gps sonar and compass
	time.sleep(3)
	while True:
		if state=='drive':
			if stage==0 or stage==1 or stage==2:
				#follow compass while checking if path edges are too close.
				it=0
				start = time.time()
				car1.forward()
				while True:
					#perform white detection
					rec.get_img()
					l1=rec.wh_det(x1=10,x2=310,y1=235,y2=100,xit=5, yit=5, debug=1)
					#check sonar. if true change behavior and break
					#check gps for change to turn state
					#if too close turn away from edge
					
					#first algorithim, follow most pixels
					sum1=0
					sum2=0
					for x in range(0, len(l1)):
						if x<len(l1)/2:
							sum1+=l1[x]
						else:
							sum2+=l1[x]
					offset=30*(.5-(float(sum1+1)/(sum1+sum2+2)))#gain times error
					print 'left'+str(sum1)
					print 'right'+str(sum2)
					print 'offset'+str(offset)
					car1.speed(55, offset)
					it+=1
					end=time.time()
					if end-start>20:
						car1.stop()
						print 'stage '+str(stage)+' performed at '+str(it/(end-start))+' hertz'
						exit()
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
