import thread
import cv2
import numpy as np
import time
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
import math
import serial
import Adafruit_BBIO.UART as UART
import smbus
import sys
import os
#imu
from Adafruit_BNO055 import BNO055

#gps
UART.setup("UART4")
gps1 = serial.Serial("/dev/ttyO4", 4800)
coordinates=[[3849.669975,7718.3229125],[3849.6589066667,7718.3471666667],[3849.7127, 7718.3901],[0.0,0.0],[0.0,0.0]]  #holds the waypoints to navigate to of signifigant turns. first waypoint is starting point
gps_position=coordinates[0] #lat,lon

state="test"  #the current state of the robot.
stage=2  #which section of sidewalk the robot is on. To compenstate for changes in environment. 0=engineering building sidewalk, 1= art building sidewalk.... etc


object_detect=0 #thread the sonar
heading =0.0
incline =0.0
inc_offset=-2.0#set at certain stages
yaw_rate=0.0 #radians per sec
pwm1=0.0
yaw=0.0

#camera
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
	def wh_det(self,x1=0,x2=640,y1=470,y2=10,xit=10,yit=-10,slope_en=0,er_max=4,debug=0):    
		#create green pointers on sidewalk
		count=[]
		tot=0
		for x in range(x1,x2,xit):
			test=er_max#used to ignore noise in the image. if the values goes too low the system stops incrementing in that y direction
			for y in range(y1,y2,yit):
				bgr=self.img[y,x]
                                #if abs(int(bgr[1])-int(bgr[2]))<14 and int(bgr[0])>85 and ((bgr[0]*1.25)>(bgr[1]+bgr[2])/2):
				#if abs(int(bgr[1])-int(bgr[2]))<14 and ((int(bgr[0])+int(bgr[1])+int(bgr[2]))/3)>80 and ((bgr[0]*1.25)>(bgr[1]+bgr[2])/2):
				#if abs(int(bgr[1])-int(bgr[2]))<10 and int(bgr[0])>50 and ((bgr[0]*1.25)>(bgr[1]+bgr[2])/2):
				#if abs(int(bgr[1])-int(bgr[2]))<14 and int(bgr[0])>90 and ((bgr[0]*1.10)>(bgr[1]+bgr[2])/2):
				if abs(int(bgr[1])-int(bgr[2]))<14 and int(bgr[0])>110 and ((bgr[0]*1.10)>(bgr[1]+bgr[2])/2):
					tot+=1
					if test<er_max:
		            			test+=1
				else:
					test-=1
					#draw where true
					if test<1:
						break
			count.append(tot)
			tot=0
		#calculate slope,	
		if slope_en==1:
			slopes=[]
			c1=float(yit)/xit
			for x1 in range(0,len(count)-1):
			    slopes.append(float(count[x1]-count[x1+1])*c1)   
			return count,slopes
		return count
	def wh_det_horizontal(self,x1=320,x2=640,y1=470,y2=200,xit=10,yit=-10,er_max=2):    
		count=[]#right side
		tot=0
		for y in range(y1,y2,yit):
			test=er_max
			for x in range(x1,x2,xit):
				bgr=self.img[y,x]
				#if abs(int(bgr[1])-int(bgr[2]))<18 and ((int(bgr[0])+int(bgr[1])+int(bgr[2]))/3)>80:
				#if abs(int(bgr[1])-int(bgr[2]))<14 and ((bgr[0]*1.25)>(bgr[1]+bgr[2])/2):
				if abs(int(bgr[1])-int(bgr[2]))<14 and int(bgr[0])>90 and ((bgr[0]*1.10)>(bgr[1]+bgr[2])/2):
					tot+=1
					if test<er_max:
			    			test+=1
				else:
					test-=1
					#draw where true
					if test<1:
						break
			count.append(tot)
			tot=0
		return count
	def sample(self,x1=160,x2=161,y1=200,y2=130,xit=10,yit=-10,slope_en=0,er_max=4,debug=0):
		count=[]
		tot=0
		for x in range(x1,x2,xit):
			for y in range(y1,y2,yit):
				bgr=self.img[y,x]
				print 'g,r differe '+str(abs(int(bgr[1])-int(bgr[2])))+' <  14'
				print 'b magnitude '+str(int(bgr[0]))+' >  80'
				print '1*b > |g,r|   '+str(((bgr[0]*1.)))+' > '+str(float(bgr[1]+bgr[2])/2)
class car:
	def __init__(self):
		# Right side motors. Pin definitions for H-bridge control
		self.right_wheel = "P9_16"  # controls speed of right front motor via PWM
		self.right_wheel_dir = "P9_12"   # combine M1 pins on upper and lower motor controllers into a single pin
		# Left side motors. Pin definitions for H-bridge control
		self.left_wheel = "P9_14"  # controls speed of left front motor via PWM
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
	def backward(self):
		GPIO.output(self.right_wheel_dir, GPIO.LOW)
		GPIO.output(self.left_wheel_dir, GPIO.LOW)
	def spin_left(self):
		GPIO.output(self.right_wheel_dir, GPIO.HIGH)
		GPIO.output(self.left_wheel_dir, GPIO.LOW)
	def spin_right(self):
		GPIO.output(self.right_wheel_dir, GPIO.LOW)
		GPIO.output(self.left_wheel_dir, GPIO.HIGH)
	def speed(self, duty=55, offset=0):
		print 'duty ',duty
		if duty>83 or duty<-20:
			print 'bad pwm'
			return
		else:
			print '  '
		#check if negative
		l_speed=duty+offset
		r_speed=duty-offset
		if l_speed<0:#negative speed, reverse
			GPIO.output(self.left_wheel_dir, GPIO.LOW)
			if l_speed<-20:
				l_speed=-20
		elif l_speed>85:#check if speed too high
			l_speed=85
			GPIO.output(self.left_wheel_dir, GPIO.HIGH)
		else:
			GPIO.output(self.left_wheel_dir, GPIO.HIGH)
		if r_speed<0:#negative speed, reverse
			GPIO.output(self.right_wheel_dir, GPIO.LOW)
			if r_speed<-20:
				r_speed=-20
		elif r_speed>85:#check if speed too high
			r_speed=85
			GPIO.output(self.right_wheel_dir, GPIO.HIGH)
		else:
			GPIO.output(self.right_wheel_dir, GPIO.HIGH)
		PWM.set_duty_cycle(self.left_wheel, abs(l_speed))
		PWM.set_duty_cycle(self.right_wheel, abs(r_speed))
	def stop(self):
		PWM.stop(self.right_wheel)
		PWM.stop(self.left_wheel)
		PWM.cleanup()

class PID:
	def __init__(self, P=12.0, I=15.0, D=3.0,I_lim=4):
		self.Kp=P
		self.Ki=I
		self.Kd=D
		self.error_prev=0.0
		self.Il=[0.0]
		for x in range(0,I_lim):
			self.Il.append(0.0)
	def update(self,error):
		#calculate output
		I=0
		for x in self.Il:
			I+=x
		I=I/len(self.Il)
		for x in range(0,len(self.Il)-1):
			self.Il[x]=self.Il[x+1]
		self.Il[-1]=error
		P=self.Kp*error
		I=self.Ki*I
		D=self.Kd*(error-self.error_prev)
		offset=P+I+D
		self.error_prev=error
		print 'P ',P
		print 'I ',I
		print 'D ',D
		return offset

def get_location():
	global gps_position
	while True:
		gps1.open()
		data1 = gps1.readline()
		gps1.close()
		data_array1 = data1.split(',')
	
		if data_array1[0]=='$GPGGA':
			if data_array1[6]!=0:
				gps_position[0] = float(data_array1[2])
				gps_position[1] = float(data_array1[4])
		time.sleep(.01)

def follow_most_pixels(xit,yit,x1=20, x2=300, y1=150, y2=100, inp=.5,alg=0):
	sum1=0
	sum2=0
	if alg==0:#first algorithim, follow most pixels
		l1=rec.wh_det(x1,x2,y1,y2,xit=xit, yit=yit,er_max=4,slope_en=0)
		print l1
		for x in range(0, len(l1)):
			if x<len(l1)/2:
				sum1+=l1[x]	
			else:
				sum2+=l1[x]
		error=(inp-(float(sum1+1)/(sum1+sum2+2)))
	elif alg==1:
		#follow right side
		l1=rec.wh_det(x1=280,x2=320,y1=215,y2=100,xit=xit, yit=yit,er_max=2,slope_en=0)
		print l1
		for x in range(0, len(l1)):
			if l1[x]<14:#14
				sum1+=1	
		sum2=len(l1)
		error=(inp-(float(sum1)/(sum2)))
	elif alg==2:
		#follow left side
		l1=rec.wh_det(x1=0,x2=40,y1=215,y2=100,xit=xit, yit=yit,er_max=2,slope_en=0)
		print l1
		for x in range(0, len(l1)):
			if l1[x]<14:#14
				sum2+=1
		sum1=len(l1)
		error=(inp-(float(sum2)/(sum1)))
	elif alg==3:
		#follow most matching pixels with emphasis on high values
		l1=rec.wh_det(x1=0,x2=320,y1=140,y2=30,xit=xit, yit=yit,er_max=2,slope_en=0)
		print l1
		for x in range(0, len(l1)):
			if x<len(l1)/2:
				sum1+=(l1[x])**1.3#exponent can be increased	
			else:
				sum2+=(l1[x])**1.3
		error=(inp-(float(sum1+1)/(sum1+sum2+2)))
	print 'sum1 ',sum1
	print 'sum2 ',sum2
	return error

def control_distance(xit,yit,slope,b,side=1,dist=40):#side=1, right of sidewalk
	if side==1:
		y1=241
		y2=80
		inc_max=abs((y2-y1)/yit)
		l1,slopes=rec.wh_det(x1=305,x2=320,y1=y1,y2=y2,xit=xit, yit=yit,er_max=3,slope_en=1)
		print l1,slopes
		#offset from slope, positive for turn right
		it=0
		tot=0
		for x in range(0, len(l1)-1):
			if l1[x]< (inc_max-2):
				tot+=slopes[x]
				it+=1
		#create line equation		
		if it<4:
			print 'condition 1'
			slope_avg=slope-.1
			b_avg=b+10
		else:
			slope1=tot/it
			slope_avg=.1*slope1+.9*slope
			b1=(((l1[-1]+l1[-2])/2)*yit)-slope*160
			b_avg=.1*b1+.9*b
		#intersection points
		x1=-b_avg/(slope_avg+(1/slope_avg))
		y1=x1*slope_avg+b_avg
		print 'intersection points'
		print x1,y1
		print 'slope,b\n',slope_avg,b_avg
		dist_to_sw=math.sqrt(x1**2+y1**2)
		#dist_to_sw=(-b)/slope
		print 'distance to sidewalk, '+str(dist_to_sw)+' pixels'
		error=(dist_to_sw-dist)
		return error,slope_avg,b_avg
	else:
		pass
def get_direction(t_compass=.2,t_gyro=.15):
	data=[26, 0, 254, 255, 33, 0, 159, 253, 217, 0, 244, 255, 255, 255, 0, 0, 1, 0, 232, 3, 88, 1]
	bno = BNO055.BNO055(rst='P8_4')
	bno.begin()
	bno.set_calibration(data)
	status, self_test, error = bno.get_system_status()
	print('System status: {0}'.format(status))
	print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
	# Read the Euler angles for heading, roll, pitch (all in degrees).
	heading, roll, pitch = bno.read_euler()
	# Read the calibration status, 0=uncalibrated and 3=fully calibrated.
	sys, gyro, accel, mag = bno.get_calibration_status()
	# Print everything out.
	print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(heading, roll, pitch, sys, gyro, accel, mag))
	global heading
	global incline
	global yaw_rate
	global yaw
	end1=0.0
	end2=0.0
	while True:
		#get compass heading
		x,y,z = bno.read_magnetometer()
		t_diff=time.time()-end1
		end1=time.time()
		h1  = math.atan2(y, x)
		#average compass results
		x1=math.cos(h1)
		x2=math.cos(math.radians(heading))
		y1=math.sin(h1)
		y2=math.sin(math.radians(heading))
		if t_diff>t_compass:
			x3=((t_diff)*(x1)+x2*(t_compass-t_diff))/t_compass
			y3=((t_diff)*(y1)+y2*(t_compass-t_diff))/t_compass
		else:
			x3=x1
			y3=y1
		h1=math.atan2(y3,x3)
		if (h1 < 0):
		        h1 += 2 * math.pi
		heading=math.degrees(h1)
		#gyro
		x,y,yaw_rate_new=bno.read_gyroscope()#angular velocity (degrees/sec)
		#print 'l_accel,', bno.read_linear_acceleration()
		pitch, incline1, yaw1 = bno.read_euler()
		t_diff=time.time()-end2
		end2=time.time()
		if t_diff>t_gyro:
			incline=((t_diff)*(incline1)+incline*(t_gyro-t_diff))/t_gyro
			yaw_rate=((t_diff)*(yaw_rate_new*(-180/math.pi))+yaw_rate*(t_gyro-t_diff))/t_gyro
			yaw=((t_diff)*(yaw1*(-180/math.pi))+yaw*(t_gyro-t_diff))/t_gyro
		else:
			incline=(incline1)
			yaw_rate=yaw_rate_new
			yaw=yaw1
		time.sleep(.0001)
		#time.sleep(.09)
def set_speed():
	global pwm1
	while True:
		#force_paralell=50*math.sin(math.radians(incline))
		#f_friction=50*.5*math.cos(math.radians(incline))
		fixed_incline=incline-inc_offset
		pwm1=340*(math.sin(math.radians(fixed_incline))+.15*math.cos(math.radians(fixed_incline)))
		#print 'incline ',incline
		#print 'F_para ',force_paralell
		#print 'F_fric ',f_friction
		time.sleep(.05)
def gps_check(destination=coordinates[1]):
	print 'current pos ',gps_position
	print 'check pos ',destination
        if math.sqrt((gps_position[0]-destination[0])**2+(gps_position[1]-destination[1])**2)< .0066:#.00641098:#less than two stdev
                return True
        else:
                return False
def follow_edge(alg=0):
	if alg==0:#right side
		l1=rec.wh_det(x1=235,x2=315,y1=101,y2=100,xit=5,yit=-20,er_max=4)
		print 'list 1', l1
		sum1=0
		for x in range(0,len(l1)):
			sum1+=l1[x]
		e1=float(sum1)/len(l1)-.5
		return e1
	elif alg==1:#left side
		l1=rec.wh_det(x1=5,x2=85,y1=101,y2=100,xit=5,yit=-20,er_max=4)
		print 'list 1', l1
		sum1=0
		for x in range(0,len(l1)):
			sum1+=l1[x]
		e1=float(sum1)/len(l1)-.5
		return -e1
	elif alg==2:#follow right side using list-- works--
		#error based on distance and predicted distance
		#l1=rec.wh_det(x1=200,x2=320,y1=151,y2=110,xit=10,yit=-20,er_max=1)
		#l1=rec.wh_det(x1=240,x2=320,y1=151,y2=150,xit=10,yit=-20,er_max=4)
		l2=rec.wh_det(x1=235,x2=315,y1=101,y2=100,xit=5,yit=-20,er_max=4)
		#print 'list 1', l1
		print 'list 2', l2
		#distance error
		#sum1=0
		#sum2=0
		#for x in range(0,len(l1)):
		#	sum1+=l1[x]
			#if x<len(l1)/2:
			#	sum1+=l1[x]
			#else:
			#	sum2+=l1[x]
		#e1=.5-(sum1+1)/(sum1+sum2+2)
		#e1=float(sum1)/len(l1)-.5
		sum1=0
		#sum2=0
		for x in range(0,len(l2)):
			sum1+=l2[x]
		#	if x<len(l2)/2:
		#		sum1+=l2[x]
		#	else:
		#		sum2+=l2[x]
		#e2=.5-(sum1+1)/(sum1+sum2+1)
		e2=float(sum1)/len(l2)-.5
		error=(e2)
		#error=(.5*e1+.5*e2)
		return error
def edge_check():
	l1=rec.wh_det(x1=100,x2=221,y1=220,y2=150,xit=40, yit=-5,er_max=2,slope_en=0)
	print 'edge_check list', l1					
	if l1[-1]<9 or l1[-2]<9 or l1[-3]<9:
		print 'edge detected'
		return 1
	else:
		print '  '
		return 0
def turn_speed_check(offset,turn_limit=18):
	print 'yaw_rate ',yaw_rate
	if (offset>0 and yaw_rate>turn_limit) or (offset<0 and yaw_rate<(-turn_limit)):
		print 'turn speed limited'
		return 0
	else:
		print '    '
		return offset
def compass_check(offset,inc_lookups, compass_lookups,use_incline=0):
	for x in range(0, len(inc_lookups)):
		inc_lookup=inc_lookups[x]
		compass_lookup=compass_lookups[x]
		x1=math.cos(inc_lookup)
		x2=math.cos(math.radians(compass_lookup))
		y1=math.sin(inc_lookup)
		y2=math.sin(math.radians(compass_lookup))
		diff=0
		if use_incline==1:
			if abs(inc_lookup-incline)<.7:
				if compass_lookup-heading<-5:
					print 'comp cond 1 at ',heading,',',incline
					return 1
		else:
			if abs(compass_lookup-heading)<5:
				print 'comp cond 2 at ',heading
				return 1
	print ' '
	return offset

def compass_check_s(offset, compass_lookup):
	x1=math.cos(heading)
	x2=math.cos(math.radians(compass_lookup))
	y1=math.sin(heading)
	y2=math.sin(math.radians(compass_lookup))
	diff=math.atan2((y1-y2),(x1-x2))*180
	print 'heading off by ',diff
	if diff>5 and offset>0:
		return 0
	elif diff<-5 and offset<0:
		return 0
	else:
		return offset

if __name__ == "__main__":
	#create regocnition, GPS, and car instance.
	time.sleep(7)
	rec=recognition()
	rec.get_img()
	rec.sample()
	car1=car()
	#Thread gps sonar and compass
	#thread.start_new_thread(get_location, ())
	thread.start_new_thread(get_direction, ())
	thread.start_new_thread(set_speed, ())
	#while True:
	#	print yaw_rate
	#	print heading
	#	pass
	time.sleep(1)
	while True:
		if state=='drive':
			if stage==0:
				it=0
				start = time.time()
				end1=start
				car1.forward()
				#I=0
				#pid1=PID(P=10,I=15,D=0)
				pid1=PID(P=.8,I=1.7,D=.4)
				vel_in=0.0
				while True:
					print '------'+state+' '+str(stage)+' ---iteration '+str(it)+' ---------'
					rec.get_img()
					#check sonar. if true change behavior and break
					err=follow_edge(alg=2)
					print 'err ',err
					vel_in=.9*vel_in+3*err#-.5 to .5, degrees/sec should be around 30 (.1*2*30) at its highest
					error=(vel_in-yaw_rate)
					print 'error ', error
					offset=pid1.update(error)
					it+=1
					print 'abs incline ', str(incline-inc_offset)
					print 'heading', heading
					car1.speed(pwm1, offset)
					end1=time.time()
					if (end1-start)>180 or gps_check(coordinates[1]):
                                                print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						global state
						state="turn"
						break
			elif stage==2:
				it=0
				start = time.time()
				end1=start
				car1.forward()
				#I=0
				#pid1=PID(P=10,I=15,D=0)
				pid1=PID(P=.8,I=1.7,D=.4)
				vel_in=0.0
				while True:
					print '------'+state+' '+str(stage)+' ---iteration '+str(it)+' ---------'
					rec.get_img()
					#check sonar. if true change behavior and break
					err=follow_edge(alg=1)
					print 'err ',err
					vel_in=.9*vel_in+3*err#-.5 to .5, degrees/sec should be around 30 (.1*2*30) at its highest
					error=(vel_in-yaw_rate)
					print 'error ', error
					offset=pid1.update(error)
					it+=1
					print 'abs incline ', str(incline-inc_offset)
					print 'heading', heading
					car1.speed(pwm1, offset)
					end1=time.time()
					if (end1-start)>180 or gps_check(coordinates[1]):
                                                print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						global state
						state="turn"
						break
			elif stage==97:#control car using new edge follower, use control system for angular velocity
				it=0
				start = time.time()
				end1=start
				car1.forward()
				#I=0
				#pid1=PID(P=10,I=15,D=0)
				pid1=PID(P=.8,I=1.7,D=.4)
				vel_in=0.0
				while True:
					print '------'+state+' '+str(stage)+' ---iteration '+str(it)+' ---------'
					rec.get_img()
					#check sonar. if true change behavior and break
					err=follow_edge(alg=2)
					print 'err ',err
					vel_in=.9*vel_in+2*err#-.5 to .5, degrees/sec should be around 30 (.1*2*30) at its highest
					error=(vel_in-yaw_rate)
					print 'error ', error
					offset=pid1.update(error)
					it+=1
					print 'abs incline ', str(incline-inc_offset)
					print 'heading', heading
					if abs(incline-inc_offset-2.4)<1.5:
						if heading>107 and heading < 360:
							if offset>0:
								offset=0
						elif heading <97 and heading > 0:
							if offset<0:
								offset=0
					car1.speed(pwm1, offset)
					end1=time.time()
					if (end1-start)>1500:#gps_check(coordinates[1])
                                                print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						global state
						state="turn"
						break
			elif stage==98:#control car using new edge follower, use control system for angular velocity
				it=0
				start = time.time()
				end1=start
				car1.forward()
				#I=0
				#pid1=PID(P=10,I=15,D=0)
				pid1=PID(P=.7,I=.7,D=.1)
				vel_in=0.0
				h5=175
				while True:
					print '------'+state+' '+str(stage)+' ---iteration '+str(it)+' ---------'
					rec.get_img()
					#check sonar. if true change behavior and break
					err=follow_edge(alg=1)
					print 'err ',err
					vel_in=.9*vel_in+1.2*err#-.5 to .5, degrees/sec should be around 30 (.1*2*30) at its highest
					error=(vel_in-yaw_rate)
					print 'error ', error
					offset=pid1.update(error)
					it+=1
					print 'abs incline ', str(incline-inc_offset)
					print 'yaw ', yaw
					print 'heading', heading
					'''if heading>h5+6 and heading < 360:
						if offset>0:
							offset=0
					elif heading <h5-6 and heading > 0:
						if offset<0:
							offset=0'''
					car1.speed(pwm1+5, offset)
					end1=time.time()
					if (end1-start)>1500:#gps_check(coordinates[1])
                                                print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						global state
						state="turn"
						break			
		elif state=='turn':
			if stage==0:
				it=0
				start = time.time()
				end1=start
				car1.forward()
				#I=0
				#pid1=PID(P=10,I=15,D=0)
				pid1=PID(P=.8,I=1.7,D=.4)
				vel_in=0.0
				while True:
					print '------'+state+' '+str(stage)+' ---iteration '+str(it)+' ---------'
					rec.get_img()
					#check sonar. if true change behavior and break
					err=follow_edge(alg=2)
					print 'err ',err
					vel_in=.9*vel_in+6.0*err#-.5 to .5, degrees/sec should be around 30 (.1*2*30) at its highest
					error=(vel_in-yaw_rate)
					print 'error ', error
					offset=pid1.update(error)
					it+=1
					print 'abs incline ', str(incline-inc_offset)
					print 'heading', heading
					car1.speed(pwm1, offset)
					end1=time.time()
					if (end1-start)>14000:# or gps_check(coordinates[1]):
                                                print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						global state
						state="turn"
						break	
			elif stage==1:
				it=0
				start = time.time()
				end1=start
				car1.forward()
				substage=0
				pid1=PID(P=.8,I=1.7,D=.4)
				vel_in=0.0
				#substage 1
				kept_yaw=yaw
				turn_wait=5
				while True:
					print '------'+state+' '+str(stage)+' ---iteration '+str(it)+' --substage '+str(substage)+'-------'
					rec.get_img()
					#check sonar. if true change behavior and break
					if substage==0:
						pwm_sub=pwm1
						substage=1
					elif substage==1:#follow edge
						pwm_sub=pwm1
						err=follow_edge(alg=1)
						print 'err ',err
						vel_in=.9*vel_in+3*err#-.5 to .5, degrees/sec should be around 30 (.1*2*30) at its highest
						error=(vel_in-yaw_rate)
						print 'error ', error
						offset=pid1.update(error)
						kept_yaw=.95*kept_yaw+.05*yaw
						c1=rec.wh_det_horizontal(x1=85,x2=5,y1=101,y2=100,xit=-5,yit=-20,er_max=3)
						print 'kept yaw', kept_yaw
						print 'horizontal check',c1
						if c1>14:
							substage+=1
							sub2_end=time.time()
							vel_in=0
							pid1=PID(P=.8,I=1.7,D=.4)
					elif substage==2:#go straight for certain amount of time
						pwm_sub=pwm1
						err=kept_yaw-yaw
						print 'err ',err
						vel_in=.9*vel_in+3*err#-.5 to .5, degrees/sec should be around 30 (.1*2*30) at its highest
						error=(vel_in-yaw_rate)
						print 'error ', error
						offset=pid1.update(error)
						if incline<.2:
							pwm_sub=0
							substage+=1
							vel_in=0
							pid1=PID(P=.8,I=1.7,D=.4)
					elif substage==3:#face compass heading
						pwm_sub=pwm1
						err=('''270'''-heading)/90
						print 'err ',err
						vel_in=.9*vel_in+3*err#-.5 to .5, degrees/sec should be around 30 (.1*2*30) at its highest
						error=(vel_in-yaw_rate)
						print 'error ', error
						offset=pid1.update(error)
						if incline<'''.2''':
							pwm_sub=40
							substage+=1
							vel_in=0
							pid1=PID(P=.8,I=1.7,D=.4)
					elif substage==4:
						time.sleep(1)
						car1.forward()
						pwm_sub=pwm1
					it+=1
					print 'abs incline ', str(incline-inc_offset)
					print 'heading', heading
					car1.speed(pwm_sub, offset)
					end1=time.time()
					if substage==5:
                                                print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						global state
						state="drive"
						global stage
						stage=2
						break			
			elif stage==2:
				it=0
				start = time.time()
				end1=start
				car1.forward()
				substage=1
				pid1=PID(P=.8,I=1.7,D=.4)
				vel_in=0.0
				#substage 1
				kept_yaw=yaw
				turn_wait=5
				while True:
					print '------'+state+' '+str(stage)+' ---iteration '+str(it)+' --substage '+str(substage)+'-------'
					rec.get_img()
					#check sonar. if true change behavior and break
					if substage==0:
						pwm_sub=pwm1
						substage=1
					elif substage==1:
						pwm_sub=pwm1
						err=follow_edge(alg=1)
						print 'err ',err
						vel_in=.9*vel_in+3*err#-.5 to .5, degrees/sec should be around 30 (.1*2*30) at its highest
						error=(vel_in-yaw_rate)
						print 'error ', error
						offset=pid1.update(error)
						kept_yaw=.95*kept_yaw+.05*yaw
						c1=rec.wh_det_horizontal(x1=85,x2=5,y1=101,y2=100,xit=-5,yit=-20,er_max=3)
						print 'kept yaw', kept_yaw
						print 'horizontal check',c1
						if c1>14:
							substage+=1
							sub2_end=time.time()
							vel_in=0
							pid1=PID(P=.8,I=1.7,D=.4)
					elif substage==2:#go straight for certain amount of time
						pwm_sub=pwm1
						err=kept_yaw-yaw
						print 'err ',err
						vel_in=.9*vel_in+3*err#-.5 to .5, degrees/sec should be around 30 (.1*2*30) at its highest
						error=(vel_in-yaw_rate)
						print 'error ', error
						offset=pid1.update(error)
						if time.time()-sub2_end > turn_wait:
							pwm_sub=0
							substage+=1
							#vel_in=0
							#pid1=PID(P=.8,I=1.7,D=.4)
					elif substage==3:#face compass heading
						offset=0
						pwm_sub=40
						car1.spin_left
						if heading <220:
							pwm_sub=20
						if heading <200:
							substage+=1
							pwm_sub=pwm1
					elif substage==4:
						time.sleep(1)
						car1.forward()
						pwm_sub=pwm1
					it+=1
					print 'abs incline ', str(incline-inc_offset)
					print 'heading', heading
					car1.speed(pwm_sub, offset)
					end1=time.time()
					if substage==5:
                                                print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						global state
						state="drive"
						global stage
						stage=2
						break
		elif state=='ob_det':
			pass
		elif state=='test':
			if stage==0:
				it=0
				start = time.time()
				end1=start
				car1.forward()
				#I=0
				#pid1=PID(P=10,I=15,D=0)
				pid1=PID(P=14,I=15,D=2)
				while True:
					print '------'+state+' '+str(stage)+' ---iteration '+str(it)+' ---------'
					rec.get_img()
					#check sonar. if true change behavior and break
					#error=follow_edge(alg=1)
					error=follow_most_pixels(10,-7,alg=1)
					
					print 'error ', error
					offset=pid1.update(error)
					it+=1
					print 'abs incline ', str(incline-inc_offset)
					print 'heading', heading
					car1.speed(pwm1, offset)
					end1=time.time()
					if (end1-start)>2600:#gps_check(coordinates[1])
                                                print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						global state
						state="turn"
						break
			elif stage==1:#control car using new edge follower, use control system for angular velocity
				it=0
				start = time.time()
				end1=start
				car1.forward()
				#I=0
				#pid1=PID(P=10,I=15,D=0)
				pid1=PID(P=.8,I=1.7,D=.4)
				vel_in=0.0
				while True:
					print '------'+state+' '+str(stage)+' ---iteration '+str(it)+' ---------'
					rec.get_img()
					#check sonar. if true change behavior and break
					err=follow_edge(alg=2)
					print 'err ',err
					vel_in=.9*vel_in+3*err#-.5 to .5, degrees/sec should be around 30 (.1*2*30) at its highest
					error=(vel_in-yaw_rate)
					print 'error ', error
					offset=pid1.update(error)
					it+=1
					print 'abs incline ', str(incline-inc_offset)
					print 'heading', heading
					if incline-inc_offset>1:
						if heading>355 or (heading>0 and heading<100):
							if offset>0:
								offset=0
					car1.speed(pwm1-5, offset)
					end1=time.time()
					if (end1-start)>2600:#gps_check(coordinates[1])
                                                print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						global state
						state="turn"
						break
			elif stage==2:#follow most pixels #innovation building straight path
				it=0
				start = time.time()
				end1=start
				car1.forward()
				pid1=PID(P=1.2,I=.7,D=0.0)
				vel_in=0.0
				t_inc=.4
				while True:
					print '------'+state+' '+str(stage)+' ---iteration '+str(it)+' ---------'
					rec.get_img()
					#check sonar. if true change behavior and break
					err=follow_most_pixels(xit=10,yit=-10,x1=110, x2=210, y1=100, y2=70, inp=.5,alg=0)
					print 'err ',err
					t_diff=time.time()-end1
					vel_in=((t_diff)*(err*30)+vel_in*(t_inc-t_diff))/t_inc
					end1=time.time()
					#vel_in=.95*vel_in+2*err#-.5 to .5, degrees/sec should be around 20 (.05*2*20) at its highest
					print 'yaw_rate ',yaw_rate
					error=(vel_in-yaw_rate)
					print 'error ', error
					offset=pid1.update(error)
					it+=1
					print 'abs incline ', str(incline-inc_offset)
					print 'heading', heading
					car1.speed(pwm1, offset)
					if (end1-start)>6600:#gps_check(coordinates[1])
                                                print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						global state
						state="turn"
						break
