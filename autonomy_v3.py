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
#old compass
#slots = "/sys/devices/bone_capemgr.9/slots"
#os.system("echo BB-I2C1 > %s" % slots)
#bus = smbus.SMBus(2)
#address = 0x1e
#thread locks
hlock = thread.allocate_lock()
ilock = thread.allocate_lock()

#gps
UART.setup("UART4")
gps1 = serial.Serial("/dev/ttyO4", 4800)
#38.82764204,-77.30581341
# section 1 end [3849.7124, 7718.3908]
coordinates=[[3849.669975,7718.3229125],[3849.6589066667,7718.3471666667],[3849.7127, 7718.3901],[0.0,0.0],[0.0,0.0]]  #holds the waypoints to navigate to of signifigant turns. first waypoint is starting point
gps_position=coordinates[0] #lat,lon

state="test"  #the current state of the robot.
stage=1  #which section of sidewalk the robot is on. To compenstate for changes in environment. 0=engineering building sidewalk, 1= art building sidewalk.... etc


object_detect=0 #thread the sonar
heading =0.0
incline =0.0
inc_offset=1.0#set at certain stages
yaw_rate=0.0 #radians per sec
pwm1=0.0

'''
def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val
def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val
def write_byte(adr, value):
    bus.write_byte_data(address, adr, value)	
def compass():
	global heading
	#global incline
	write_byte(0, 0b01110000)
	write_byte(1, 0b00100000)
	write_byte(2, 0b00000000)
	
	x_offset = 18#9
	y_offset = -95#-80
	z_offset = -27#0
	#for calibration
	while True:
		x_out = (read_word_2c(3) - x_offset) * 1.271859
		y_out = (read_word_2c(7) - y_offset) * 1
		h1  = math.atan2(y_out, x_out)
	        if (h1 < 0):
		        h1 += 2 * math.pi
                #if (d1 < 0):
		#        d1 += 2 * math.pi
		#hlock.acquire()
		#heading = .8*heading + .2*(math.degrees(h1))
		heading=math.degrees(h1)
                #hlock.release()
		time.sleep(.02)
'''	
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
				if abs(int(bgr[1])-int(bgr[2]))<16 and int(bgr[0])>50 and ((bgr[0]*1.25)>(bgr[1]+bgr[2])/2):
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
				if abs(int(bgr[1])-int(bgr[2]))<14 and int(bgr[0])>85 and ((bgr[0]*1.25)>(bgr[1]+bgr[2])/2):
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
		#create green pointers on sidewalk
		count=[]
		tot=0
		for x in range(x1,x2,xit):
			for y in range(y1,y2,yit):
				bgr=self.img[y,x]
				print 'g,r differe '+str(abs(int(bgr[1])-int(bgr[2])))+' <  14'
				print 'b magnitude '+str(int(bgr[0]))+' >  80'
				print 'b > |g,r|   '+str(((bgr[0]*1.25)))+' > '+str((bgr[1]+bgr[2])/2)
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
		if offset>15:
			x=15
		elif offset<-15:
			x=-15
		else:
			x=offset
		print 'duty ',duty
		if duty>75 or duty<-20:
			print 'bad pwm'
			return
		else:
			print '  '
		#smaller offset for high speeds, assumed top offset of 15
		i=x/15#i between -1 and 1
		mult=-.25*abs(duty)+30
		#mult=-.199*abs(duty)+25
		x=mult*i
		print 'effective offset ', x
		#check if negative
		l_speed=duty+x
		r_speed=duty-x
		if l_speed<0:#negative speed, reverse
			GPIO.output(self.left_wheel_dir, GPIO.LOW)
		else:
			GPIO.output(self.left_wheel_dir, GPIO.HIGH)
		if r_speed<0:#negative speed, reverse
			GPIO.output(self.right_wheel_dir, GPIO.LOW)
		else:
			GPIO.output(self.right_wheel_dir, GPIO.HIGH)
		PWM.set_duty_cycle(self.left_wheel, abs(l_speed))
		PWM.set_duty_cycle(self.right_wheel, abs(r_speed))
	def speed1(self, duty=55, offset=0):
		print 'duty ',duty
		if duty>75 or duty<-20:
			print 'bad pwm'
			return
		else:
			print '  '
		#check if negative
		l_speed=duty+offset
		r_speed=duty-offset
		if l_speed<0:#negative speed, reverse
			GPIO.output(self.left_wheel_dir, GPIO.LOW)
		elif l_speed>80:#check if speed too high
			l_speed=80
		else:
			GPIO.output(self.left_wheel_dir, GPIO.HIGH)
		if r_speed<0:#negative speed, reverse
			GPIO.output(self.right_wheel_dir, GPIO.LOW)
		elif r_speed>80:#check if speed too high
			r_speed=80
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

def follow_most_pixels(xit,yit,inp=.5,alg=0):
	sum1=0
	sum2=0
	if alg==0:#first algorithim, follow most pixels
		l1=rec.wh_det(x1=0,x2=320,y1=215,y2=35,xit=xit, yit=yit,er_max=4,slope_en=0)
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
def get_direction(t_compass=.2,t_gyro=.2):
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
		#x3=.9*x2+.1*x1
		#y3=.9*y2+.1*y1
		if t_diff<t_compass:
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
		#angles
		x,y,yaw_rate_new=bno.read_gyroscope()#angular velocity (degrees/sec)
		#print 'l_accel,', bno.read_linear_acceleration()
		#bno.read_gravity()
		pitch, incline1, yaw = bno.read_euler()
		t_diff=time.time()-end2
		end2=time.time()
		#incline=.05*(incline1-inc_offset)+incline*.95
		#x1=math.cos(yaw)
		#x2=math.cos(yaw_old)
		#y1=math.sin(yaw)
		#y2=math.sin(yaw_old)
		#yaw_rate_n=math.atan2((y1-y2),(x1-x2))/t_diff
		#yaw_old=yaw
		#print 'yaw rate ',yaw_rate_n
		#print str(t_diff/t_gyro)
		#print str((t_gyro-t_diff)/t_gyro)
		if t_diff<t_gyro:
			incline=((t_diff)*(incline1)+incline*(t_gyro-t_diff))/t_gyro
			yaw_rate=((t_diff)*(yaw_rate_new*(-180/math.pi))+yaw_rate*(t_gyro-t_diff))/t_gyro
		else:
			incline=(incline1)
			yaw_rate=yaw_rate_new
		time.sleep(.009)
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
def gps_check(destination=coordinates[stage+1]):
	print 'current pos ',gps_position
	print 'check pos ',destination
        if math.sqrt((gps_position[0]-destination[0])**2+(gps_position[1]-destination[1])**2)< .0066:#.00641098:#less than two stdev
                return True
        else:
                return False
def follow_edge(alg=0,input1=[16,14,8]):#1 foot, input1=[16, 11, 7]
	if alg==0:
		l1=rec.wh_det_horizontal(x1=160,x2=320,y1=171,y2=110,xit=10,yit=-30,er_max=1)
		print 'follow_edge list', l1
		#set y=171 error, count should be above 16= 160 pixels
		e1=(l1[0]-input1[0])
		#set y=141 error, count should be above 14= 140 pixels
		e2=(l1[1]-input1[1])
		#set y=111 error, count should be above 8= 80 pixels
		e3=(l1[2]-input1[2])
		error=(.3*e1+.4*e2+.3*e3)
		return error
	elif alg==1:#right side
		#error based on distance and predicted distance
		l1=rec.wh_det_horizontal(x1=200,x2=320,y1=151,y2=110,xit=10,yit=-20,er_max=1)
		print 'follow_edge list', l1
		#distance error
		e1=(l1[0]-8)/8
		e2=(l1[1]-6)/8
		e3=(l1[2]-4)/8
		#slope error
		slope=((l1[2]-l1[0])/2)
		e4=-4-slope
		print 'slope ',slope
		error=(.2*e1+.3*e2+.3*e3+.2*e4)
		return error
	elif alg==2:#follow right side using list
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
if __name__ == "__main__":
	#create regocnition, GPS, and car instance.
	rec=recognition()
	rec.get_img()
	rec.sample()
	car1=car()
	#while True:
	#	bno = BNO055.BNO055(rst='P9_12')
	#	print bno.get_calibration_status()
	#Thread gps sonar and compass
	thread.start_new_thread(get_location, ())
	#thread.start_new_thread(compass, ())
	thread.start_new_thread(get_direction, ())
	thread.start_new_thread(set_speed, ())
	#while True:
		#print yaw_rate
	#	print heading
	#	pass
	time.sleep(5)
	#set gyro offset on start
	while True:
		if state=='drive':
			if stage==0:
				it=0
				start = time.time()
				end1=start
				car1.forward()
				#I=0
				pid1=PID(P=15,I=15,D=0)
				while True:
					print '------'+state+' '+str(stage)+' ---iteration '+str(it)+' ---------'
					rec.get_img()
					#check sonar. if true change behavior and break
					error=follow_most_pixels(xit=5,yit=-5,inp=.5,alg=1)
					#print 'error ', error
					offset=pid1.update(error)
					it+=1
					print 'incline ', incline
					print 'inc_offset ', inc_offset
					#camera limit
					l1=rec.wh_det(x1=160,x2=161,y1=150,y2=90,xit=40, yit=-5,er_max=1,slope_en=0)
					print 'edge close list', l1					
					if l1[-1]< 11:
						if offset>0:
							offset=-7
					#compass limit
					print 'heading ',heading
					if heading>279 and heading<360:
						if offset>0:
							offset=-7
					#check for edge
					if edge_check():
						car1.speed(-20, offset)
						time.sleep(.3)
						car1.speed(-40, -offset)
						time.sleep(.4)
					else:
						car1.speed(pwm1, offset)
					end1=time.time()
					if gps_check(coordinates[1]) or (end1-start)>30:
                                                print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						global state
						state="turn"
						break
			elif stage==1:
				it=0
				start = time.time()
				end1=start
				car1.forward()
				#I=0
				pid1=PID(P=15,I=15,D=0)
				while True:
					print '------'+state+' '+str(stage)+' ---iteration '+str(it)+' ---------'
					rec.get_img()
					#check sonar. if true change behavior and break
					error=follow_most_pixels(xit=5,yit=-5,inp=.5,alg=1)
					print 'error ', error
					offset=pid1.update(error)
					it+=1
					#set speed
					print 'heading ', heading
					#check for offset=0
					#camera limit
					l1=rec.wh_det(x1=160,x2=161,y1=150,y2=90,xit=40, yit=-5,er_max=1,slope_en=0)
					print 'edge close list', l1					
					if l1[-1]< 11:
						if offset>-5:
							offset=-4*(11-l1[-1])
					#compass limit
					#if heading>333 or heading<50:#old compass
					if heading>35 and heading<100:
						if offset>0:
							offset=0
					if heading>200 and heading <305:
						if offset<0:
							offset=0
                                        print 'offset ',offset
					print 'incline ', incline
					print 'inc_offset ', inc_offset    
					#check for edge
					if edge_check():
						car1.speed(-20, offset)
						time.sleep(.3)
						car1.speed(-40, -offset)
						time.sleep(.4)
					else:
						car1.speed(pwm1, offset)    
					end1=time.time()
					if gps_check([3849.6998, 7718.3833]):#coordinates[2]
                                                print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						print 'change to turn stage ',stage
						global state
						state='turn'
						break
			elif stage==2:
				it=0
				start = time.time()
				end1=start
				car1.forward()
				car1.speed(pwm1, 0)
				turn=0
				while True:#go till edge is detected
					print '------'+state+' '+str(stage)+' ---iteration '+str(it)+' ---------'
					#perform white detection
					rec.get_img()
					l1=rec.wh_det(x1=0,x2=121,y1=220,y2=90,xit=120, yit=-5,er_max=2,slope_en=0)
					print l1
					#check sonar. if true change behavior and break
					it+=1
					end1=time.time()
					if l1[-1]< 20 or l1[-2]<18:
						turn+=1
					if turn ==3:
						print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						print 'change to turn stage 2'
						global state
						state='turn'
						car1.speed(0, 5)
                                                break
						
			elif stage==3:
				it=0
				start = time.time()
				car1.forward()
				pid1=PID(P=20,I=50)
				while True:
					print '-----'+state+' '+str(stage)+' ---iteration '+str(it)+' ---------'
					#perform white detection
					rec.get_img()
					#check sonar. if true change behavior and break
					#check gps for change to turn state
					error=follow_most_pixels(xit=10,yit=-10, alg=3)
					print 'error ', error
					it+=1
					offset=pid1.update(error)
					#set speed
					print 'offset ',offset					
					car1.speed(60, offset)
					end1=time.time()
					if gps_check():
                                                print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						print 'change to turn stage ',stage
						global state
						state='turn'
						break
			elif stage==4:
				it=0
				start = time.time()
				car1.forward()
				pid1=PID(P=15,I=15)
				while True:
					print '------'+state+' '+str(stage)+' ---iteration '+str(it)+' ---------'
					#perform white detection
					rec.get_img()
					#check sonar. if true change behavior and break
					#check gps for change to turn state
					error=follow_most_pixels(xit=10,yit=-10, alg=3)
					print 'error ', error
					it+=1
					offset=pid1.update(error)
					#set speed
					print 'offset ',offset					
					car1.speed(65, offset)
					end1=time.time()
					if gps_check(coordinates[5]):
                                                print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						print 'change to turn stage ',stage
						global state
						state='turn'
						break
			elif stage==99:#end stage
				it=0
				start = time.time()
				end1=start
				car1.forward()
				#I=0
				pid1=PID(P=15,I=15,D=0)
				while True:
					print '------'+state+' '+str(stage)+' ---iteration '+str(it)+' ---------'
					rec.get_img()
					#check sonar. if true change behavior and break
					error=follow_most_pixels(xit=5,yit=-5,inp=.5,alg=2)
					#print 'error ', error
					offset=pid1.update(error)
					it+=1
					print 'incline ', incline
					print 'inc_offset ', inc_offset
					#camera limit
					l1=rec.wh_det(x1=160,x2=161,y1=150,y2=90,xit=40, yit=-5,er_max=1,slope_en=0)
					print 'edge close list', l1					
					if l1[-1]< 11:
						if offset>0:
							offset=-7
					#compass limit
					print 'heading ',heading
					if heading>279 and heading<360:
						if offset>0:
							offset=-7
					#check for edge
					#if edge_check():
					#	car1.speed(-20, offset)
					#	time.sleep(.3)
					#	car1.speed(-pwm1, offset)
					#	time.sleep(.4)
					#else:
					car1.speed(pwm1, offset)
					end1=time.time()
					if gps_check(coordinates[1]) or (end1-start)>20:
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
				pid1=PID(P=15,I=15,D=0)
				while True:
					print '------'+state+' '+str(stage)+' ---iteration '+str(it)+' ---------'
					rec.get_img()
					#check sonar. if true change behavior and break
					error=follow_most_pixels(xit=5,yit=-5,inp=.5,alg=1)
					#print 'error ', error
					offset=pid1.update(error)
					it+=1
					print 'incline ', incline
					print 'inc_offset ', inc_offset
					#camera limit
					l1=rec.wh_det(x1=160,x2=161,y1=160,y2=100,xit=40, yit=-5,er_max=1,slope_en=0)
					print 'edge close list', l1					
					if l1[-1]< 11:
						if offset>-5:
							offset=-4*(12-l1[-1])
					#compass limit
					print 'heading ',heading
					#check for edge
					#if edge_check():
					#	car1.speed(-20, offset)
					#	time.sleep(.3)
					#	car1.speed(-pwm1, offset)
					#	time.sleep(.4)
					#else:
					car1.speed(pwm1, offset)
					end1=time.time()
					if heading>330:
                                                print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						print 'change to drive stage 1'
						global stage
						global state
						state="drive"
						stage=1
						break
			elif stage==1:
				it=0
				start = time.time()
				end1=start
				car1.forward()
				#I=0
				pid1=PID(P=15,I=15,D=0)
				while True:
					print '------'+state+' '+str(stage)+' ---iteration '+str(it)+' ---------'
					rec.get_img()
					#check sonar. if true change behavior and break
					error=follow_most_pixels(xit=5,yit=-5,inp=.5,alg=1)
					#print 'error ', error
					offset=pid1.update(error)
					it+=1
					print 'incline ', incline
					print 'inc_offset ', inc_offset
					#camera limit
					l1=rec.wh_det(x1=160,x2=161,y1=150,y2=90,xit=40, yit=-5,er_max=1,slope_en=0)
					print 'edge close list', l1					
					if l1[1]< 11:
						if offset>-5:
							offset=-4*(11-l1[1])
					#compass limit
					print 'heading ',heading
					#check for edge
					#if edge_check():
					#	car1.speed(-20, offset)
					#	time.sleep(.3)
					#	car1.speed(-pwm1, offset)
					#	time.sleep(.4)
					#else:
					car1.speed(pwm1, offset)
					end1=time.time()
					if heading>50 and heading<100:
                                                print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						print 'change to drive stage 2'
						global stage
						global state
						state="drive"
						stage=2
						break
			elif stage==2:#turn left when allowed #stage 2 turn into forest area
				it=0
				start = time.time()
				end1=start
				car1.forward()
				turn=-1#when >0 turn towards sidewalk, else turn away sidewalk
				while True:
					print '------'+state+' '+str(stage)+' ---iteration '+str(it)+' ---------'
					rec.get_img()
					#check for turn
					#count=rec.wh_det_horizontal(x1=160,x2=320,y1=200,y2=180,xit=10,yit=-10,er_max=2)
					l1=rec.wh_det(x1=0,x2=121,y1=220,y2=90,xit=120, yit=-5,er_max=2,slope_en=0)
					print l1	
					#check sonar. if true change behavior and break
					#check gps for change to turn state					
					if l1[-1] > 16 and l1[-2]>18:
						if turn<3:
							turn+=1
					else:
						if turn>-3:
							turn-=1
					#set speed					
					if turn ==1:
						print 'turn left'
						car1.forward()
						car1.speed(pwm1, 15)
					elif turn== -1:
						print 'turn right'
						car1.spin_right()
						car1.speed(pwm1, 0)
					it+=1
					end1=time.time()
					#check if compass is in direction of forest sidewalk
                                        print 'heading ',heading
					if heading<360 and heading>200:
						print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						print 'change to drive stage 3'
						global stage
						global state
						state="drive"
						stage=3
						break	
			elif stage==20:
				it=0
				start = time.time()
				end1=start
				car1.forward()
				I=0
				slope=-.5
				b=10
				while True:
					print '------'+state+' '+str(stage)+' ---iteration '+str(it)+' ---------'
					#perform white detection
					rec.get_img()
					#check sonar. if true change behavior and break
					#check gps for change to turn state
					error,slope,b=control_distance(5,-5,dist=40,slope=slope,b=b)
					print 'slope ', slope					
					print 'error ', error
					offset=50*error+50*I
					print 'P ',str(50*error)
					print 'I ',str(50*I)
					#update I
					it+=1
					end=time.time()
					period=(end-start)/it
					I=.6*I+.4*error*(time.time()-end1)/period#multiply by change in time

					
					#set speed
					print 'offset ',offset					
					car1.speed(55, offset)

					end1=time.time()
					if end1-start>40:
						car1.stop()
						print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						exit()
			elif stage==21:#turn right when allowed
				it=0
				start = time.time()
				end1=start
				car1.forward()
				turn=0#when >0 turn towards sidewalk, else turn away sidewalk 
				ymax=220-135/5
				while True:
					print '------'+state+' '+str(stage)+' ---iteration '+str(it)+' ---------'
					#perform white detection
					rec.get_img()
					#check for turn
					#count=rec.wh_det_horizontal(x1=160,x2=320,y1=200,y2=180,xit=10,yit=-10,er_max=2)
					l1=rec.wh_det(x1=119,x2=320,y1=220,y2=90,xit=120, yit=-5,er_max=2,slope_en=0)
					print l1	
					#check sonar. if true change behavior and break
					#check gps for change to turn state					
					if l1[-1] > 22 and l1[-2]>20:
						if turn<3:
							turn+=1
					else:
						if turn>-3:
							turn-=1
					#set speed					
					if turn ==1:
						print 'turn right'
						car1.forward()
						car1.speed(40, 15)
					elif turn== -1:
						print 'turn left'
						car1.spin_left()
						car1.speed(40, 0)
					it+=1
					end1=time.time()
					if end1-start>40:
						car1.stop()
						print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						exit()	
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
					print 'incline ', incline
					print 'inc_offset ', inc_offset
					print 'heading', heading
					#check for edge
					#if edge_check():
					#	#break
					#	car1.speed(-20, offset)
					#	time.sleep(.4)
					#	#reverse
					#	car1.speed(-50, -offset)
					#	time.sleep(.2)
					#else:
					#	#set speed
					#	car1.speed(pwm1, offset)
					#offset=turn_speed_check(offset)
					car1.speed(pwm1, offset)
					end1=time.time()
					if (end1-start)>600:#gps_check(coordinates[1])
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
					print 'incline ', str(incline-inc_offset)
					print 'heading', heading
					#car1.speed1(pwm1, offset)
					end1=time.time()
					if (end1-start)>600:#gps_check(coordinates[1])
                                                print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						global state
						state="turn"
						break
