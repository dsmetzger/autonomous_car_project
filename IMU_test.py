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


#accelerometer
from adxl345_v2 import ADXL345
incline=0.0
#

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
	def stop(self):
		PWM.stop(self.right_wheel)
		PWM.stop(self.left_wheel)
		PWM.cleanup()
def get_direction(t_compass=.2,t_gyro=.2):
	bno = BNO055.BNO055(rst='P9_12')
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
	yaw_old=0.0
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
		print 'ang_vel,' ,bno.read_gyroscope()#angular velocity (degrees/sec)
		print 'l_accel,', bno.read_linear_acceleration()
		#bno.read_gravity()
		pitch, incline1, yaw = bno.read_euler()
		t_diff=time.time()-end2
		end2=time.time()
		#incline=.05*(incline1-inc_offset)+incline*.95
		x1=math.cos(yaw)
		x2=math.cos(yaw_old)
		y1=math.sin(yaw)
		y2=math.sin(yaw_old)
		yaw_rate_n=math.atan2((y1-y2),(x1-x2))/t_diff
		yaw_old=yaw
		#print 'yaw rate ',yaw_rate_n
		#print str(t_diff/t_gyro)
		#print str((t_gyro-t_diff)/t_gyro)
		if t_diff<t_gyro:
			incline=((t_diff)*(incline1)+incline*(t_gyro-t_diff))/t_gyro
			yaw_rate=((t_diff)*(yaw_rate_n)+yaw_rate*(t_gyro-t_diff))/t_gyro
		else:
			incline=(incline1-inc_offset)
			yaw_rate=yaw_rate_n
		time.sleep(.009)
def set_speed():
	global pwm1
	while True:
		#force_paralell=50*math.sin(math.radians(incline))
		#f_friction=50*.5*math.cos(math.radians(incline))
		pwm1=500*(math.sin(math.radians(incline))+.11*math.cos(math.radians(incline)))
		#print 'incline ',incline
		#print 'F_para ',force_paralell
		#print 'F_fric ',f_friction
		time.sleep(.05)

if __name__ == "__main__":
	thread.start_new_thread(get_direction, ())
	thread.start_new_thread(set_speed, ())
	#accel1=ADXL345()
	car1=car()
	car1.forward()
	time.sleep(3)
	car1.speed(duty=pwm)
	time.sleep(3)
	#incline=-10
	while True:
		print 'pwm-incline,',pwm,',',incline
		car1.speed(pwm)
		time.sleep(.01)
