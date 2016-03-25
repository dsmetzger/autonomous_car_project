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
incline=-10
#

class car:
	def __init__(self):
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
		if offset>20:
			x=20
		elif offset<-20:
			x=-20
		else:
			x=offset		
		PWM.set_duty_cycle(self.left_wheel, duty+x)
		PWM.set_duty_cycle(self.right_wheel, duty-x)	
	def stop(self):
		PWM.stop(self.right_wheel)
		PWM.stop(self.left_wheel)
		PWM.cleanup()

def get_incline():
	accel1=ADXL345()
	global incline
	incline=-10
	while True:
		incline=.1*accel1.getXAngle()+incline*.9
		time.sleep(.1)
if __name__ == "__main__":
	thread.start_new_thread(get_incline, ())
	#accel1=ADXL345()
	car1=car()
	car1.forward()
	car1.speed(duty=40)
	#incline=-10
	while True:
		#incline=.1*accel1.getXAngle()+incline*.9
		force_paralell=35*math.sin(math.radians(incline))
		f_friction=35*.6*math.cos(math.radians(incline))
		duty1=85*(math.sin(math.radians(incline))+.64*math.cos(math.radians(incline)))
		print 'incline ',incline
		print 'F_para ',force_paralell
		print 'F_fric ',f_friction
		print 'duty ',duty1
		if duty1>70:
			pass
		car1.speed(duty=duty1)
		time.sleep(1)
