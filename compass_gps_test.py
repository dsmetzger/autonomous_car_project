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
#compass
bus = smbus.SMBus(1)
address = 0x1e
#thread locks
hlock = thread.allocate_lock()
ilock = thread.allocate_lock()

stage=0

#gps
UART.setup("UART4")
gps1 = serial.Serial("/dev/ttyO4", 4800)

#38.82764204,-77.30581341
coordinates=[[3849.669975,7718.3229125],[3849.6589066667,7718.3471666667],[3849.7124, 7718.3908]]  #holds the waypoints to navigate to of signifigant turns, first waypoint is starting point


gps_position=[0.0,0.0]  #lat,lon

heading =0.0
incline =0.0
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
	global incline
	write_byte(0, 0b01110000)
	write_byte(1, 0b00100000)
	write_byte(2, 0b00000000)
	
	x_offset = 18#9
	y_offset = -95#-80
	z_offset = -27#0
	#for calibration
	xmax=0
	ymax=0
	zmax=0
	xmin=0
	ymin=0
	zmin=0
	while True:
		x_out = (read_word_2c(3) - x_offset)# * 1.271859
		y_out = (read_word_2c(7) - y_offset)# * 1
		z_out = (read_word_2c(5) - z_offset)# * 1.183432
		h1  = math.atan2(y_out, x_out)
		d1 = math.atan2(-z_out, math.sqrt(x_out**2+y_out**2)) 
		#print 'x_out ',x_out		
		#print 'y_out ',y_out
		#print 'z_out ',z_out
		#declination is -10*31' = -10.517
	        if (h1 < 0):
		        h1 += 2 * math.pi
                if (d1 < 0):
		        d1 += 2 * math.pi
		hlock.acquire()
		heading = .8*heading + .2*(math.degrees(h1))
                hlock.release()
                ilock.acquire()
		incline = .8*incline + .2*(math.degrees(d1))
                ilock.release()
		#for calibration
		if x_out>xmax:
			xmax=x_out
		if y_out>ymax:
			ymax=y_out
		if z_out>zmax:
			zmax=z_out
		if x_out<xmin:
			xmin=x_out
		if y_out<ymin:
			ymin=y_out
		if z_out<zmin:
			zmin=z_out
		print str((xmax+xmin)/2)
		print str((ymax+ymin)/2)
		print str((zmax+zmin)/2)
		print str((xmax-xmin)/2)
		print str((ymax-ymin)/2)
		print str((zmax-zmin)/2)
		time.sleep(.05)


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


def gps_check(destination=coordinates[stage+1]):
	#used at end of stages
        if math.sqrt((gps_position[0]-destination[0])**2+(gps_position[1]-destination[1])**2)< .00641098:
                return True
        else:
                return False

if __name__ == "__main__":
	#Thread gps sonar and compass
	thread.start_new_thread(get_location, ())
	thread.start_new_thread(compass, ())
	avg_lat=0.0
	avg_lon=0.0
	#test threads loop
	test=0
	while test==0:
		print 'heading ', heading
		print 'incline ', incline
		time.sleep(.05)
	while test==1:
		#print 'location', gps_position
		print 'heading ', heading
		print 'incline ', incline
		time.sleep(.5)
		avg_lat= .1*gps_position[0]+avg_lat*.9
		avg_lon= .1*gps_position[1]+avg_lon*.9
		print gps_position
		print avg_lat, avg_lon
		print gps_check(destination=[3849.7007, 7718.3796])
		pass
