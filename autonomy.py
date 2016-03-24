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

#gps
UART.setup("UART4")
gps1 = serial.Serial("/dev/ttyO4", 4800)

#38.82764204,-77.30581341
# section 1 end [3849.7124, 7718.3908]
coordinates=[[3849.669975,7718.3229125],[3849.6589066667,7718.3471666667],[3849.7127, 7718.3901]]  #holds the waypoints to navigate to of signifigant turns, first waypoint is starting point

state="turn"  #the current state of the robot.
stage=0  #which section of sidewalk the robot is on. To compenstate for changes in environment. 0=engineering building sidewalk, 1= art building sidewalk.... etc

gps_position=[0.0,0.0]  #lat,lon

object_detect=0 #thread the sonar


#compass
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
	while True:
		x_out = (read_word_2c(3) - x_offset) * 1.271859
		y_out = (read_word_2c(7) - y_offset) * 1
		z_out = (read_word_2c(5) - z_offset) * 1.183432
		h1  = math.atan2(y_out, x_out)
		d1 = math.atan2(-z_out, math.sqrt(x_out**2+y_out**2))
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
		time.sleep(.05)

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
                                if abs(int(bgr[1])-int(bgr[2]))<14 and int(bgr[0])>90 and ((bgr[0]*1.25)>(bgr[1]+bgr[2])/2):
				#if abs(int(bgr[1])-int(bgr[2]))<14 and ((int(bgr[0])+int(bgr[1])+int(bgr[2]))/3)>80 and ((bgr[0]*1.25)>(bgr[1]+bgr[2])/2):
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
	def wh_det_horizontal(self,x1=320,x2=640,y1=470,y2=200,xit=10,yit=-10,side=0,er_max=2):    
		count=[]#right side
		tot=0
		for y in range(y1,y2,yit):
			test=er_max
			for x in range(x1,x2,xit):
				bgr=self.img[y,x]
				if abs(int(bgr[1])-int(bgr[2]))<18 and ((int(bgr[0])+int(bgr[1])+int(bgr[2]))/3)>80:
				#if abs(int(bgr[1])-int(bgr[2]))<14 and ((bgr[0]*1.25)>(bgr[1]+bgr[2])/2):
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
		#changes the speed of individual motors to a specified inputs using PWM.
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

class PID:
	def __init__(self, P=20.0, I=30.0, D=10.0):
		self.Kp=P
		self.Ki=I
		self.Kd=D
		self.error_prev=0.0
		self.Il=[0.0]
		for x in range(0,5):
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


def follow_most_pixels(xit,yit,inp=.5,alg=0):
	#l1=rec.wh_det(x1=0,x2=320,y1=215,y2=35,xit=xit, yit=yit,er_max=2,slope_en=0)
	#print l1
	#first algorithim, follow most pixels
	sum1=0
	sum2=0
	if alg==0:
		l1=rec.wh_det(x1=0,x2=320,y1=215,y2=35,xit=xit, yit=yit,er_max=2,slope_en=0)
		print l1
		for x in range(0, len(l1)):
			if x<len(l1)/2:
				sum1+=l1[x]	
			else:
				sum2+=l1[x]
		error=(inp-(float(sum1+1)/(sum1+sum2+2)))
	elif alg==1:
		#right side
		l1=rec.wh_det(x1=280,x2=320,y1=215,y2=100,xit=xit, yit=yit,er_max=2,slope_en=0)
		print l1
		for x in range(0, len(l1)):
			if l1[x]<14:
				sum1+=1	
		sum2=len(l1)
		error=(inp-(float(sum1+1)/(sum2+1)))
	elif alg==2:
		#left side
		l1=rec.wh_det(x1=0,x2=160,y1=215,y2=100,xit=xit, yit=yit,er_max=2,slope_en=0)
		print l1
		for x in range(0, len(l1)):
			if x<len(l1)/2:
				if l1[x]>8:
					sum1+=1	
			else:
				if l1[x]>8:
					sum2+=1
		error=(inp-(float(sum1+1)/(sum1+sum2+2)))
	elif alg==3:
		#follow most matching pixels with emphasis on high values
		l1=rec.wh_det(x1=0,x2=320,y1=215,y2=35,xit=xit, yit=yit,er_max=2,slope_en=0)
		print l1
		for x in range(0, len(l1)):
			if x<len(l1)/2:
				sum1+=(l1[x])**2#exponent can be increased	
			else:
				sum2+=(l1[x])**2
		error=(inp-(float(sum1+1)/(sum1+sum2+2)))
	print 'left pixels ',sum1
	print 'right pixels ',sum2
	return error

def control_distance(xit,yit,slope,b,side=1,dist=40):#side=1, right of sidewalk
	if side==1:
		y1=240
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
			slope_avg=.03333*slope1+.9666*slope
			b1=(((l1[-1]+l1[-2])/2)*yit)-slope*160
			b_avg=.03333*b1+.9666*b
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
def gps_check(destination=coordinates[stage+1]):
	#used at end of stages
        if math.sqrt((gps_position[0]-destination[0])**2+(gps_position[1]-destination[1])**2)< .00641098:
                return True
        else:
                return False
if __name__ == "__main__":
	#create regocnition, GPS, and car instance.
	rec=recognition()
	car1=car()
	
	#Thread gps sonar and compass
	thread.start_new_thread(get_location, ())
	thread.start_new_thread(compass, ())

	time.sleep(3)
	while True:
		if state=='drive':
			if stage==0:
				it=0
				start = time.time()
				end1=start
				car1.forward()
				#I=0
				#Il=[0.0]
				#for x in range(0,5):
				#	Il.append(0.0)
				#error_prev=0
				pid1=PID()
				while True:
					print '------'+state+' '+str(stage)+' ---iteration '+str(it)+' ---------'
					rec.get_img()
					#check sonar. if true change behavior and break
					error=follow_most_pixels(xit=5,yit=-5,inp=.5,alg=1)
					print 'error ', error
					#find I
					#I=0
					#for x in Il:
					#	I+=x
					#I=I/len(Il)
					#for x in range(0,len(Il)-1):
					#	Il[x]=Il[x+1]
					#Il[-1]=error
					#offset=30*error+40*I+20*(error-error_prev)
					#offset=20*error+30*I+10*(error-error_prev)
					offset=pid1.update(error)
					#error_prev=error
					#print 'P ',str(20*error)
					#print 'I ',str(30*I)
					#update I
					it+=1
					end=time.time()
					period=(end-start)/it
					#I=.6*I+.4*error*(time.time()-end1)/period#multiply by change in time

					
					#set speed
                                        print 'offset ',offset	                                        
                                        car1.speed(55, -offset)
					end1=time.time()
					print 'position ',gps_position
					if gps_check():
                                                print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						print 'change to turn stage ',stage
						global state
						state='turn'
						break
			elif stage==1:
				it=0
				start = time.time()
				end1=start
				car1.forward()
				I=0
                                section=0
				pwm=0
				while True:
					print '------'+state+' '+str(stage)+' ---iteration '+str(it)+' ----section '+str(section)+'-----'
					rec.get_img()
					#check sonar. if true change behavior and break
					#check gps for change to turn state
					#error=control_distance(5,-5,dist=40)
					error=follow_most_pixels(xit=5,yit=-5,inp=.5,alg=1)
					print 'error ', error
					offset=40*error+30*I
					print 'P ',str(40*error)
					print 'I ',str(30*I)
					#update I
					it+=1
					end=time.time()
					period=(end-start)/it
					I=.6*I+.4*error*(time.time()-end1)/period#multiply by change in time

					
					#set speed
					
                                        print heading
                                        #if math.sqrt((gps_position[0]-3849.6860348)**2+(gps_position[1]-7718.3563952)**2)< .00641098:
					if section==0 and gps_check(destination=[3849.6823, 7718.364]):                                           
						section=1
					elif section==1 and gps_check(destination=[3849.7007, 7718.3796]):
						section=2
					elif section==2 and gps_check(destination=[3849.6938, 7718.3792]):
						section=3
                                        if section==0:       
                                                pwm=60
                                        elif section==1:
                                                pwm=60
                                                if heading>289 or heading<0:#338-20
                                                        if offset>0:
								offset=0
                                        else:
                                                pwm=65
                                        print 'offset ',offset	                                        
                                        car1.speed(pwm, -offset)
					end1=time.time()
					print gps_position
					if gps_check():
                                                print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						print 'change to turn stage ',stage
						global state
						state='turn'
						break
			elif stage==2: #possibly stage 2
				it=0
				start = time.time()
				end1=start
				car1.forward()
				car1.speed(45, 0)
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
						car1.speed(0, 0)
                                                break
						
			elif stage==3:
				it=0
				start = time.time()
				end1=start
				car1.forward()
				I=0
				while True:
					print '------'+state+' '+str(stage)+' ---iteration '+str(it)+' ---------'
					#perform white detection
					rec.get_img()
					#check sonar. if true change behavior and break
					#check gps for change to turn state
					#error=control_distance(5,-5,dist=40)
					error=follow_most_pixels(xit=10,yit=-10)
					#error=follow_most_pixels(xit=5,yit=-5,inp=8/14,alg=1)#follow right side
					print 'error ', error
					#offset=20*error+40*I#working
					offset=20*error+80*I
					print 'P ',str(20*error)
					print 'I ',str(80*I)
					#update I
					it+=1
					end=time.time()
					period=(end-start)/it
					#I=.95*I+error*(time.time()-end1)/period#working
					I=.9*I+.5*error*(time.time()-end1)/period
					
					#set speed
					print 'offset ',offset					
					car1.speed(65, -offset)
					end1=time.time()
					if gps_check(car1):
                                                print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						print 'change to turn stage ',stage
						global state
						state='turn'
						break
		elif state=='turn':
			if stage==0:
				it=0
				start = time.time()
				end1=start
				car1.forward()
				I=0
				error_prev=0
				while True:
					print '------'+state+' '+str(stage)+' ---iteration '+str(it)+' ---------'
					rec.get_img()
					#check sonar. if true change behavior and break
					error=follow_most_pixels(xit=5,yit=-5,inp=.5,alg=1)
					print 'error ', error
					#offset=40*error+30*I
					offset=30*error+30*I+20*(error-error_prev)
					error_prev=error
					print 'P ',str(40*error)
					print 'I ',str(30*I)
					#update I
					it+=1
					end=time.time()
					period=(end-start)/it
					I=.6*I+.4*error*(time.time()-end1)/period

					
					#set speed based on time
					pwm=55-((end-start)/2)
					if pwm<45:
						pwm=45
					print 'offset ',offset					
					car1.speed(pwm, -offset)#nominal 40
                                        print 'heading ',heading
                                        end1=time.time()
                                        if heading>280 or heading < 30:
                                                print 'change to drive 1'
                                                car1.speed(65,0)
                                                global state
                                                global stage
                                                state='drive'
                                                stage=1
					        print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						break
			elif stage==1:
				it=0
				start = time.time()
				end1=start
				car1.forward()
				I=0
				while True:
					print '------'+state+' '+str(stage)+' ---iteration '+str(it)+' ---------'
					#perform white detection
					rec.get_img()
					#check sonar. if true change behavior and break
					#check gps for change to turn state
					#error=control_distance(5,-5,dist=40)
					error=follow_most_pixels(xit=5,yit=-5,inp=.5,alg=1)
					print 'error ', error
					offset=30*error+40*I
					print 'P ',str(30*error)
					print 'I ',str(30*I)
					#update I
					it+=1
					end=time.time()
					period=(end-start)/it
					I=.6*I+.4*error*(time.time()-end1)/period

					
					#set speed
					print 'offset ',offset					
					
					car1.speed(50, -offset)
					#check if comapass is 30 degrees to sidewalk (started turn), change to next drive state 2
					print 'heading ',heading
					end1=time.time()
					if end1-start>50:
						car1.stop()
						print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						exit()
			elif stage==2:#turn left when allowed #stage 2 turn into forest area
				it=0
				start = time.time()
				end1=start
				car1.forward()
				turn=0#when >0 turn towards sidewalk, else turn away sidewalk
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
						car1.speed(50, 15)
					elif turn== -1:
						print 'turn right'
						car1.spin_right()
						car1.speed(50, 0)
					it+=1
					end1=time.time()
					#check if compass is in direction of forest sidewalk
                                        print 'heading ',heading
					if end1-start>25:
						car1.stop()
						print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						exit()	
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
					car1.speed(55, -offset)

					end1=time.time()
					if end1-start>20:
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
						car1.speed(50, -15)
					elif turn== -1:
						print 'turn left'
						car1.spin_left()
						car1.speed(50, 0)
					it+=1
					end1=time.time()
					if end1-start>20:
						car1.stop()
						print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						exit()	
		elif state=='ob_det':
			pass
