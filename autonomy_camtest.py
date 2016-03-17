import thread
import cv2
import numpy as np
import time
import math

coordinates=[]  #holds the waypoints to navigate to, odd indexs should be end of straight section of sidewalk.

state="turn"  #the current state of the robot.
stage=20  #which section of sidewalk the robot is on. To compenstate for changes in environment. 0=engineering building sidewalk, 1= art building sidewalk.... etc

gps_read_flag=0  #set to 0 while its being written to in the thread
gps_position=[0,0]  #lat,lon

debug_mode=1  #more print statements and saving of pictures may result.

object_detect=0 #thread the sonar

speed=0 #speed to be changed by inclination

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
				if abs(int(bgr[1])-int(bgr[2]))<14 and ((int(bgr[0])+int(bgr[1])+int(bgr[2]))/3)>80:
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
				if abs(int(bgr[1])-int(bgr[2]))<14 and ((int(bgr[0])+int(bgr[1])+int(bgr[2]))/3)>80:
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
		pass
	def forward(self):
		pass
	def backward():
		pass
	def speed1(self, duty=60, offset=0):
		pass	
	def stop(self):
		pass
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
	elif alg==1:
		#right side
		l1=rec.wh_det(x1=200,x2=320,y1=215,y2=100,xit=xit, yit=yit,er_max=2,slope_en=0)
		print l1
		for x in range(0, len(l1)):
			if l1[x]<18:
				sum1+=1	
		sum2=len(l1)
	else:
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
	print 'left pixels ',sum1
	print 'right pixels ',sum2
	error=(inp-(float(sum1+1)/(sum1+sum2+2)))#gain times error
	#offset=gain*float(sum2-sum1)
	return error

def control_distance(xit,yit,slope,b,side=1,dist=40):#side=1, right of sidewalk
	if side==1:
		y1=240
		y2=80
		inc_max=abs((y2-y1)/yit)
		l1,slopes=rec.wh_det(x1=290,x2=320,y1=y1,y2=y2,xit=xit, yit=yit,er_max=3,slope_en=1)
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
if __name__ == "__main__":
	#create regocnition, GPS, and car instance.
	rec=recognition()
	car1=car()
	
	#Thread gps sonar and compass
	while True:
		if state=='drive':
			if stage==0 or stage==1 or stage==2:
				it=0
				start = time.time()
				end1=start
				car1.forward()
				I=0
				while True:
					print '------iteration '+str(it)+' ---------'
					#perform white detection
					rec.get_img()
					#check sonar. if true change behavior and break
					#check gps for change to turn state
					#error=control_distance(5,-5,dist=40)
					error=follow_most_pixels(xit=10,yit=-10)
					#error=follow_most_pixels(xit=5,yit=-5,inp=8/14,alg=1)#follow right side
					print 'error ', error
					offset=20*error+40*I
					print 'P ',str(20*error)
					print 'I ',str(40*I)
					#update I
					it+=1
					end=time.time()
					period=(end-start)/it
					I=.95*I+error*(time.time()-end1)/period#multiply by change in time

					
					#set speed
					print 'offset ',offset					
					
					car1.speed1(60, -offset)

					#if debug_mode==1:
					#	cv2.imwrite('debug/debug_img'+str(it)+'.jpg', rec.img)
					end1=time.time()
					if end1-start>20:
						car1.stop()
						print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						exit()
		elif state=='turn':
			print "state=turn"
			if stage==0:
				print "stage=0"
				it=0
				start = time.time()
				end1=start
				car1.forward()
				I=0
				while True:
					print '------iteration '+str(it)+' ---------'
					#perform white detection
					rec.get_img()
					#check sonar. if true change behavior and break
					#check gps for change to turn state
					#error=control_distance(5,-5,dist=40)
					error=follow_most_pixels(xit=5,yit=-5,inp=.5,alg=1)
					#error=follow_most_pixels(xit=5,yit=-5,inp=6/14,alg=2)
					print 'error ', error
					offset=50*error+40*I
					print 'P ',str(30*error)
					print 'I ',str(30*I)
					#update I
					it+=1
					end=time.time()
					period=(end-start)/it
					I=.3*I+error*(time.time()-end1)/period#multiply by change in time

					
					#set speed
					print 'offset ',offset					
					
					car1.speed1(55, -offset)

					#if debug_mode==1:
					#	cv2.imwrite('debug/debug_img'+str(it)+'.jpg', rec.img)
					end1=time.time()
					if end1-start>20:
						car1.stop()
						print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						exit()
			elif stage==20:
				print "stage=20"
				it=0
				start = time.time()
				end1=start
				car1.forward()
				I=0
				slope=-.5
				b=10
				while True:
					print '------iteration '+str(it)+' ---------'
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
					
					car1.speed1(55, -offset)

					#if debug_mode==1:
					#	cv2.imwrite('debug/debug_img'+str(it)+'.jpg', rec.img)
					end1=time.time()
					if end1-start>20:
						car1.stop()
						print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						exit()		
			elif stage==21:#turn when allowed
				print "stage=21"
				it=0
				start = time.time()
				end1=start
				car1.forward()
				I=0
				while True:
					print '------iteration '+str(it)+' ---------'
					#perform white detection
					rec.get_img()
					#check for turn
					count=wh_det_horizontal(x1=160,x2=320,y1=200,y2=180,xit=10,yit=-10,er_max=2)
					print count					
					#check sonar. if true change behavior and break
					#check gps for change to turn state					
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
					
					car1.speed1(55, -offset)

					#if debug_mode==1:
					#	cv2.imwrite('debug/debug_img'+str(it)+'.jpg', rec.img)
					end1=time.time()
					if end1-start>20:
						car1.stop()
						print 'stage '+str(stage)+' performed at '+str(it/(end1-start))+' hertz'
						exit()	
			elif stage==2:
				#keep moving until the edge of the camera detects white.
				#when detected, turn in a constant circle(radius of about .6 m) until the edge of the side walk is detected within a specified distance. orientate and drive.
				pass
		elif state=='ob_det':
			pass
