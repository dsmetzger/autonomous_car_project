import thread

global coordinates=[]#holds the waypoints to navigate to, odd indexs should be end of straight section of sidewalk.
global state="drive"#the current state of the robot.
global stage=0#which sidewalk is the robot on.

class recognition:
	def __init__(self):
		#Begins camera connection
	def get_img(self):
		#update current image.
	def get_lines(self):
		#function that returns a series of lines in the current image. Possibly using an input region of interest of the image.
	def white_det(self):
		#returns pixel values that match that of the sidewalk.
class GPS:
	def __init__(self):
		#start the gps connection
		#attribute of current gps position, current speed, and estimated direction vector of cumulative gps positions.
		u = mraa.Uart(0)
		path=u.getDevicePath()
		self.f=open(path,'r')
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
		#attribute of current current speed.
	def speed(self):
		#changes the speed of individual motors to a specified inputs using PWM.
	def stop(self):
		#called in emergency if the sensor activates at a close proximity.


def __main__()
	#create regocnition object, GPS object, and car instance.

	#continous funcion that uses the gps vector attribute, position on sidewalk, and angle to sidewalk to create movement calculations.
	while True:
		#use gps to determine if turn is coming up
		#possibly use eigenvectors to determine stage
		#check sonar
		#stage will be constant at 0 for now
		if stage==0:
			if state=='drive':
				pass
			elif state=='turn':
				pass
			elif state=='ob_det':
				pass
