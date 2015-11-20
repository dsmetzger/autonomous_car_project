global coordinates=[]#holds the waypoints to navigate to.

class line_recognition:
	def __init__(self):
		#start video feed
		#attribute of current image, distance to edge of sidewalk, and angle to sidewalk
	def get_img(self):
		#update current image
	def get_lines(self):
		#continous function that controls the outputs a controlled series of lines in the current image. Possibly uses an input region of interest of the image.
class Adafruit_GPS:
	def __init__(self):
		#start the connection and get path
		u = mraa.Uart(0)
		path=u.getDevicePath()
		self.f=open(path,'r')
	def read_data(self,data_str='$GPGGA',iterations=50):
		#wait for, optional checksum, and return the specified identifier
		for i in range(iterations):
			nmea=self.f.readline()
			segmented_data=nmea.split(',')
			if segmented_data[0]==data_str: #and checksum(data):
				return segmented_data,i
			else:
				return False,i
	def read_position_data(self,iterations=50):
		#calls read_data() for the position nmea
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
		#attribute of current gps position, pwm speed, estimated direction vector of cumulative gps positions.
	def read_data(self,data_str='$GPGGA',iterations=50):
		#update position, update vector based on certainty of gps.
	def drive(self):
		#Pass in line_regocnition object, GPS object, and starting speed.
		#continous funcion that uses the gps vector attribute, distance to sidewalk, and angle to sidewalk to create movement calculations.
	def speed(self):
		#changes the speed to a specified input.
	def stop(self):
		#called in emergency if the sensor activates at a close proximity.

