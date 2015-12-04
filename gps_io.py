import mraa
import operator

#try signals https://docs.python.org/2/library/signal.html

def checksum(data):
	#doesnt currently work
	#strip the new line, compare the checksum at end of string with beggining of string
	data = data.strip('\n')
	nmeadata,cksum = sentence.split('*', 1)
	calc_cksum = reduce(operator.xor, (ord(x) for x in nmeadata), 0)
	if calc_cksum!=int(cksum,16):
		print 'checksum failed'
		return False
	else:
		return True

class GPS:
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
			#return segmented_data[2:6]
			#return full data
			return segmented_data
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

if __name__ == "__main__":
	gps_conn=GPS()
	x=gps_conn.read_position_data()
	print x
