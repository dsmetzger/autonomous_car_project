import Adafruit_BBIO.UART as UART
import serial

def parse(in_str,gps):
	#print in_str
	segmented_data=in_str.split(',')
	if segmented_data[0]=='$GPGGA':# and segmented_data[6]!=0:
		print gps
		print in_str
		#return lat,lon

def __main__():
	UART.setup("UART1")
	UART.setup("UART4")
	GPS1 = serial.Serial('/dev/ttyO1', 4800)
	GPS4 = serial.Serial('/dev/ttyO4', 4800)
	while(1):
		if GPS1.inWaiting() ==1:
			parse(GPS1.readline(),1)	 
		if GPS4.inWaiting() ==1:
			parse(GPS4.readline(),4)

__main__()
