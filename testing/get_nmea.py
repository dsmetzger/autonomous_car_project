import Adafruit_BBIO.UART as UART

def parse(in_str):
	segmented_data=in_str.split(',')
	if segmented_data[0]=='$GPGGA' and segmented_data[6]!=0:
		print in_str
		#return lat,lon

def __main__():
	UART.setup("UART1")
	UART.setup("UART2")
	UART.setup("UART3")
	GPS1 = serial.Serial('/dev/ttyO1', 9600)
	GPS2 = serial.Serial('/dev/ttyO2', 9600)
	GPS3 = serial.Serial('/dev/ttyO3', 9600)
	while(1):
		if GPS1.inWaiting() ==0:
			parse(NMEA1=GPS1.readline())	 
		if GPS2.inWaiting() ==0:
			parse(NMEA1=GPS2.readline())	
		if GPS3.inWaiting() ==0:
			parse(NMEA1=GPS3.readline())
