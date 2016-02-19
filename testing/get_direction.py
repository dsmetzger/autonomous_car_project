import smbus
import Adafruit_BBIO.GPIO as GPIO


if __name__ == "__main__":
	GPIO.setup("P8_14", GPIO.IN)
	#read adress 0x3D
	bus = Adafruit_I2C(address=0x3D)
	#check if dready is high
	if GPIO.input("P8_14"):
		#register address(in hex)
		#03 msb of x
		#04 lsb
		#07 msb of y
		#08 lsb
		x1=bus.readList(self, 0x03, 8)
		x2=bus.readList(self, 0x04, 8)

		y1=bus.readList(self, 0x07, 8)
		y2=bus.readList(self, 0x08, 8)
		print x1,x2
		print y1,y2
		#concatanate and convert to float.
