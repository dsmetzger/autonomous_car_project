import smbus

#check if dready is high

#register address(in hex)
#03 msb of x
#04 lsb
#07 msb of y
#08 lsb

#read adress 0x3D

bus = Adafruit_I2C(address=0x3D)

x1=bus.readList(self, 0x03, 8)
x2=bus.readList(self, 0x04, 8)

y1=bus.readList(self, 0x07, 8)
y2=bus.readList(self, 0x08, 8)

#concatanate and convert to float.
