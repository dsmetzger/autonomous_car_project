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


def compass():
	global heading
	bus.write_byte_data(0, 0b01110000)
	bus.write_byte_data(1, 0b00100000)
	bus.write_byte_data(2, 0b00000000)
	
	x_offset = 
	y_offset = 
	x_out = (read_word_2c(3) - x_offset) * 0.92
	y_out = (read_word_2c(7) - y_offset) * 0.92
	z_out = (read_word_2c(5)) * 0.92
	

	heading  = (math.atan2(y_out, x_out))
	if (heading < 0):
		heading += 2 * math.pi
		

	heading = (degrees(heading))