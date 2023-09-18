import time
import spidev

bus = 0

device = 0

#NOTE: if at first this doesn't work try restarting the pi

spi = spidev.SpiDev(bus, device)

#spi.open(bus, device)

spi.max_speed_hz = 50000
spi.mode = 0

msg = "Hello World"
msgList = [0x76, 0x77]

while 1:
	spi.writebytes(msgList)

	time.sleep(5)

