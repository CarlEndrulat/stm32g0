import time
import spidev
import os

bus = 0

device = 0

os.system("python restartSPI.py")

spi = spidev.SpiDev(bus, device)

spi.open(bus, device)

spi.max_speed_hz = 100000
spi.mode = 0

msg = "Hello World"
msgList = [0x76, 0x77]
msgByte = [0x00]
msgBytes = [0x5A]

#clear buffer?
#clear = spi.readbytes(4096)

spi.writebytes([0x5A])

# test buffer
#time.sleep(0.005)
#clear = spi.readbytes(4096)
#print(clear)

while 1:
	msgBytes = msgByte

	print(msgBytes)
#	spi.writebytes(msgBytes)
#	time.sleep(.25)
#	ret = spi.readbytes(1)
	ret = spi.xfer2(msgBytes, 1000000, 0, 8)
#	ret = spi.xfer2(msgByte)
#	msgBytes = ret
	print(ret)
