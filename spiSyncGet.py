import time
import spidev
import os
import RPi.GPIO as GPIO

#confirm spi lines are set properly
os.system("python restartSPI.py")
time.sleep(0.5)

bus = 0

device = 0

spi = spidev.SpiDev(bus, device)

#start spi clock
spi.open(bus, device)

spi.max_speed_hz = 100000
spi.mode = 0
time.sleep(0.5)

SOF = [0x5A]
getCmdFrame = [0x00, 0xFF]
ACK = [0x79]
NACK = [0x1F]
MOSISYNC = [0x5A]
MISOSYNC = [0xA5]
syncHandsake = [0x00]

#Send sync command
while 1:
#	spi.writebytes(MOSISYNC)
	recBytes = spi.xfer2(MOSISYNC, 100000, 0, 8)
#	recBytes = spi.readbytes(1)
	if(recBytes == MISOSYNC):
		print("step one done")
		break
	else:
		print(recBytes)

recByte = spi.xfer2([0x00], 100000, 0, 8)
print(recByte)
print(spi.readbytes(4096))

while 1:
	recByte = spi.xfer2([0x00], 100000, 0, 8)
#	byte = spi.readbytes(1)
	if(recByte != [165L]):
		print(recByte)
		break
print("found non-sync byte")

#spi.writebytes([0x00])
#xx = spi.xfer2([0x00], 100000, 0, 8)

while 1:

#	spi.writebytes([0x00])
#	time.sleep(0.05)
#	xx = spi.readbytes(2)
#	print(xx)
	if(xx == MISOSYNC):
#		spi.writebytes(MOSISYNC)
		xx = spi.xfer2([0x00])
		continue

#	print("received non-sync")
	spi.writebytes(xx)
#	time.sleep(0.05)
	ret = spi.readbytes(1)
	if(ret == ACK):
		print("ACK returned!")

		spi.writebytes(ACK)
		time.sleep(0.05)
		if(spi.readbytes(1) == ret):
			print("SYNCED!")
			break

time.sleep(0.05)
#send commands
spi.writebytes(SOF)

#send cmd frame
#spi.writebytes(getCmdFrame)

#wait for ACK
while 1:
	spi.writebytes(getCmdFrame)
	time.sleep(0.5)
	recBytes = spi.readbytes(1)
	if(recBytes == ACK):
		print("received ACK!")
		break
	if(recBytes == NACK):
		#spi.writebytes(getCmdFrame)
		print("try again!")

	print(recBytes)
	#time.sleep(0.1)

#print out response
numberOfBytes = api.readbytes(1)
print(spi.readbytes(numberOfBytes))

