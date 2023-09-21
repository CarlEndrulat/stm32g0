#restart the SPI interface

import os
import subprocess
import re
import time
import spidev

returnStr = subprocess.check_output(["lsmod"])

testStr = "spi_bcm2835"

spiStrMatch = re.search("spi_bcm....", returnStr)
spiStr = spiStrMatch.group()

os.system("sudo rmmod %s" % spiStr)

os.system("sudo modprobe %s" %spiStr)
time.sleep(0.25)

#open and close spi comms
bus=0
device=0
spi = spidev.SpiDev(bus, device)
spi.open(bus, device)
spi.max_speed_hz = 5000
spi.mode = 0
spi.close
time.sleep(0.1)
