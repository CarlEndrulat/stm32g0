import time
import RPi.GPIO as GPIO
import os

os.system("python pinsOff.py")

GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)
GPIO.setup(15, GPIO.OUT)

#using PA5. make sure it's high impedance/input
GPIO.setup(23, GPIO.IN)

#set PA14 (BOOT0) as high
# GPIO17 = pin11
GPIO.output(11, True)
time.sleep(0.5)

#restart stm
# GPIO22 =pin15
GPIO.output(15, False)
time.sleep(1)
GPIO.output(15, True)
GPIO.setup(15, GPIO.IN)

