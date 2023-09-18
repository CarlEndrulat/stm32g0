import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)

#set all pins as high impedance
GPIO.setup(11, GPIO.IN)
GPIO.setup(15, GPIO.IN)
GPIO.setup(19, GPIO.IN)
GPIO.setup(21, GPIO.IN)
GPIO.setup(23, GPIO.IN)
GPIO.setup(24, GPIO.IN)

