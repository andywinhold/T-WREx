import RPi.GPIO as GPIO
import math
import time
import subprocess
import sys

# import pandas(?)


# PWM Work
# ---------------------------------------
# first insight: https://www.raspberrypi.org/forums/viewtopic.php?f=35&t=62438&p=463458#p463458

GPIO.setmode(GPIO.BCM)   # choose BCM numbering
GPIO.setup(__, GPIO.OUT) #set __ pin as output

hp = GPIO.PWM(__, 1) # create object hotplate (hp) on pin __ at 1 Hz (Prof. Groppi recommends we start at 1 Hz to start)

desired_temp = 600 # temp of hp in deg C.
curr_temp = ___    # this should be read from a TC


# Will allow you to kill the program
except KeyboardInterrupt:
    hp.stop() # stop PWM output
    GPIO.cleanup() #clean up GPIO pins on CTRL+C exit()


if__name__=='__main__':
    main()
