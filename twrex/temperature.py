#!/usr/bin/env python
# The intent of this file is initially to establish a connection between a
# Raspberry Pi and multiple Adafruit MAX-31855 thermocouple amplifiers (TC amp). With
# this connection, the RPi will obtain readings from each TC amp on the order
# of once a second. These readings will be stored for later analysis as well
# as for live observation of TC readings during T-VAC operation.
# Much of this code was taken from limited's pimometer repo
# on github: https://github.com/limited/pimometer/blob/master/src/ThermocoupleRead.py
# Adaptations may occur as needed to fit our requirements, seeing that we'll
# likely be using more than 4 thermocouples.

import math
import time
import sys
import RPi.GPIO as GPIO
import pandas as pd
import matplotlib.pyplot as plt

#----------------------------------------
# MISO: Master Input, Slave Output
# CS_ARRAY: Chip Set Array
# CLK: Clock
# Changing  setupSpiPins so I can tell it which to use.
MISO = 9
##CS_ARRAY = [18, 23, 24, 25]
CLK = 11

def setupSpiPins(MISO, CS_ARRAY, CLK):
    ''' Set pins as an output except MISO (Master Input, Slave Output)

    Arguments
    ---------
    MISO: int
        Master Input, Slave Output
    CS_ARRAY: array
        Chip Set Array
    CLK: int
        Clock

    Returns
    -------
    None
    '''
    GPIO.setup(CLK, GPIO.OUT)
    GPIO.setup(MISO, GPIO.IN)
    for cs in CS_ARRAY:
      GPIO.setup(cs, GPIO.OUT)
    
    GPIO.output(CLK, GPIO.LOW)
    for cs in CS_ARRAY:
      GPIO.output(cs, GPIO.HIGH)
     

def readTemp(miso):
    
    tcValue = recvBits(miso, 32)
    #print "TC value: %x" % (tcValue)

    if (tcValue & 0x8):
      return 0 #"ERROR: Reserved bit 3 is 1" 

    if (tcValue & 0x20000):
      return 0 #"ERROR: Reserved bit 17 is 1" 

    if (tcValue & 0x10000):
      if (tcValue & 0x1):
        return 0 #"ERROR: Open Circuit"
      if (tcValue & 0x2):
        return 0 #"ERROR: Short to GND"
      if (tcValue & 0x4):
        return 0 #"ERROR: Short to Vcc"

    internal = (tcValue >> 4) & 0xFFF
    #print "Internal: %fC" % (internal*.0625)
    #print "Internal: %fF" % (internal*.0625*9.0/5.0+32)


    tcValue = tcValue >> 18

    temp = tcValue & 0x3FFF
    if (tcValue & 0x2000):
      temp = temp | 0xC000

    temp = temp * 0.25
    return temp
    
def recvBits(cs, numBits):
    '''Receives arbitrary number of bits'''
    retVal = 0

    # Start the read with chip select low
    GPIO.output(cs, GPIO.LOW)
    
    for bit in range(numBits):
        # Pulse clock pin 
        GPIO.output(CLK, GPIO.HIGH)
        
        # Advance input to next bit
        retVal <<= 1

        # Read 1 data bit in
        if GPIO.input(MISO):
            #print 31-bit, "1"
            retVal |= 0x1
        #else:
	#    print 31-bit, "0"
        
        GPIO.output(CLK, GPIO.LOW)
        time.sleep(.001)
    
    # Set chip select high to end the read
    GPIO.output(cs, GPIO.HIGH)

    return (retVal)

def convertTypeJToTypeK(temp):
    ''' Output temperature based on a Type J Thermocouple. 
        Theory: 
          The output temperature for a thermocouple is based on a voltage
          difference across the device. To use a Type J Thermocouple with the
          MAX31855K measurement sensor, do the following:
           1) Use the ITS-90 Type-K Direct Polynomial to find the 
              thermoelectric voltage from the reported temperature
           2) Use the ITS-90 Type-J Inverse Polynomial to find the
              temperature from the computed thermoelectric voltage
          NB: If polynomials are too hard to compute, use a lookup table 
              instead.
    '''
    volt = tcKTempTouV(temp)
    new_temp = tcuVToJTemp(volt)

def tcKTempTouV(temp):
    ''' Return thermoelectic voltage for a temperature'''
    # Type-K Coefficients for 0-1372C
    direct_coeff = [-1.7600413686e1,
                     3.8921204975e1,
                     1.8558770032e-2,
                    -9.9457592874e-5,
                     3.1840945719e-7,
                    -5.6072844889e-10,
                     5.6075059059e-13,
                    -3.2020720003e-16,
                     9.7151147152e-20,
                    -1.210472127e-23]
    a_0 =  1.185976e2
    a_1 = -1.183432e-4

    e = 0
    for (i, coeff) in enumerate(direct_coeff):
        e += coeff * pow(temp, i)

    exp = a_1 * pow(temp - 126.9686, 2)
    e += a_0 * pow(math.e, exp)
    return e

def tcuVToJTemp(volt):
    ''' Return temp across a J-Type thermocouple'''
    # high is 760-1200C, mid is 0-760C
    indirect_coeff={'high': [-3.11358187e3,
                              3.00543684e-1,
                             -9.94773230e-6,
                              1.70276630e-10,
                             -1.43033468e-15,
                              4.73886084e-21],
                    'mid': [0,
                            1.978425e-2,
                            -2.001204e-7,
                            1.036969e-11,
                            -2.549687e-16,
                            3.585153e-21,
                            -5.344285e-26,
                            5.099890e-31]}
    if volt >= 0 and volt < 42919:
        mode = 'mid'
    elif volt >= 42919:
        mode ='high'

    temp = 0
    for (i, coeff) in enumerate(indirect_coeff[mode]):
        temp += coeff * pow(volt, i)
        
    return temp

def convertCToF(temp):
    return (temp*9.0/5.0+32)
    
endplate = []
vacplate = []
shield = []
tcspare = []
filename = 'tc_testing.csv'

if __name__ == '__main__':
    try:
        start_time = time.time()

        GPIO.setmode(GPIO.BCM)
        #print "Started"
        # Change pins here for desired tc amps.
        MISO = 9
        CS_ARRAY = [18, 23, 24, 25]
        CLK = 11
        setupSpiPins(MISO, CS_ARRAY, CLK)
        print "Themocouples Setup"
        while (True):
            for cs in CS_ARRAY:
              val = readTemp(cs)
              if cs == 18:
                  print "TC End Plate Temp:", str(val),"deg C"
                  endplate.append(val)
              if cs == 23:
                  print "TC Vac Plate Temp:", str(val),"deg C"
                  vacplate.append(val)
              if cs == 24:
                  print "TC Shield Temp:", str(val),"deg C"
                  shield.append(val)
              if cs == 25:
                  print "TC ? Temp:", str(val),"deg C"
                  tcspare.append(val)
            time.sleep(10)
            print("--- Time elapsed: %s seconds ---" % (time.time() - start_time))
    except KeyboardInterrupt:
        GPIO.cleanup()
##        sys.exit(0)
        # make lists into data frame
        df = pd.DataFrame({'TC End Plate': endplate, 'TC Vac Plate': vacplate,\
                           'TC Shield': shield, 'TC Spare': tcspare})

        # save to .csv file then read
        df.to_csv(filename, sep=',')
        new_df = pd.read_csv(filename)
        #This row may be needed
        del new_df['Unnamed: 0']

        # create plot
        ax = new_df.plot.area(stacked=False)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Temperature (C)')
        plt.show()
