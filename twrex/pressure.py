# This file's purpose is to establish a connection to the LabJack
# U3-LV DAQ device. On the other side of this connection are (or
# should be) two pressure sensors. They are connected by red/black
# wire and are sending pressure readings as voltages. This code will 
# be reading in those voltages at set intervals and converting the 
# voltages into units of pressure.

#insight: https://labjack.com/support/software/examples/ud/labjackpython/low-level

import time
import pandas as pd
import matplotlib.pyplot as plt
##import u3
##
###Empty lists for both pressure sensors
##P1 = []
##P2 = []
##
### Establish connection to first U3 device found. We should only have one connected.
##d = u3.U3()
##
### Get the calibration data of the device. The calibration data 
###will be used by functions that convert binary data to 
###voltage/temperature and vice versa.
##d.getCalibrationData()
##
### Check which FIOs are analog and which are digital
##configDict = d.configIO()
##configDict["FIOAnalog"]
###63
##
### Set the first four (0-3) to analog (15 = 1111 binary) 
###and the rest to digital
###d.configIO(FIOAnalog = 15)
##
### Read from AIN0 in one function
##ain0val = d.getAIN(0)
##print "testing AIN0 {0:.5f}".format(ain0val)
###1.501376
##
### Read from AIN1 in one function
##ain1val = d.getAIN(1)
##print "testing AIN1 {0:.5f}".format(ain1val)
##print ain1val
stuff = []
while (True):
        add = raw_input('add')
        if add == 'done':
                print "end of collection"
                print stuff
                break
        add = float(add)
        stuff.append(add)

df1 = pd.DataFrame(stuff)
df1.to_csv('test1.csv', sep=',')
new_df = pd.read_csv('test1.csv')
del new_df['Unnamed: 0']
new_df.columns = ['row 1']
ax = new_df.plot.area(stacked=False)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Pressure (Torr)')
plt.show()

##while (True):
##	ain0val = d.getAIN(0)
##	print "Reading for AIN0 {0:.5f}".format(ain0val)
##		
##	time.sleep(1)
##
##	ain1val = d.getAIN(1)
##	print "Reading for AIN1 {0:.5f}".format(ain1val)
##
##	time.sleep(1)

def v2p(U):
        """convert voltage readings provided by pressure sensor to
        readings of pressure in units of Torr

        Arguments
        ---------
        U: float
        voltage received from pressure sensor

        Returns
        -------
        p: float
        pressure reading in units of Torr
        """
        #conversion factor to convert V to Torr
        c = -0.125

        p = 10**((U - 7.75)/0.75 + c)
        return p

# Ctrl+C to close program
