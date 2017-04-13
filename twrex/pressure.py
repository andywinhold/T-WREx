# This file's purpose is to establish a connection to the LabJack
# U3-LV DAQ device. On the other side of this connection are (or
# should be) two pressure sensors. They are connected by red/black
# wire and are sending pressure readings as voltages. This code will 
# be reading in those voltages at set intervals and converting the 
# voltages into units of pressure.

#insight: https://labjack.com/support/software/examples/ud/labjackpython/low-level

import time
import sys
import pandas as pd
import matplotlib.pyplot as plt
##import u3

def v_upconvert(V):
        """ Convert the incoming voltage back to voltage measured by pressure sensor.
        The voltage range output by the pressure sensors in use (IONIVAC ITR-90) is from
        0 - 10V. The Labjack DAQ (U3-LV) device used to collect voltages has a max reading of ~2.4V.
        A voltage divider is used to down convert the pressure voltage to one readable by the U3-LV.
        This down converted voltage needs to be up-converted so the correct pressure is calculated.

        Arguments
        ---------
        V: float
        voltage received by U3-LV from pressure sensor via voltage divider

        Returns
        -------
        V_up: float
        voltage value converted to reflect what was sent initially by pressure sensor.
        """
        # values of resistors used in voltage divider in ohms.
        r1 = 1000
        #in our case r2 is a 300 ohm and 15 ohm in series.
        r2 = 315
        r = (r1 + r2)/r2
        V_up = V * r
        return V_up
        
        
def v2p(V):
        """convert voltage readings provided by pressure sensor to
        readings of pressure in units of Torr

        Arguments
        ---------
        V: float
        voltage received from pressure sensor

        Returns
        -------
        p: float
        pressure reading in units of Torr
        """
        #conversion factor to convert V to Torr
        c = -0.125

        p = 10**((V - 7.75)/0.75 + c)
        return p

if __name__ == '__main__':
    try:
        start_time = time.time()

        ###Empty lists for both pressure sensors
        ##p1 = []
        ##p2 = []
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
        p1 = []
        p2 = []
        filename = 'testing_keybord_inter.csv'
        while (True):
                # Get reading from first pressure sensor
        ##      vval1 = d.getAIN(0)
        ##	print "Uncalibrated reading for p1 {0:.5f}".format(vval1)
        ##      calvval1 = v_upconvert(vval1)
        ##      print "Calibrated reading for p1 {0:.5f}".format(calvval1)

                # Get reading from second pressure sensor
        ##	vval2 = d.getAIN(1)
        ##	print "Uncalibrated reading for p2 {0:.5f}".format(vval2)
        ##      calvval2 = v_upconvert(vval2)
        ##      print "Calibrated reading for p2 {0:.5f}".format(calvval2)
                
                # Convert voltage to Torr
        ##        pval1 = v2p(calvval1)
        ##        pval2 = v2p(calvval2)

                time.sleep(2)
                p1int = raw_input('add1')
                p2int = raw_input('add2')
                if p1int == 'done':
                        print "end of collection"
                        print stuff
                        break
                p1fl = float(p1int)
                p2fl = float(p2int)
                p1.append(p1fl)
                p2.append(p2fl)
##                stuff.append(add)

                # add readings to their lists
        ##        pval1 = float(pval1)
        ##        pval2 = float(pval2)
        ##        p1.append(pval1)
        ##        p2.append(pval2)
                print("--- Time elapsed: %s seconds ---" % (time.time() - start_time))
    except KeyboardInterrupt:
##            sys.exit(0)
            # make lists into data frame
            df = pd.DataFrame({'Sensor 1': p1, 'Sensor 2': p2})

            # save to .csv file then read
            df.to_csv(filename, sep=',')
            new_df = pd.read_csv(filename)
            #This row may be needed
            del new_df['Unnamed: 0']

            # create plot
            ax = new_df.plot.area(stacked=False)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Pressure (Torr)')
            plt.show()

            #df1 = pd.DataFrame(stuff)
            ##df1.to_csv('test1.csv', sep=',')
            #new_df = pd.read_csv('test1.csv')
            #del new_df['Unnamed: 0']
            ##new_df.columns = ['row 1']
            ##ax = new_df.plot.area(stacked=False)
            ##ax.set_xlabel('Time (s)')
            ##ax.set_ylabel('Pressure (Torr)')
            ##plt.show()




    # Ctrl+C to close program
