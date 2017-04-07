# PID calculation for our hotplate.
# This code was taken from: https://github.com/steve71/RasPiBrew/blob/master/RasPiBrew/pid/pidpy.py
# And adapted for our own purposes. Steve71 wrote this in Python when it was originally
# written in C here: http://www.vandelogt.nl/uk_regelen_pid.php#PID1
# This second link is a good resource if you do not understand PID algorithms very much.
# I found it useful.

class pidpy(object):
    
    #version of error during previous cycle. Thlt = Temp of Hot Liquid Tun (tank?), related to brewing I beleive.
    ek_1 = 0.0  # e[k-1] = SP[k-1] - PV[k-1] = Tset_hlt[k-1] - Thlt[k-1]

    #version of error two cycles previous.
    ek_2 = 0.0  # e[k-2] = SP[k-2] - PV[k-2] = Tset_hlt[k-2] - Thlt[k-2]

    #input variable during previous cycle.
    xk_1 = 0.0  # PV[k-1] = Thlt[k-1]

    #input variable two cycles previous.
    xk_2 = 0.0  # PV[k-2] = Thlt[k-1]

    #output variable during previous cycle.
    yk_1 = 0.0  # y[k-1] = Gamma[k-1]

    #output variable two cycles previous.
    yk_2 = 0.0  # y[k-2] = Gamma[k-1]

    #low pass filter, used in calcPID_reg3
    lpf_1 = 0.0 # lpf[k-1] = LPF output[k-1]
    lpf_2 = 0.0 # lpf[k-2] = LPF output[k-2]

    #updated output variable, start at 0.
    yk = 0.0 # output

    # Gamma limits. The high and low limits the PID output can be. ie 0-100% 
    GMA_HLIM = 100.0
    GMA_LLIM = 0.0
    
    def __init__(self, ts, kc, ti, td):
        """ initialize the PID environment

        Arguments
        ---------
        ts: float
        setpoint value

        kc: float
        controller gain

        ti: float
        integral time constant

        td: float
        derivative time constant

        Returns
        -------

        None
        """
        print "boo yah"
        # see above
        self.kc = kc
        self.ti = ti
        self.td = td
        self.ts = ts

        #low pass filter, only in calcPID_reg3
        self.k_lpf = 0.0

        #k values for PID controller
        self.k0 = 0.0
        self.k1 = 0.0
        
        #k2 and k3 not used?
        self.k2 = 0.0
        self.k3 = 0.0

        #more low pass filter
        self.lpf1 = 0.0
        self.lpf2 = 0.0

        #intended for timer but not used here?
        self.ts_ticks = 0

        #type of PID controller, I'm assuming 3 means type C since the links above favor that type.
        self.pid_model = 3

        #p/i/d values that combine to make output value (duty cycle for PWM).
        self.pp = 0.0
        self.pi = 0.0
        self.pd = 0.0
        
        if (self.ti == 0.0):
            self.k0 = 0.0
        else:
            self.k0 = self.kc * self.ts / self.ti
        self.k1 = self.kc * self.td / self.ts
        self.lpf1 = (2.0 * self.k_lpf - self.ts) / (2.0 * self.k_lpf + self.ts)
        self.lpf2 = self.ts / (2.0 * self.k_lpf + self.ts) 
        
                          
    def calcPID_reg4(self, xk, tset, enable):
        #initial error
        ek = 0.0
        
        #subtract current variable (xk) from desired temperature (tset) to get current error
        ek = tset - xk # calculate e[k] = SP[k] - PV[k]
        
        if (enable):
            #-----------------------------------------------------------
            # Calculate PID controller:
            # y[k] = y[k-1] + kc*(PV[k-1] - PV[k] +
            # Ts*e[k]/Ti +
            # Td/Ts*(2*PV[k-1] - PV[k] - PV[k-2]))
            #-----------------------------------------------------------
            self.pp = self.kc * (pidpy.xk_1 - xk) # y[k] = y[k-1] + Kc*(PV[k-1] - PV[k])
            self.pi = self.k0 * ek  # + Kc*Ts/Ti * e[k]
            self.pd = self.k1 * (2.0 * pidpy.xk_1 - xk - pidpy.xk_2)
            pidpy.yk += self.pp + self.pi + self.pd
        else:
            pidpy.yk = 0.0
            self.pp = 0.0
            self.pi = 0.0
            self.pd = 0.0

        #update the "past two" cycles for next cycle.
        pidpy.xk_2 = pidpy.xk_1  # PV[k-2] = PV[k-1]
        pidpy.xk_1 = xk    # PV[k-1] = PV[k]
        
        # limit y[k] to GMA_HLIM and GMA_LLIM
        if (pidpy.yk > pidpy.GMA_HLIM):
            pidpy.yk = pidpy.GMA_HLIM
        if (pidpy.yk < pidpy.GMA_LLIM):
            pidpy.yk = pidpy.GMA_LLIM

        #get you some!
        return pidpy.yk
        

if __name__=="__main__":

    sampleTime = 2
    pid = pidpy(sampleTime,0,0,0)
    temp = 80
    setpoint = 100
    enable = True
    print pid.calcPID_reg4(temp, setpoint, enable)

## I never see RasPiBrew use this function so I'm setting it aside for now.
##
##def calcPID_reg3(self, xk, tset, enable):
##        ek = 0.0
##        lpf = 0.0
##        ek = tset - xk # calculate e[k] = SP[k] - PV[k]
##        #--------------------------------------
##        # Calculate Lowpass Filter for D-term
##        #--------------------------------------
##        lpf = self.lpf1 * pidpy.lpf_1 + self.lpf2 * (ek + pidpy.ek_1);
##        
##        if (enable):
##            #-----------------------------------------------------------
##            # Calculate PID controller:
##            # y[k] = y[k-1] + kc*(e[k] - e[k-1] +
##            # Ts*e[k]/Ti +
##            # Td/Ts*(lpf[k] - 2*lpf[k-1] + lpf[k-2]))
##            #-----------------------------------------------------------
##            self.pp = self.kc * (ek - pidpy.ek_1) # y[k] = y[k-1] + Kc*(PV[k-1] - PV[k])
##            self.pi = self.k0 * ek  # + Kc*Ts/Ti * e[k]
##            self.pd = self.k1 * (lpf - 2.0 * pidpy.lpf_1 + pidpy.lpf_2)
##            pidpy.yk += self.pp + self.pi + self.pd
##        else:
##            pidpy.yk = 0.0
##            self.pp = 0.0
##            self.pi = 0.0
##            self.pd = 0.0
##        
##        pidpy.ek_1 = ek # e[k-1] = e[k]
##        pidpy.lpf_2 = pidpy.lpf_1 # update stores for LPF
##        pidpy.lpf_1 = lpf
##            
##        # limit y[k] to GMA_HLIM and GMA_LLIM
##        if (pidpy.yk > pidpy.GMA_HLIM):
##            pidpy.yk = pidpy.GMA_HLIM
##        if (pidpy.yk < pidpy.GMA_LLIM):
##            pidpy.yk = pidpy.GMA_LLIM
##            
##        return pidpy.yk
