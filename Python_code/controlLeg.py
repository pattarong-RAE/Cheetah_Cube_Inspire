import math
import numpy as np
import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class ControlLegRobot:
    def __init__(self,ID_leg,position,rangePosition,name):
        self.ID_leg         = ID_leg-1
        self.Name           = name
        self.position       = {"knee":position[0],"hip":position[1]}
        self.rangePosition  = {"knee":rangePosition[0],"hip":rangePosition[1]}
        self.force          = 0
        self.MI             = 0
        self.O              = np.zeros((4,2))
        self.fi             = [0,0]
        self.T_Provide      = 0
        self.T              = 0
        
    #https://www.frontiersin.org/articles/10.3389/fncir.2023.1111285/full?fbclid=IwAR39kpClk_bYCKoVDMag4O4GsVFzs1NNgR1Hh68WghoB6_AApWWkb1-f_8A#B57
    #Parameter " Adaptive physical communication "
        self.seta1Provide   = self.position["hip"]
        self.seta1Current   = 0
        self.alpha          = 0.9
        self.p              = 0.99 
        self.F_eCurrent     = 0

        #y()
        self.Kf_y           = 0
        self.Ks_y           = 0
        self.Af_y           = 0.57
        self.As_y           = 0.99
        self.Bf_y           = 0.002
        self.Bs_y           = 0.0002
    #Parameter " Adaptive neural communication "
        self.CID            = np.zeros((50,4))
        self.tpeak          = [0,0,0,0]
        self.Outputpeak     = [0,0,0,0]
        self.N              = 50
        self.sigma          = 0.4
        self.NCount         = 0
        self.g_n            = [0,0]
        self.En             = 0.01    
    #Parameter FMi(t)  is the predicted foot contact sensor
        self.miwFMi         = 0.3
        self.wFMi           = 0.5
        self.FMi            = 0
        self.motor          = 0     #Knee
    #Plot Graph
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.xs = []
        self.ys = []
    def __str__(self):
        return f"ID : {self.ID_leg} {self.Name} > Knee : {self.position['knee']} and Hip : {self.position['hip']}"
    #" Adaptive physical communication "
    def G(self,seta1Current):
        if seta1Current <= self.seta1Provide:
            return 1
        else :
            return 0
    def F_e(self,seta1Current):
        self.F_eCurrent = self.alpha*((self.p*self.G(seta1Current))+((1-self.p)*self.F_eCurrent))
        self.seta1Provide = seta1Current
        return self.F_eCurrent
    def Ks(self,e):
        AsKs = self.As_y*self.Ks_y
        Bse  = self.Bs_y*e
        self.Ks_y = AsKs+Bse
        return self.Ks_y
    def Kf(self,e):
        AfKf = self.Af_y*self.Kf_y
        Bfe  = self.Bf_y*e
        self.Kf_y = AfKf+Bfe
        return self.Kf_y
    def e(self,seta1Current):
        F = self.Fm()-self.F_e(seta1Current)
        if F <= 0:
            F = 0
        return F
    def y(self):
        seta1Current = self.seta1Current
        e = e(seta1Current)
        K = self.Kf(e)+self.Ks(e)
        return K
    def f(self,motor,seta1Current):
        self.motor = motor
        self.seta1Current = seta1Current
        y = self.y()
        Fm = self.Fm()
        self.f[0] =  y*Fm*math.cos(self.O[self.ID_leg][0])
        self.f[1] =  y*Fm*math.sin(self.O[self.ID_leg][1])
        return self.f
    #" Adaptive neural communication "
    def CID_lk(self,l,k):
        if(l == k):
            return 0
        t = self.T
        tl_k  = self.tpeak[l]-self.tpeak[k]
        tl_klT= tl_k/(t-self.T_Provide)
        self.T_Provide = t
        CIDlk = tl_klT*2*math.pi
        return CIDlk
    def CID_Mean(self):
        CIDSum = np.sum(self.CID[0:self.N-1],axis=0)
        CIDMean = CIDSum/self.N
        return CIDMean
    def K(self):
        CIDGen = [[self.CID_lk(self.ID_leg,0),self.CID_lk(self.ID_leg,1),self.CID_lk(self.ID_leg,2),self.CID_lk(self.ID_leg,3)]]
        self.CID = self.CID[1:]
        self.CID = np.append(self.CID,CIDGen,axis=0)
        
        if self.NCount >= self.N:   
            d = self.dn()
            if d <= self.sigma:
                return 1
            else :
                return 0
        else :
            self.NCount += 1
            return 0
    def dn(self):
        CIDMean  = self.CID_Mean()
        CID_sum  = 0
        CID_sub  = np.subtract(self.CID[50],CIDMean)
        for i in range(0,4):
            CID_sum += abs(CID_sub[i])**2
        d = math.sqrt(CID_sum)
        return d
    def gn(self,T):
        self.T = T
        g = [0,0]
        K = self.K()
        sum_gn = [0,0]
        for k in range(0,4):
            for i in range(0,2):
                O_sum = self.O[self.ID_leg][i]-self.O[k][i]-self.CID[50][k]
                sum_gn[i] += O_sum
        for i in range(0,2):
            g[i] = K*self.En*sum_gn[i]
        return g
        
    #FMi(t)  is the predicted foot contact sensor https://ieeexplore.ieee.org/document/9088155
    def Fm(self):
        motor = self.motor
        wmotor = self.wFMi*motor
        I_WFmi = (1-self.wFMi)*self.FMi
        self.FMi = self.miwFMi*(I_WFmi+wmotor)
        return self.FMi
    #plot graph https://learn.sparkfun.com/tutorials/graph-sensor-data-with-python-and-matplotlib/update-a-graph-in-real-time
    def animate(self,i, xs, ys):
        ax = self.ax
        # Read temperature (Celsius) from TMP102
        temp_c = round(tmp102.read_temp(), 2)

        # Add x and y to lists
        xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
        ys.append(temp_c)

        # Limit x and y lists to 20 items
        xs = xs[-20:]
        ys = ys[-20:]

        # Draw x and y lists
        ax.clear()
        ax.plot(xs, ys)

        # Format plot
        plt.xticks(rotation=45, ha='right')
        plt.subplots_adjust(bottom=0.30)
        plt.title('TMP102 Temperature over Time')
        plt.ylabel('Temperature (deg C)')
    def plotGraph(self):
        ani = animation.FuncAnimation(self.fig, self.animate, fargs=(self.xs, self.ys), interval=1000)
        plt.show()
