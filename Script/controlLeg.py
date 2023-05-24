import math
import numpy as np


 

class ControlLegRobot:
    def __init__(self,ID_leg,position,rangePosition,name):
        self.yG = []
        self.yGG = []
        self.ID_leg         = ID_leg-1
        self.Name           = name
        self.position       = {"hip":position[0],"knee":position[1]}
        self.rangePosition  = {"hip":[rangePosition[0][0],rangePosition[0][1]],"knee":[rangePosition[1][0],rangePosition[1][1]]} #[knee][n]  if n = 0 min,n = 1 max    
        self.MI             = 0.4

        self.O              = np.array([[-0.75,0.8],[0.75,-0.2],[0.75,-0.2],[-0.75,0.8]]) #[[0.5,0.5],[0.5,0.5],[0.5,0.5],[0.5,0.5]]
        self.O_Provide      = 0
        self.T_Provide      = 0
        self.T              = 0
        self.bias           = 0
        self.wnn            = [[1.55,0.25+self.MI],[-(0.25+self.MI),1.55]] #[[0,0],[0,0]]
        self.dir            = True if self.rangePosition["knee"][0] > self.rangePosition["knee"][1] else False 
        self.count          = 0
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
        self.N              = 49
        self.sigma          = 0.4
        self.NCount         = 0
        self.g_n            = [0,0]
        self.En             = 0.01
        self.period         = 0 
    #Parameter FMi(t)  is the predicted foot contact sensor
        self.miwFMi         = 0.3
        self.wFMi           = 0.5
        self.FMi            = 0
        self.motor          = 0     #Knee
    #Parameter Data use out Class
        self.Output         = [0,0]
    def RegisterID(self):
        return f"ID : {self.ID_leg+1} {self.Name} > Knee : {self.position['knee']} and Hip : {self.position['hip']}"
    #CPG Output
    def map_value(self,sensor_value, in_min, in_max, out_min, out_max):
        return int((sensor_value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
    def outputSeta(self,motor,seta1Current,tm):
        self.seta1Current   = seta1Current
        self.motor          = motor/900
        self.T              = tm
        seta    = np.array([0,0])
        self.Timeperiod(self.O[self.ID_leg][1]-self.Output[1])
        # if self.count >= 500 and self.count <= 800 and self.ID_leg == 0:
        #     self.O[self.ID_leg][0] = 0.7
        #     self.O[self.ID_leg][1] = 0.2
        # if self.count >= 500 and self.count <= 800 and self.ID_leg == 3:
        #     self.O[self.ID_leg][0] = 0.7
        #     self.O[self.ID_leg][1] = 0.2
        # if self.count >= 50 and self.count <= 0 and self.ID_leg == 0:
        #     self.O[self.ID_leg][0] = 0.4
        #     self.O[self.ID_leg][1] = 0.9
        # if self.count >= 50 and self.count <= 0 and self.ID_leg == 3:
        #     self.O[self.ID_leg][0] = 0.4
        #     self.O[self.ID_leg][1] = 0.9

        a       = self.outputNeural()
        O       = np.tanh(a)
        self.count += 1
        seta[0] = self.map_value(O[0], -1, 1, self.rangePosition["hip"][0],self.rangePosition["hip"][1])
        seta[1] = self.map_value(O[1], -1, 1, self.rangePosition["knee"][0],self.rangePosition["knee"][1])
        
        self.T_Provide      = self.T
        self.seta1Provide   = seta1Current 
        self.Output         = O
        self.O[self.ID_leg][0] = O[0]
        self.O[self.ID_leg][1] = O[1]

        return seta
    def outputNeural(self):
        # O1 = w11*O1+w12*O2-f1+gn
        # O2 = w22*O2+w21*O1-f2+gn
        #Parameter motor,seta1Current,t "realtime"
        a       = [0,0]
        f       = self.f()
        g       = self.gn()
        self.yGG= [f[0],f[1]]
        self.yG = [g[0],g[1]]
        a[0]    = self.wnn[0][0]*self.O[self.ID_leg][0]+self.wnn[0][1]*self.O[self.ID_leg][1]-f[0]+g[0]
        a[1]    = self.wnn[1][1]*self.O[self.ID_leg][1]+self.wnn[1][0]*self.O[self.ID_leg][0]-f[1]+g[1]
        #Warning Insert Output 1-4 O1,O2 
        return a
    #" Adaptive physical communication "
    def G(self,seta1Current):
        seta1Provide = self.seta1Provide
        if self.dir == False :
            seta1Current = self.rangePosition["knee"][0]-seta1Current
            seta1Provide = self.rangePosition["knee"][0]-seta1Provide
        if seta1Current <= seta1Provide:
            return 1
        else :
            return 0
    def F_e(self,seta1Current):
        self.F_eCurrent = self.alpha*((self.p*self.G(seta1Current))+((1-self.p)*self.F_eCurrent))
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
        F = (self.motor)-self.F_e(seta1Current) #Fa-Fe
        # F = (self.motor*0.9)-self.Fm()
        if F <= 0:
            F = 0
        return F
    def y(self):
        e = self.e(self.seta1Current)
        
        K = self.Kf(e)+self.Ks(e)
        return K
    def f(self):
        y = self.y()
        Fn = self.motor
        f = [0,0]
        f[0] =  y*Fn*math.cos(self.O[self.ID_leg][0])
        f[1] =  y*Fn*math.sin(self.O[self.ID_leg][1])
        return f
    #" Adaptive neural communication "
    def CID_lk(self,l,k):
        if(l == k):
            return 0
        tl_k  = self.tpeak[l]-self.tpeak[k]
        tl_klT= tl_k/self.period
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
        CIDMean  = self.CID_Mean() #[CIDMean,CIDMean,CIDMean,CIDMean]
        CID_sum  = 0
        CID_sub  = np.subtract(self.CID[self.N],CIDMean) #[self.CID[50],self.CID[50],self.CID[50],self.CID[50]] - [CIDMean,CIDMean,CIDMean,CIDMean]
        #Frobenius norm###
        for i in range(0,4):
            CID_sum += abs(CID_sub[i])**2
        d = math.sqrt(CID_sum)
        ##################
        return d
    def gn(self):
        g = [0,0]
        K = self.K()
        sum_gn = [0,0]
        for k in range(0,4):
            for i in range(0,2):
                O_sum = self.O[self.ID_leg][i]-self.O[k][i]-self.CID[self.N][k]
                sum_gn[i] += math.sin(O_sum)
        for i in range(0,2):
            g[i] = K*self.En*sum_gn[i]
            print(g[i])
            
        return g
    def Timeperiod(self,difO):
        # calculate the time difference between the two data points
        time_difference = self.T - self.T_Provide
        # calculate the angular frequency of the wave
        angular_frequency = math.acos(difO / 2) / time_difference
        # calculate the period of the wave
        self.period = 2 * math.pi / angular_frequency
        print("Time : ",self.period)
    #FMi(t)  is the predicted foot contact sensor https://ieeexplore.ieee.org/document/9088155
    def Fm(self):
        motor = self.motor
        wmotor = self.wFMi*motor
        I_WFmi = (1-self.wFMi)*self.FMi
        self.FMi = self.miwFMi*(I_WFmi+wmotor)
        return self.FMi
    #Data to output
    def Output12(self):
        return [[self.Output[0],self.Output[1]]]
    def UpdateTpeak(self,Tpeak,outP):
        self.tpeak = Tpeak
        for i in range (4):
            self.O[i][0] = outP[i][0][0]
            self.O[i][1] = outP[i][0][1]
    def GraphData(self):
        return self.yG
    def Graphff(self):
        return self.yGG
    #plot graph https://learn.sparkfun.com/tutorials/graph-sensor-data-with-python-and-matplotlib/update-a-graph-in-real-time
