import serial
import time
import controlLeg as cL
import numpy as np
import pandas as pd
serialcomm = serial.Serial('COM12', 921600)
serialcomm.timeout = 1
#           FR    BR    FL    BL
Pos = [0,   0,    434,  434,  0,
            700,  535,  300,  264,
            200,  250,  200,  200   ]
# 1 = FR,2 = BR,3 = FL,4 = BL
# RangePos = [[0,0],
#             [[215,800],[0,434]],
#             [[215,800],[0,434]],
#             [[0,649],[434,0]],
#             [[0,649],[434,0]]]
RangePos = [[0,0],
            [[468,658],[100,370]],
            [[468,685],[100,370]],
            [[400,180],[334,64]],
            [[400,156],[334,64]]]
Database = {"O1FR": [],"O2FR": [],"O1BL": [],"O2BL": [],"O1FL": [],"O2FL": [],"O1BR": [],"O2BR": [],"gFR1":[],"gFR2":[],"fFR1":[],"fFR2":[]}
ID = [0,1,2,3,4]
O_FR  = np.zeros((2,50))
O_BR  = np.zeros((2,50))
O_FL  = np.zeros((2,50))
O_BL  = np.zeros((2,50))
Tpeak = [0,0,0,0]
time_stack = [0] * 50
FR = cL.ControlLegRobot(ID[1],[Pos[5],Pos[9]], RangePos[ID[1]],"FR")
BR = cL.ControlLegRobot(ID[2],[Pos[6],Pos[10]],RangePos[ID[2]],"BR")
FL = cL.ControlLegRobot(ID[3],[Pos[7],Pos[11]],RangePos[ID[3]],"FL")
BL = cL.ControlLegRobot(ID[4],[Pos[8],Pos[12]],RangePos[ID[4]],"BL")

print(str(FR.RegisterID()))
print(str(BR.RegisterID()))
print(str(FL.RegisterID()))
print(str(BL.RegisterID()))

time_feedback = 0
current_feedback = [0,0,0,0]
pos_hip          = [0,0,0,0]
pos_knee         = [0,0,0,0]
res_pos          = [[0,0,0,0],[0,0,0,0]]
ESP_stack = bytearray()
def byteSend(hip,knee):
    packet = bytearray()
    packet.append(0xFF)
    packet.append(0xFF)
    packet.append(0x11)
    for i in range(4):
        dataH = hip[i] >> 8
        dataL = hip[i] & 0XFF
        packet.append(dataH)
        packet.append(dataL)
    for i in range(4):
        dataH = knee[i] >> 8
        dataL = knee[i] & 0XFF
        packet.append(dataH)
        packet.append(dataL)
    check_sum = sum(packet[2:])
    
    packet.append(check_sum &0xff)
    serialcomm.write(packet)

def main():
    global O_FR,O_BR,O_FL,O_BL,Tpeak,time_stack,time_feedback,ESP_stack,pos_hip,pos_knee,current_feedback,res_pos,Pos
    state_res = "start"
    if(state_res == "start"):
        print("Start Task")
        #test loop
        State_py = 0
        index = 0
        # byte
        Len = 0
        sum_data = 0
        State_Cal = 0
        start_time = time.time()
        count = 0
        # for i in range(100):
        while(True):
            end_time = time.time()                                                                          #   time :     hip   :    knee   :  current
            ESP_Req = serialcomm.read(1)
            if State_py == 0:   
                if ESP_Req ==  b'\xff' :
                    sum_data = 0
                    Len = 0
                    State_py = 1
            elif State_py == 1 :
                if ESP_Req ==  b'\xff' :
                    State_py = 2
                else:
                    State_py = 0
            elif State_py == 2:
                Len = int.from_bytes(ESP_Req, byteorder='big', signed=False)
                State_py = 3
                index = 0
            elif State_py == 3:
                if index < Len-1:
                    ESP_stack.append(int.from_bytes(ESP_Req, byteorder='big', signed=False))
                    index += 1
                else :
                    sum_data = int.from_bytes(ESP_Req, byteorder='big', signed=False)
                    index = 0
                    State_py = 4
                
            elif State_py == 4:  
                        
                check_sum = Len+sum(ESP_stack)
                check_sum = check_sum &0xff   
                if check_sum == sum_data and ESP_stack != b'':
                    State_py = 5
                    Len = 0
                else :
                    ESP_stack = bytearray()
                    State_py = 0
                    Len = 0
            elif State_py == 5:

                hip_FR = ESP_stack[0]
                hip_FR = hip_FR<<8
                hip_FR += ESP_stack[1]
                hip_BR = ESP_stack[2]
                hip_BR = hip_BR<<8
                hip_BR += ESP_stack[3]
                hip_FL = ESP_stack[4]
                hip_FL = hip_FL<<8
                hip_FL += ESP_stack[5]
                hip_BL = ESP_stack[6]
                hip_BL = hip_BL<<8
                hip_BL += ESP_stack[7]

                knee_FR = ESP_stack[8]
                knee_FR = knee_FR<<8
                knee_FR += ESP_stack[9]
                knee_BR = ESP_stack[10]
                knee_BR = knee_BR<<8
                knee_BR += ESP_stack[11]
                knee_FL = ESP_stack[12]
                knee_FL = knee_FL<<8
                knee_FL += ESP_stack[13]
                knee_BL = ESP_stack[14]
                knee_BL = knee_BL<<8
                knee_BL += ESP_stack[15]

                load_FR = ESP_stack[16]
                load_FR = load_FR<<8
                load_FR = load_FR & 0xFF00
                load_FR += ESP_stack[17]
                load_BR = ESP_stack[18]
                load_BR = load_BR<<8
                load_BR = load_BR & 0xFF00
                load_BR += ESP_stack[19]
                load_FL = ESP_stack[20]
                load_FL = load_FL<<8
                load_FL = load_FL &0xFF00
                load_FL += ESP_stack[21]
                load_BL = ESP_stack[22]
                load_BL = load_BL<<8
                load_BL = load_BL &0xFF00
                load_BL += ESP_stack[23]
                
                pos_hip = [hip_FR,hip_BR,hip_FL,hip_BL]
                pos_knee =[knee_FR,knee_BR,knee_FL,knee_BL]
                current_feedback = [load_FR,load_BR,load_FL,load_BL]
                State_py = 0
                State_Cal = 1
                ESP_stack = bytearray()
            if State_Cal == 1:
                State_Cal = 0
                time_feedback = (end_time-start_time)*1000
                print(time_feedback)
                # if count >= 1000 and count < 2000:
                #     current_feedback[0] = 0
                # count += 1
                resFR = FR.outputSeta(current_feedback[0],pos_knee[0],time_feedback)
                resBR = BR.outputSeta(current_feedback[1],pos_knee[1],time_feedback)
                resFL = FL.outputSeta(current_feedback[2],pos_knee[2],time_feedback)
                resBL = BL.outputSeta(current_feedback[3],pos_knee[3],time_feedback)
                res_pos = [[resFR[0],resBR[0],resFL[0],resBL[0]],[resFR[1],resBR[1],resFL[1],resBL[1]]]
                # res_pos = [[700,  535,  300,  264],[150,  250,  200,  235]]
                byteSend(res_pos[0],res_pos[1])
                Data_output = [FR.Output12(),BR.Output12(),FL.Output12(),BL.Output12()]
                
                #Pop list
                O_FR  = np.delete(O_FR, 0, axis=1)
                O_BR  = np.delete(O_BR, 0, axis=1) 
                O_FL  = np.delete(O_FL, 0, axis=1) 
                O_BL  = np.delete(O_BL, 0, axis=1)  
                
                #Append list
                Database["O1FR"].append(FR.Output12()[0][0])
                Database["O2FR"].append(FR.Output12()[0][1])
                Database["O1BL"].append(BL.Output12()[0][0])
                Database["O2BL"].append(BL.Output12()[0][1])
                Database["O1FL"].append(FL.Output12()[0][0])
                Database["O2FL"].append(FL.Output12()[0][1])
                Database["O1BR"].append(BR.Output12()[0][0])
                Database["O2BR"].append(BR.Output12()[0][1])
                Database["gFR1"].append(FR.GraphData()[0])
                Database["gFR2"].append(FR.GraphData()[1])
                Database["fFR1"].append(FR.Graphff()[0])
                Database["fFR2"].append(FR.Graphff()[1])
                count+=1
                O_FR = np.append(O_FR,np.array(Data_output[0]).reshape(-1,1),1)
                O_BR = np.append(O_BR,np.array(Data_output[1]).reshape(-1,1),1)
                O_FL = np.append(O_FL,np.array(Data_output[2]).reshape(-1,1),1)
                O_BL = np.append(O_BL,np.array(Data_output[3]).reshape(-1,1),1)
                time_stack.append(time_feedback)
                # print(np.argmax(O_FR),np.argmax(O_BR),np.argmax(O_FL),np.argmax(O_BL))
                Tpeak = [time_stack[np.argmax(O_FR[1])],time_stack[np.argmax(O_BR[1])],time_stack[np.argmax(O_FL[1])],time_stack[np.argmax(O_BL[1])]]
                time_stack.pop(0)
                FR.UpdateTpeak(Tpeak,Data_output)
                BR.UpdateTpeak(Tpeak,Data_output)
                FL.UpdateTpeak(Tpeak,Data_output)
                BL.UpdateTpeak(Tpeak,Data_output)
                # byteSend(res_pos[0],res_pos[1])
                # print(Data_output)
    else :
        print("End Task")

if __name__ == '__main__':
    try:
        main()
        serialcomm.close()
    except :
        df1 = pd.DataFrame(Database)
        df1.to_csv("outputFR.csv")  
        serialcomm.close()
        pass
    # main()