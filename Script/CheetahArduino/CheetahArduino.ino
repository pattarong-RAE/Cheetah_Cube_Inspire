#include <SCServo.h>

#define BUFFER_SIZE 40
//-------------------------------------------------------------
//MOTOR parameter
SCSCL sc;
//                           FR    BR    FL    BL
    /*1*/int Pos[13] = { 0,  15,    419,  400,  15,
                             649,  649,  215,  215,
                             0,    0,    434,  434
                        };
    int StayPos[13] = { 0,  15,    419,  400,  15,
                            700,  535,  300,  264,
                            200,  250,  200,  200
                        };
    //Feedback Position
    int FeedPos[13] =  { 0,  0,    434,  434,  0,
                             649,  649,  215,  215,
                             0,    0,    434,  434
                      };
    //Current Knee
    double FeedLoad[5]  = {
                         0,   0,    0,    0,    0
                      };     
//-------------------------------------------------------------

unsigned long PrevTime =0;
unsigned long Interval =25;
unsigned long currentTime;

unsigned char State_Serial =0;
unsigned char State_Data =0;
unsigned char Len =0;
unsigned char buff_data[BUFFER_SIZE];


void setup() {
  Serial1.begin(1000000); //communication to motor SCS
  Serial2.begin(921600); //commuication to notebook python ,max bus 921600
  Serial.begin(115200);//display data in arduino
  sc.pSerial = &Serial1;
//reset position
  pinMode(LED_BUILTIN, OUTPUT);
  for(int i = 1;i <= 12;i++){
    sc.RegWritePos(i,Pos[i], 0, 300);
  }
  sc.RegWriteAction();
  delay(749);
  for(int i = 1;i <= 12;i++){
    Pos[i] = StayPos[i];
    sc.RegWritePos(i,StayPos[i], 0, 300);
  }
  sc.RegWriteAction();
  delay(749);
  PrevTime = millis(); // Start Timer
//reset position
}
void byteUpdate(){
  Serial2.write(0xFF);//1
  Serial2.write(0xFF);//2
  unsigned int data_buffer = 0;
  unsigned int buffer_H;
  unsigned int buffer_L;
  Serial2.write(0x19);//3 Len
  unsigned char sum = 0x19;//0+Len
  for(int i = 5;i<=12;i++){
    data_buffer = FeedPos[i];
    buffer_H = data_buffer >> 8;//shift bit
    buffer_L = data_buffer & 0xFF;
    sum += buffer_H;
    sum += buffer_L;
    //2 byte per data
    Serial2.write(buffer_H);//byte1
    Serial2.write(buffer_L);//byte2
  }
  for(int i = 1;i<=4;i++){
    data_buffer = FeedLoad[i];
    buffer_H = data_buffer >> 8;//shift bit
    buffer_L = data_buffer & 0xFF;
    sum += buffer_H;
    sum += buffer_L;
    //2 byte per data
    Serial2.write(buffer_H);//byte1
    Serial2.write(buffer_L);//byte2
  }
  sum = sum & 0xff;
  Serial2.write(sum);
}
unsigned int count = 0;
void loop() {
  // put your main code here, to run repeatedly:

  static unsigned char index =0;

  
  if(Serial2.available() > 0) {
      switch(State_Serial){
        case 0:
          if (Serial2.read() == 0xFF){
             State_Serial++;
          }
          break;

        case 1:
          if(Serial2.read() == 0xFF)
          State_Serial++;
          Len=0;
          break;

        case 2:
          Len = Serial2.read();
          State_Serial++;
          index=0;
          break;

        case 3:
        //Len-1 = range data ["0-255" x 16] >> 4 Position Hip, 4 Position Knee
        //Loop = Len 
          if(index < (Len-1)){ //fetch data to buff_data
            buff_data[index] = Serial2.read();
            index++;
          }
          else{
            index = 0;
            State_Serial++;
          }
          break;

        case 4:
          unsigned char sum = Len;
          for(int i=0;i<Len;i++){
            sum += buff_data[i];
          }
          
          if(sum == Serial2.read()){
            State_Data = 1;
          
          }else{
            State_Data = 0;
            // Serial.println("sum != 0");
          }
          State_Serial=0; 
          break;

      }
  }

  currentTime = millis();
  if(currentTime-PrevTime > Interval){
    if(State_Data == 1){
      State_Data = 0;
      unsigned int hip_FR = buff_data[0];
      hip_FR = hip_FR<<8;
      hip_FR += buff_data[1];
      unsigned int hip_BR = buff_data[2];
      hip_BR = hip_BR<<8;
      hip_BR += buff_data[3];
      unsigned int hip_FL = buff_data[4];
      hip_FL = hip_FL<<8;
      hip_FL += buff_data[5];
      unsigned int hip_BL = buff_data[6];
      hip_BL = hip_BL<<8;
      hip_BL += buff_data[7];
      unsigned int knee_FR = buff_data[8];
      knee_FR = knee_FR<<8;
      knee_FR += buff_data[9];
      unsigned int knee_BR = buff_data[10];
      knee_BR = knee_BR<<8;
      knee_BR += buff_data[11];
      unsigned int knee_FL = buff_data[12];
      knee_FL = knee_FL<<8;
      knee_FL += buff_data[13];
      unsigned int knee_BL = buff_data[14];
      knee_BL = knee_BL<<8;
      knee_BL += buff_data[15]
      //Update Position Hip
      Pos[5]  = hip_FR;
      Pos[6]  = hip_BR;
      Pos[7]  = hip_FL;
      Pos[8]  = hip_BL;
      //Update Position Knee
      Pos[9]  = knee_FR;
      Pos[10] = knee_BR;
      Pos[11] = knee_FL;
      Pos[12] = knee_BL;

      String txt  = "";
      int Pos_Feed = 0;
      int Load_Feed = 0;
        sc.RegWritePos(1,Pos[1], 0, 700);
        sc.RegWritePos(2,Pos[2], 0, 700);
        sc.RegWritePos(3,Pos[3], 0, 700);
        sc.RegWritePos(4,Pos[4], 0, 700);
        sc.RegWriteAction();
          for(int i = 5;i <= 12;i++){
            //Move Position Hip and Knee
            sc.WritePos(i, Pos[i] , 0,700);
            Pos_Feed = sc.ReadPos(i);
            if(i >= 5 && i <= 8){
              Load_Feed = sc.ReadLoad(i);
              if(Load_Feed <0){
                Load_Feed =0;
              }
              FeedLoad[i-4] = abs(Load_Feed);
            }
            FeedPos[i] = Pos_Feed;
            // txt += String(Pos[i]);
          }
      }// end if state 0
      byteUpdate();
      PrevTime = millis();
      Serial.println(1);
    }
}
