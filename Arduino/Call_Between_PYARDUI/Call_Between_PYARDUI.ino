#include <SCServo.h>
String incomingByte ;    

SCSCL sc;
//                       FR    BR    FL    BL
/*1*/int Pos[13] = { 0,  0,    434,  434,  0,
/*2*/                    215,  215,  215,  215,
/*3*/                    0,    0,    434,  434
                    };
void setup() {

  Serial.begin(115200);
  Serial1.begin(1000000);
  Serial2.begin(115200);
  sc.pSerial = &Serial1;
  pinMode(LED_BUILTIN, OUTPUT);
  bool start = true;
  while(start){
    if (Serial2.available() > 0) {
      incomingByte = Serial2.readStringUntil('\n');
      if(incomingByte == "start"){
        Serial2.write("start");
        start = false;
      }
      else{
        Serial2.write("end");
      } 
    }
  }

}
void loop() {
  
  if (Serial2.available() > 0) {
    unsigned long current_time = millis();
    incomingByte = Serial2.readStringUntil('\n');
    String str_res = String(incomingByte);
    String res = incomingByte[0]+incomingByte[1]+incomingByte[2]
    if(incomingByte == "call_data"){
      Serial2.print(String(current_time));
    }
    else if(res == "res"){
      String txt = "";
      int layer = 0;
      int IDlayer1 = 1;
      int IDlayer2 = 1;
      for(int i = 4;i <= str_res.length();i++){
        if(layer == 0 && str_res[i] != ":"){
          if(str_res[i] != "/"){
            txt += str_res[i];
          }
          else {   
            Pos[IDlayer1+4] = txt.toInt();
            txt = "";
            IDlayer1++;
          }
        }
        else {
          if(str_res[i] != "/"){
            txt += str_res[i];
          }
          else {
            Pos[IDlayer2+8] = txt.toInt();
            txt = "";
            IDlayer2++;
          }
        }
        if(str_res[i] == ":"){
          layer = 1;
        }
      }
    }
  }

}