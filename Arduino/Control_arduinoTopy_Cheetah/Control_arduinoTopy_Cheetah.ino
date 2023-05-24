#include <SCServo.h>
String incomingByte ;    

SCSCL sc;
//                           FR    BR    FL    BL
    /*1*/int Pos[13] = { 0,  0,    434,  434,  0,
                             649,  649,  215,  215,
                             0,    0,    434,  434
                        };
    //Feedback Position
    int FeedPos[13] =  { 0,  0,    434,  434,  0,
                             649,  649,  215,  215,
                             0,    0,    434,  434
                      };
    //Current Knee
    double FeedCur[5]  = {
                         0,   0,    0,    0,    0
                      };          
void setup() {

  //Serial.begin(115200); //printf
  Serial1.begin(1000000); //communication to motor SCS
  Serial2.begin(921600); //commuication to notebook

  sc.pSerial = &Serial1;

  pinMode(LED_BUILTIN, OUTPUT);
  bool start = true;
  for(int i = 1;i <= 12;i++){
    sc.RegWritePos(i,Pos[i], 0, 300);
  }
  sc.RegWriteAction();
  delay(749);

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
    byte buff_input[120];
    int len = Serial2.readBytesUntil('\n', buff_input, sizeof(buff_input));
    buff_input[len - 1] = '\0';
    String incomingByte = String((char*)buff_input);
    String res = String(incomingByte[0])+String(incomingByte[1])+String(incomingByte[2]);
    if (incomingByte == "call_dat") {
      String txt = "";
      float time_value = (float)current_time*0.001;
      if (!isnan(time_value)) {
          txt += String(time_value);
      }
      String h = "";
      String k = "";
      String curr = "";
      for(int i = 1; i <= 4; i++) {
          if (FeedPos[i+4] != 0) {
              h += String(FeedPos[i+4]) + "/";
          }
          if (FeedPos[i+8] != 0) {
              k += String(FeedPos[i+8]) + "/";
          }
          if (FeedCur[i] >= 0) {
              curr += String(FeedCur[i]) + "/";
          }
          else{
              curr += String(0) + "/";
          }
      }
      txt += ":" + h + "0:" + k + "0:" + curr + "0";
      byte feedback[txt.length()+1];
      txt.getBytes(feedback, txt.length()+1);
      Serial2.write(feedback, sizeof(feedback));
  
  }else if(res == "res"){
      String txt = "";
      int layer = 0;
      int IDlayer1 = 1;
      int IDlayer2 = 1;
      for(int i = 3;i <= incomingByte.length();i++){
        String sig = String(incomingByte[i]);
        if(layer == 0 && sig != ":"){
          if(sig != "/"){
            txt += sig;
          }
          else {   
            Pos[IDlayer1+4] = txt.toInt();
            txt = "";
            IDlayer1++;
          }
        }
        else if(layer == 1 && sig != ":"){
          if(sig != "/"){
            txt += sig;
          }
          else {
            Pos[IDlayer2+8] = txt.toInt();
            txt = "";
            IDlayer2++;
          }
        }
        if(sig == ":"){
          layer = 1;
        }
      }
      // while(1){
      //   for(int i = 1;i <= 12;i++){
      //     Serial.println(String(Pos[i])+"/");
      //   }
      //   delay(1000);
      // }
      
      //Control Motor
      for(int i = 5;i <= 12;i++){
        sc.WritePos(i,Pos[i], 0, 500);
            // bool tf = true;
            // while(tf){
            //     if(sc.FeedBack(i) != -1){
            //       FeedPos[i] = sc.ReadPos(-1);
            //       if(i >= 5 && i <= 8){
            //         FeedCur[i-4] = (double)abs(sc.ReadLoad(-1)/700);
            //       }
            //       tf = false;
            //     }
            //     delay(10);
            // }
      }
    
      //Control Motor
    }
    
  }
  for(int i = 1;i <= 12;i++){
    bool tf = true;
    while(tf){
        if(sc.FeedBack(i) != -1){
          FeedPos[i] = sc.ReadPos(-1);
          if(i >= 5 && i <= 8){
            FeedCur[i-4] = (double)abs(sc.ReadLoad(-1)/700);
          }
          tf = false;
        }
        delay(10);
    }
  }

}