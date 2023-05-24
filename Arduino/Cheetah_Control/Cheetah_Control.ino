#define ID_MAX 12 
#define LEDpin 13
#include <SCServo.h>

SCSCL sc;
int Pos[13];
int Current[13];
int PosRes[13];
int Speed[13];
int ID[13];
void setup() {
  // put your setup code here, to run once:
  pinMode(LEDpin,OUTPUT);
	digitalWrite(LEDpin, HIGH);
	Serial1.begin(1000000);
  Serial.begin(115200);
  sc.pSerial = &Serial1;
  for(int i = 0;i <= ID_MAX;i++){
    Pos[i]      = 0;
    Current[i]  = 0;
    PosRes[i]   = 0;
    Speed[i]    = 0;
    ID[i]       = i;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i = 1;i <= ID_MAX;i++){
    if(sc.FeedBack(i)!=-1){
      Pos[i] = sc.ReadPos(-1);
      Current[i] = sc.ReadCurrent(-1);
      Serial.print("ID >> "+String(i));
      Serial.print("Position : ");
      Serial.println(Pos[i]);
      Serial.print("Current  : ");
      Serial.println(Current[i]);
    }
  }
  /*
    Send data between python and arduino
  */
  sc.SyncWritePos(ID, ID_MAX, PosRes, 0, Speed);
}
