#include<positionalnew.h>
UniversalEncoder myEnc(2,3,1);//Create Encoder Object from its pin(En1, En2 , DirectionalOffset)
Motor m1(9,5);//Create Motor Object (Pwm pin, Dir pin1, Dir pin2 (Optional))
positionalnew p1(&m1);
double AggKp=1,AggKi=0,Aggkd=0;
double SoftKp=1,SoftKi=0,Softkd=0;

int min=-255,max=255,targetpulse=0;
void setup() {
  m1.setEncoder(&myEnc);
  m1.setEncoder(2,3,-1);
  p1.setThreshold(500);
 Serial.begin(115200);
  p1.setOutputLimits( min, max);
  p1.setAggTunings(AggKp, AggKi, Aggkd);
  p1.setSoftTunings(SoftKp, SoftKi, Softkd);
  
  

}

void loop() {
  if(Serial.available()>0)
  {
    targetpulse=Serial.parseInt();
    p1.setPulse(targetpulse);
  }
  p1.compute();

}
