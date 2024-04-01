#include <Servo.h>
int steerRadius=0;
int steerDirection;
int batteryProbe=A0;
int batteryPulse=26;
float batteryFlag;
String command;
long unsigned previousTime=0;
class DCMotor {
  public:
  int IN1;
  int IN2;
  void sendPWM(String direction, int value);
  
  DCMotor(int x, int y){
    IN1=x;
    IN2=y;
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);
  }
};

void DCMotor::sendPWM(String direction, int value){
  	if (direction=="forward"){
      digitalWrite(IN1,LOW);
      analogWrite(IN2,value);
  	}
  	else if (direction=="backward"){
      digitalWrite(IN2,LOW);
      analogWrite(IN1,value);
  	}
    else {
      Serial.println("Wrong entry for direction ");
    }
}
Servo wheelRF;
Servo wheelLF;
Servo wheelRB;
Servo wheelLB;

class Rover{
  public:

  int initRF=70;
  int initLF=67;
  int initRB=67;
  int initLB=67;
  int posRF=0;
  int posLF=0;
  int posRB=0;
  int posLB=0;
  void initServos();
  void steer(int valRF, int valLF, int valRB, int valLB, int speed);
  void goToSpeed(String direction, int targetSpeed, int currentSpeed);
  void keepRoverAtSpeed(String direction, int speed);
  void speedSteer(String direction, String side, int targetRadius);

};

DCMotor motorRF(46,10);//U7
DCMotor motorLF(2,3);
DCMotor motorRM(13,44);
DCMotor motorLB(4,5);//changed
DCMotor motorLM(8,9);//changed
DCMotor motorRB(7,6);//changed

void Rover::initServos(){
  wheelRF.attach(25);
  wheelLF.attach(23);
  wheelRB.attach(24);
  wheelLB.attach(22);
  delay(25);
  wheelRF.write(initRF);
  posRF=initRF;
  delay(25);
  wheelLF.write(initLF);
  posLF=initLF;
  delay(25);
  wheelRB.write(initRB);
  posRB=initRB;
  delay(25);
  wheelLB.write(initLB);
  posLB=initLB;
  delay(25);
  //delays to avoid current peak
}

void Rover::steer(int valRF, int valLF, int valRB, int valLB, int speed){
  int incrementRF;
  int incrementLF;
  int incrementRB;
  int incrementLB;
  if (valRF > posRF){
    incrementRF=1;}
  else {
    incrementRF=-1;}
  if (valLF > posLF){
    incrementLF=1;}
  else {
    incrementLF=-1;}
  if (valRB > posRB){
    incrementRB=1;}
  else {
    incrementRB=-1;}
  if (valLB > posLB){
    incrementLB=1;}
  else {
    incrementLB=-1;}
  while(posRF != valRF || posLF != valLF || posRB != valRB || posLB != valLB){
    
    if(posRF != valRF){
      posRF=posRF+incrementRF;
      wheelRF.write(posRF);
    }
    if(posLF != valLF){
      posLF=posLF+incrementLF;
      wheelLF.write(posLF);
    }
    if(posRB != valRB){
      posRB=posRB+incrementRB;
      wheelRB.write(posRB);
    }
    if(posLB != valLB){
      posLB=posLB+incrementLB;
      wheelLB.write(posLB);
    }
    delay(speed);
  }

}



void Rover::keepRoverAtSpeed(String direction, int speed){
  motorRF.sendPWM(direction,speed);
  motorLF.sendPWM(direction,speed);
  motorRM.sendPWM(direction,speed);
  motorLM.sendPWM(direction,speed);
  motorRB.sendPWM(direction,speed);
  motorLB.sendPWM(direction,speed);
}

void Rover::speedSteer(String direction, String side, int targetRadius){
  int speedRF;
  int speedLF;
  int speedRM;
  int speedLM;
  int speedRB;
  int speedLB;
  if (side=="R"){
    float radiusLF=targetRadius+Adversity_Calculations.LFx;
    float radiusRF=targetRadius-Adversity_Calculations.RFx;
    float radiusLB=targetRadius+Adversity_Calculations.LBx;
    float radiusRB=targetRadius-Adversity_Calculations.RBx;
    float radiusRM=targetRadius-Adversity_Calculations.Mx;
    float radiusLM=targetRadius+Adversity_Calculations.Mx;
    //in case of right turn, LF has the most to travel
    speedLM=255;//tinker to put the maximum value to match mapping accounting for deviation of motors speed
    float speed_RF=(radiusRF/radiusLM)*speedLM;
    float speed_LB=(radiusLB/radiusLM)*speedLM;
    float speed_RB=(radiusRB/radiusLM)*speedLM;
    float speed_RM=(radiusRM/radiusLM)*speedLM;
    float speed_LF=(radiusLF/radiusLM)*speedLM;
    speedRF=map(speed_RF, 0, 255, 0, 255);
    speedLB=map(speed_LB, 0, 255, 0, 255);
    speedRB=map(speed_RB, 0, 255, 0, 255);
    speedRM=map(speed_RM,0,255,0,255);
    speedLF=map(speed_LF,0,255,0,255);
  }
  else {
    float radiusLF=targetRadius-Adversity_Calculations.LFx;
    float radiusRF=targetRadius+Adversity_Calculations.RFx;
    float radiusLB=targetRadius-Adversity_Calculations.LBx;
    float radiusRB=targetRadius+Adversity_Calculations.RBx;
    float radiusRM=targetRadius+Adversity_Calculations.Mx;
    float radiusLM=targetRadius-Adversity_Calculations.Mx; 
    //in case of left turn, RF has the most to travel
    speedRM=255;//to tinker
    float speed_LF=(radiusLF/radiusRM)*speedRM;
    float speed_LB=(radiusLB/radiusRM)*speedRM;
    float speed_RB=(radiusRB/radiusRM)*speedRM;
    float speed_RF=(radiusRF/radiusRM)*speedRM;
    float speed_LM=(radiusLM/radiusRM)*speedRM;
    speedRF=map(speed_RF, 0, 255, 0, 255);
    speedLB=map(speed_LB, 0, 255, 0, 255);
    speedRB=map(speed_RB, 0, 255, 0, 255);
    speedLM=map(speed_LM,0,255,0,255);
    speedLF=map(speed_LF,0,255,0,255);
  }
  motorLF.sendPWM(direction,speedLF);
  motorRF.sendPWM(direction,speedRF);
  motorRM.sendPWM(direction,speedRM);
  motorLM.sendPWM(direction,speedLM);
  motorRB.sendPWM(direction,speedRB);
  motorLB.sendPWM(direction,speedLB); 
}

class Rover_Calculations{
  public:
  //distances from centers of wheels to center of steering
    float LFy=274.28;
    float RFy=274.28;
    float RBy=133;
    float LBy=133;

    float LFx=184;
    float RFx=184;
    float RBx=133;
    float LBx=133;
    float Mx=254;
    int getTargetAngleLF(String direction, float targetRadius);
    int getTargetAngleRF(String direction, float targetRadius);
    int getTargetAngleLB(String direction, float targetRadius);
    int getTargetAngleRB(String direction, float targetRadius);
};
int Rover_Calculations::getTargetAngleLF(String direction, float targetRadius){
  float LF_Radius;
  float angleDecimal;
  int angle;
  if (direction=="R"){
    LF_Radius=targetRadius+LFx;
    angleDecimal=degrees(asin(LFy/LF_Radius));
    angle=map(angleDecimal, 0, 180, 0, 180);
  }
  else {
    LF_Radius=targetRadius-LFx;
    angleDecimal=degrees(asin(LFy/LF_Radius));
    angle=map(angleDecimal, 0, 180, 0, 180);
    angle=angle*(-1);
    //when steering right, the angle is substracted
  }
  return angle;
}
int Rover_Calculations::getTargetAngleRF(String direction, float targetRadius){
  float RF_Radius;
  float angleDecimal;
  int angle;
  if (direction=="R"){
    RF_Radius=targetRadius-RFx;
    angleDecimal=degrees(asin(RFy/RF_Radius));
    angle=map(angleDecimal, 0, 180, 0, 180);
  }
  else {
    RF_Radius=targetRadius+RFx;
    angleDecimal=degrees(asin(RFy/RF_Radius));
    angle=map(angleDecimal, 0, 180, 0, 180);
    angle=angle*(-1);
  }
  return angle;
}
int Rover_Calculations::getTargetAngleLB(String direction, float targetRadius){
  float LB_Radius;
  float angleDecimal;
  int angle;
  if (direction=="R"){
    LB_Radius=targetRadius+LBx;
    angleDecimal=degrees(asin(LBy/LB_Radius));
    angle=map(angleDecimal, 0, 180, 0, 180);
    angle=angle*(-1);
  }  
  else {
    LB_Radius=targetRadius-LBx;
    angleDecimal=degrees(asin(LBy/LB_Radius));
    angle=map(angleDecimal, 0, 180, 0, 180);
  }
}
int Rover_Calculations::getTargetAngleRB(String direction, float targetRadius){
  float RB_Radius;
  float angleDecimal;
  int angle;
  if (direction=="R"){
    RB_Radius=targetRadius-RBx;
    angleDecimal=degrees(asin(RBy/RB_Radius));
    angle=map(angleDecimal, 0, 180, 0, 180);
    angle=angle*(-1);
  }  
  else {
    RB_Radius=targetRadius+RBx;
    angleDecimal=degrees(asin(RBy/RB_Radius));
    angle=map(angleDecimal, 0, 180, 0, 180);
  }
}
Rover Adversity;
Rover_Calculations Adversity_Calculations;
int redLed=48;
int greenLed=47;
int blueLed=49;

void setup()
{
  Serial.begin(115200);
  Adversity.initServos();
  delay(500);
  pinMode(redLed,OUTPUT);
  pinMode(greenLed,OUTPUT);
  pinMode(blueLed,OUTPUT);
  pinMode(batteryProbe,INPUT);
  pinMode(batteryPulse,OUTPUT);
  digitalWrite(redLed,LOW);
  digitalWrite(blueLed,LOW);
  
  for (int i=0; i<5; i++){
      digitalWrite(greenLed,HIGH);
      delay(300);
      digitalWrite(greenLed,LOW);
      delay(300);
  }
  //Serial.println("setup ready");  
}

void loop()
{
  Executor();
  batteryFlag=batteryValue();
  if (batteryFlag <= 10){
  digitalWrite(redLed,HIGH);
  }
}

float batteryValue(){
  float batteryRead;
  digitalWrite(batteryPulse,HIGH);
  batteryRead=analogRead(batteryProbe);
  batteryRead=(12/3.3)*batteryRead;
  return batteryRead; 
}

void Executor(){
  if(Serial.available()>0){
  command=Serial.readStringUntil('\r');
  Serial.println("received");
  if (command=="F"){
      digitalWrite(blueLed,HIGH);
      Adversity.keepRoverAtSpeed("forward", 255);
      while (Serial.available()<0){};
      digitalWrite(blueLed,LOW);
  }
  else if (command=="B"){
      digitalWrite(greenLed,HIGH);
      Adversity.keepRoverAtSpeed("backward",255);{}
      while (Serial.available()<0);
      digitalWrite(greenLed,LOW);
  }
  else if (command=="S"){
      digitalWrite(redLed,HIGH);
      Adversity.keepRoverAtSpeed("backward",0);{}
      while (Serial.available()<0);
      digitalWrite(redLed,LOW);
  }
  
  else if (command=="R"){
      digitalWrite(blueLed,HIGH);
      digitalWrite(greenLed,HIGH);
      Adversity.steer(Adversity.posRF+5, Adversity.posLF+5,Adversity.posRB-5,Adversity.posLB-5,20);
      previousTime=millis();
      while(millis()-previousTime <50){}
      digitalWrite(blueLed,LOW);
      digitalWrite(greenLed,LOW);
  }
  else if (command=="L"){
      digitalWrite(blueLed,HIGH);
      digitalWrite(greenLed,HIGH);
      Adversity.steer(Adversity.posRF-5, Adversity.posLF-5,Adversity.posRB+5,Adversity.posLB+5,20);
      previousTime=millis();
      while(millis()-previousTime <50){}
      digitalWrite(blueLed,LOW);
      digitalWrite(greenLed,LOW);
  }
}
}
