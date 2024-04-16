#include <Servo.h>
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

  int initRF=67;
  int initLF=67;
  int initRB=61;
  int initLB=63;
  int posRF=0;
  int posLF=0;
  int posRB=0;
  int posLB=0;
  void initServos();
  void steer(int valRF, int valLF, int valRB, int valLB, int speed);
  void goToSpeed(String direction, int targetSpeed, int currentSpeed);
  void keepRoverAtSpeed(String direction, int speed);

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

void Rover::goToSpeed(String direction, int targetSpeed, int currentSpeed){
  //acceleration
  if (targetSpeed>currentSpeed){
    for (int i=currentSpeed; i<=targetSpeed; i++){
      motorRF.sendPWM(direction,i);
      motorLF.sendPWM(direction,i);
      motorRM.sendPWM(direction,i);
      motorLM.sendPWM(direction,i);
      motorRB.sendPWM(direction,i);
      motorLB.sendPWM(direction,i);
      delay(5);
    }
  }
  //deceleration
  if (currentSpeed>targetSpeed){
    for(int i=currentSpeed; i>=targetSpeed; i--){
      motorRF.sendPWM(direction,i);
      motorLF.sendPWM(direction,i);
      motorRM.sendPWM(direction,i);
      motorLM.sendPWM(direction,i);
      motorRB.sendPWM(direction,i);
      motorLB.sendPWM(direction,i);
      delay(5);
    }
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
Rover Adversity;
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
      if(Adversity.posRF < (Adversity.initRF+35)){
	Adversity.steer(Adversity.posRF+5, Adversity.posLF+5,Adversity.posRB-5,Adversity.posLB-5,20);
      }
	previousTime=millis();
      while(millis()-previousTime <50){}
      digitalWrite(blueLed,LOW);
      digitalWrite(greenLed,LOW);
  }
  else if (command=="L"){
      digitalWrite(blueLed,HIGH);
      digitalWrite(greenLed,HIGH);
      if (Adversity.posRF > (Adversity.initRF-35)){
       Adversity.steer(Adversity.posRF-5, Adversity.posLF-5,Adversity.posRB+5,Adversity.posLB+5,20);
      }
      previousTime=millis();
      while(millis()-previousTime <50){}
      digitalWrite(blueLed,LOW);
      digitalWrite(greenLed,LOW);
  }
}
}
