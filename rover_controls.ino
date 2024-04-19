#include <Servo.h>
int steerCounter=0;
int steerRadius=0;
int targetRadiusList[]={2500,2000,1500,1000,800,700,600,500};
String steerDirection;
int batteryProbe=A0;
int batteryPulse=26;
float batteryFlag;
char isMooving='n';
String command;
long unsigned previousTime=0;
int debounceTime = 1200;
long unsigned int lastPress;
volatile bool failure=false;

long unsigned targetNav2;
String value="";
String cleanValue="";
int powers[4]={1000,100,10,1};
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
  return angle;
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
  return angle;
}
Rover_Calculations Adversity_Calculations;

class Rover{
  public:
  int maxRF=130;
  int minRF=80;
  int maxLF=180;
  int minLF=80;
  int maxRM=211;
  int minRM=80;
  int maxLM=233;
  int minLM=125;
  int maxRB=255;
  int minRB=90;
  int maxLB=155;
  int minLB=70;
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
  void speedSteer(String direction, String side, int targetRadius);

};

DCMotor motorRF(46,10);//U7
DCMotor motorLF(2,3);
DCMotor motorRM(44,13);
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
  if (targetRadius <= 800){
    maxRF=130*0.8;
    maxLF=180*0.8;
    maxRM=211*0.8;
    maxLM=223*0.8;
    maxRB=255*0.8;
    maxLB=155*0.8;
  }
  else {
    maxRF=130;
    maxLF=180;
    maxRM=211;
    maxLM=223;
    maxRB=255;
    maxLB=155;
  }  
  if (side=="R"){
    float radiusLF=targetRadius+Adversity_Calculations.LFx;
    float radiusRF=targetRadius-Adversity_Calculations.RFx;
    float radiusLB=targetRadius+Adversity_Calculations.LBx;
    float radiusRB=targetRadius-Adversity_Calculations.RBx;
    float radiusRM=targetRadius-Adversity_Calculations.Mx;
    float radiusLM=targetRadius+Adversity_Calculations.Mx;
    //in case of right turn, LM has the most to travel
    speedLM=maxLM;
    float speed_RF=(radiusRF/radiusLM)*speedLM;
    float speed_LB=(radiusLB/radiusLM)*speedLM;
    float speed_RB=(radiusRB/radiusLM)*speedLM;
    float speed_RM=(radiusRM/radiusLM)*speedLM;
    float speed_LF=(radiusLF/radiusLM)*speedLM;
    speedRF=map(speed_RF, 0, 255, minRF, maxRF);
    speedLB=map(speed_LB, 0, 255, minLB, maxLB);
    speedRB=map(speed_RB, 0, 255, minRB, maxRB);
    speedRM=map(speed_RM,0,255,minRM,maxRM);
    speedLF=map(speed_LF,0,255,minLF,maxLF);
  }
  else {
    float radiusLF=targetRadius-Adversity_Calculations.LFx;
    float radiusRF=targetRadius+Adversity_Calculations.RFx;
    float radiusLB=targetRadius-Adversity_Calculations.LBx;
    float radiusRB=targetRadius+Adversity_Calculations.RBx;
    float radiusRM=targetRadius+Adversity_Calculations.Mx;
    float radiusLM=targetRadius-Adversity_Calculations.Mx; 
    //in case of left turn, RM has the most to travel
    speedRM=maxRM;
    float speed_LF=(radiusLF/radiusRM)*speedRM;
    float speed_LB=(radiusLB/radiusRM)*speedRM;
    float speed_RB=(radiusRB/radiusRM)*speedRM;
    float speed_RF=(radiusRF/radiusRM)*speedRM;
    float speed_LM=(radiusLM/radiusRM)*speedRM;
    speedRF=map(speed_RF, 0, 255, minRF, maxRF);
    speedLB=map(speed_LB, 0, 255, minLB, maxLB);
    speedRB=map(speed_RB, 0, 255, minRB, maxRB);
    speedLM=map(speed_LM,0,255,minLM,maxLM);
    speedLF=map(speed_LF,0,255,minLF,maxLF);
  }
  motorLF.sendPWM(direction,speedLF);
  motorRF.sendPWM(direction,speedRF);
  motorRM.sendPWM(direction,speedRM);
  motorLM.sendPWM(direction,speedLM);
  motorRB.sendPWM(direction,speedRB);
  motorLB.sendPWM(direction,speedLB); 
}

Rover Adversity;
int redLed=48;
int greenLed=47;
int blueLed=49;

void setup()
{
  pinMode(21, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(21), ISR_button, FALLING);
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
  if (order.substring(0,1)=="R"){
      digitalWrite(blueLed,HIGH);
      digitalWrite(greenLed,HIGH);
      targetNav2=control(command);
      if(order.substring(5,6)=="R"){
        steerDirection="R";
        if (targetNav2 !=0){
          Adversity.steer(Adversity.initRF + (Adversity_Calculations.getTargetAngleRF(steerDirection,targetNav2)),
                          Adversity.initLF + (Adversity_Calculations.getTargetAngleLF(steerDirection,targetNav2)),
                          Adversity.initRB + (Adversity_Calculations.getTargetAngleRB(steerDirection,targetNav2)),
                          Adversity.initLB + (Adversity_Calculations.getTargetAngleLB(steerDirection,targetNav2)),
                          20);
        }
        else {
          Adversity.steer(Adversity.initRF, Adversity.initLF,Adversity.initRB,Adversity.initLB,20);
        }
      }
      else if(order.substring(5,6)=="L"){
        steerDirection="L";
        if (targetNav2 !=0){
          Adversity.steer(Adversity.initRF + (Adversity_Calculations.getTargetAngleRF(steerDirection,targetNav2)),
                          Adversity.initLF + (Adversity_Calculations.getTargetAngleLF(steerDirection,targetNav2)),
                          Adversity.initRB + (Adversity_Calculations.getTargetAngleRB(steerDirection,targetNav2)),
                          Adversity.initLB + (Adversity_Calculations.getTargetAngleLB(steerDirection,targetNav2)),
                          20);
        }
        else {
          Adversity.steer(Adversity.initRF, Adversity.initLF,Adversity.initRB,Adversity.initLB,20);
        }

      }
      else{}
      if (isMooving=='f' && steerCounter!=0){
        Adversity.speedSteer("forward",steerDirection,targetRadiusList[abs(steerCounter)-1]);
      }
      previousTime=millis();
      while(millis()-previousTime <50){}
      digitalWrite(blueLed,LOW);
      digitalWrite(greenLed,LOW);
  }

  if (command=="F"){
      digitalWrite(blueLed,HIGH);
      if (steerCounter==0){
        Adversity.keepRoverAtSpeed("forward", 255);
      }
      else {
        Adversity.speedSteer("forward",steerDirection,targetRadiusList[abs(steerCounter)-1]);
      }
      isMooving='f';
      while (Serial.available()<0){};
      digitalWrite(blueLed,LOW);
  }
  else if (command=="B"){
      digitalWrite(greenLed,HIGH);
      //if (steerCounter==0){
        Adversity.keepRoverAtSpeed("backward", 255);
      //}
      /*else {
        Adversity.speedSteer("backward",steerDirection,targetRadiusList[abs(steerCounter)-1]);
      }*/
      isMooving='b';
      while (Serial.available()<0){};
      digitalWrite(greenLed,LOW);
  }
  else if (command=="S"){
      digitalWrite(redLed,HIGH);
      Adversity.keepRoverAtSpeed("backward",0);{}
      isMooving='n';
      while (Serial.available()<0);
      digitalWrite(redLed,LOW);
  }
}
}

long unsigned control(String command){
  value=command.substring(1,6);
  int i=0;
  while(value.substring(i,i+1)=="0"){
    i++;
  }
  int cleanPowers[5-(i+1)];
  for (int j=0;j<(5-(i+1));j++){
    cleanPowers[j]=powers[i+j];
  }
  cleanValue=value.substring(i,5);
  long unsigned total=0;
  long unsigned nb=0;
  long unsigned conversion;
  for (int x=0;x<4-i;x++){
    conversion=stringToInt(cleanValue.substring(x,x+1));
    nb=conversion*cleanPowers[x];
    total=total+nb;
  }
  return total;
}

int stringToInt(String myStr){
  int output=0;
  
  if(myStr=="1"){
    output=1;}
  else if(myStr=="2"){
    output=2;}
  else if(myStr=="3"){
    output=3;}
  else if(myStr=="4"){
    output=4;}
  else if(myStr=="5"){
    output=5;}
  else if(myStr=="6"){
    output=6;}
  else if(myStr=="7"){
    output=7;}
  else if(myStr=="8"){
    output=8;}
  else if(myStr=="9"){
    output=9;}
  else if(myStr=="0"){
    output=0;}
  
  return output;
}
void ISR_button(){
  if ((millis()-lastPress)>debounceTime){
    failure=true;
    lastPress=millis();
    motorLF.sendPWM("forward",0);
    motorRF.sendPWM("forward",0);
    motorRM.sendPWM("forward",0);
    motorLM.sendPWM("forward",0);
    motorRB.sendPWM("forward",0);
    motorLB.sendPWM("forward",0); 
    Serial.end();
  }
}