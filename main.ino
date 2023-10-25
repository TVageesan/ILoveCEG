//imports
#include "MeMCore.h"

//pin assign
MeLineFollower lineFinder(PORT_2); 
MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
MePort IR(PORT_3); //A2 -> IR , A3 -> LDR
#define CONTROL1 A0
#define CONTROL2 A1
#define IR A2
#define ULTRASONIC 12
int red_v,blue_v,green_v,white_v;
//variables
#define TURN_TIME 330
#define TIMEOUT 200000
#define FAST  235
#define SLOW  165



//Sensor Package
float detect(){
  pinMode(ULTRASONIC,OUTPUT);
  digitalWrite(ULTRASONIC, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC, LOW);
  pinMode(ULTRASONIC,INPUT);
  long cms = pulseIn(ULTRASONIC,HIGH,TIMEOUT)* 0.01725;
  Serial.print("Distance from wall: ");
  Serial.println(cms);
  return cms;
}

int linefollow(){
  return lineFinder.readSensors();
}

//Movement Package
void stop(){
  leftMotor.run(0);
  rightMotor.run(0);
}

void forward(int time){
  leftMotor.run(-FAST);
  rightMotor.run(FAST);
  delay(time); 
  stop();
}

void turn(int left, int right, int time = TURN_TIME){
  leftMotor.run(left);
  rightMotor.run(right);
  delay(time);
  stop();
}

void moveTillBlack(){
    while (lineFinder.readSensors() == S1_OUT_S2_OUT){
      if (detect()>13){
      leftMotor.run(-FAST);
      rightMotor.run(SLOW);
      }
      else if (detect() < 8){
        leftMotor.run(-SLOW);
        rightMotor.run(FAST);
      }
      else{
        leftMotor.run(-FAST);
        rightMotor.run(FAST);
      }
    }
    
    stop();
    while (lineFinder.readSensors() == S1_OUT_S2_IN){
      turn(-SLOW,-SLOW,100);
    }
    while (lineFinder.readSensors() == S1_IN_S2_OUT){
      turn(SLOW,SLOW,100);
    }
}


//left turn : red
void left(){
  turn(FAST,FAST);
} 
//righ turn : green
void right(){
  turn(-FAST,-FAST);
}
//spin in place : orange
void spinaround(){
  turn(FAST,FAST,2*TURN_TIME);
}
//double left : purple
void doubleleft(){
  left();
  forward(1000);
  left();
}
//double right :lightblue
void doubleright(){
  right();
  forward(1000);
  right();
}
//end point : white
void end(){
  //celebrate();
}

//fourth test: IR reading and ultrasonic readings
void loop_4(){
  detect();
}

//third test: calibrating turns
void loop_3(){
  turn(SLOW,SLOW,500);
  stop();
  while(1){}
}

//second test: move up to black strip and turn left
void loop_2(){
  moveTillBlack();
  left();
  while(1){}
}



void blue(){
  digitalWrite(A0,HIGH);
  digitalWrite(A1,LOW);
}

void green(){
  digitalWrite(A0,LOW);
  digitalWrite(A1,HIGH);
}

void red(){
  digitalWrite(A0,HIGH);
  digitalWrite(A1,HIGH);
}

void white(){
  digitalWrite(A0,LOW);
  digitalWrite(A1,LOW);
}




void led_cycle(){ 
  red();
  delay(200);
  red_v = analogRead(A3);
  green();
  delay(200);
  green_v = analogRead(A3);
  blue();
  delay(200);
  blue_v = analogRead(A3);
  white();
  delay(200);
  white_v = analogRead(A3);
  Serial.print("Red ");
  Serial.print(red_v);
  Serial.print("Green ");
  Serial.print(green_v);
  Serial.print("Blue ");
  Serial.print(blue_v);
  Serial.print("White ");
  Serial.println(white_v);
}

void setup()
{
  //delay(5000); 
  Serial.begin(9600);
  pinMode(ULTRASONIC,OUTPUT);
  digitalWrite(ULTRASONIC, LOW);
  pinMode(A0,OUTPUT);
  pinMode(A1,OUTPUT);
  white();
}

void loop_cycle(){
  led_cycle();
}
void loop_sensor(){
  digitalWrite(A0,LOW);
  digitalWrite(A1,LOW);
  Serial.print("IR value: ");
  Serial.println(analogRead(A2));
  //Serial.print("LDR value: ");
  //Serial.println(analogRead(A3));
  delay(1000);
}

//retest: see if its a battery issue
void loop() {
  moveTillBlack();
  left();
  moveTillBlack();
  left();
  moveTillBlack();
  while(1){}
}



