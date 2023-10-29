//imports
#include "MeMCore.h"

//pin assign
MeLineFollower lineFinder(PORT_2); 
MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
MePort IR(PORT_3); //A2 -> IR , A3 -> LDR
#define ULTRASONIC 12

//variables
#define TURN_TIME 330
#define TIMEOUT 2000
#define FAST  235
#define SLOW  165
int red,blue,green,white;

//Movement Package
void stop(){
  leftMotor.run(0);
  rightMotor.run(0);
}

void move(int left, int right, int time = TURN_TIME){ //supply {left} power to left motor, {right} power for right motor for {time} ms
  leftMotor.run(left);
  rightMotor.run(right);
  delay(time);
  stop();
}

void forward(int time){ //move forward for {time} ms
  move(-FAST,FAST,time);
}

//left turn : red
void left(){
  move(FAST,FAST);
} 
//righ turn : green
void right(){
  move(-FAST,-FAST);
}
//spin in place : orange
void spinaround(){
  move(FAST,FAST,2*TURN_TIME);
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


//Sensor Package

float detect(){
  pinMode(ULTRASONIC,OUTPUT);
  digitalWrite(ULTRASONIC, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC, LOW);
  pinMode(ULTRASONIC,INPUT);
  return pulseIn(ULTRASONIC,HIGH,TIMEOUT) * 0.01725;
}

int read_colour(int A0_VAL,int A1_VAL){
  digitalWrite(A0,A0_VAL);
  digitalWrite(A1,A1_VAL);
  delay(200);
  return analogRead(A3);
}

//Advanced Actions

void moveTillBlack(){
    while (lineFinder.readSensors() == S1_OUT_S2_OUT){
      float dist = detect();
      Serial.print("Distance:");
      Serial.println(dist);
      if (dist>13){ //if distance from right wall > 13cm, adjust right
      leftMotor.run(-FAST);
      rightMotor.run(SLOW);
      }
      else if (0<dist<8){ // if distance from right wall < 8 cm, adjust left
        leftMotor.run(-SLOW);
        rightMotor.run(FAST);
      }
      else{ //if dist == 0 (no wall) or 8<dist<13 keep going straight
        leftMotor.run(-FAST);
        rightMotor.run(FAST);
      }
    }
    
    stop();

    //Nudge if slightly off
    while (lineFinder.readSensors() == S1_OUT_S2_IN){
      move(-SLOW,-SLOW,100);
    }
    while (lineFinder.readSensors() == S1_IN_S2_OUT){
      move(SLOW,SLOW,100);
    }
}

void colourAction(){ 
  red = read_colour(HIGH,HIGH);
  green = read_colour(LOW,HIGH);
  blue = read_colour(HIGH,LOW);
  white = read_colour(LOW,LOW);
  /*
  Serial.print("Red ");
  Serial.print(red);
  Serial.print("Green ");
  Serial.print(green);
  Serial.print("Blue ");
  Serial.print(blue);
  Serial.print("White ");
  Serial.println(white);
  */
  Serial.print("Predicting Colour:");
  if (red > 600 && green > 600 && blue > 600) { // WHITE
   Serial.println("White");
  } 
  else if (red < 350 && green < 350 && blue < 350) { //BLACK
   Serial.println("Black");
  } 
  else if (blue > red - 50 && blue > green) { //BLUE
   Serial.println("Blue");
  } 
  else if (green > red - 50 && green > blue) { //GREEN
   Serial.println("Green");
  } 
  else if (white < 180){ // RED
   Serial.println("Red");
  }
  else { // ORANGE
   Serial.println("Orange or Purple");
  }
}

//MAIN

void setup()
{
  //delay(5000); 
  Serial.begin(9600);
  pinMode(ULTRASONIC,OUTPUT);
  digitalWrite(ULTRASONIC, LOW);
  pinMode(A0,OUTPUT);
  pinMode(A1,OUTPUT);
  digitalWrite(A0,LOW);
  digitalWrite(A1,LOW);
}

//run naive loop with no colour sensing
void loop() {
  moveTillBlack();
  left();
  moveTillBlack();
  left();
  moveTillBlack();
  while(1){}
}

void final_loop() {
  moveTillBlack();
  colourAction();
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

