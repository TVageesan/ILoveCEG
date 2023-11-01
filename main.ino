//imports
#include "MeMCore.h"

//pin assign
MeLineFollower lineFinder(PORT_2); 
MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
#define ULTRASONIC 12

//variables
#define TURN_TIME 200
#define TIMEOUT 3000
#define FAST  200
#define SLOW  160

int conv[4][2] = {{HIGH,HIGH},{LOW,HIGH},{HIGH,LOW},{LOW,LOW}}; // Flash LED red, green, blue, off
float colourArray[3] = {0,0,0};
float whiteArray[3] = {820,891,922};
float blackArray[3] = {372,518,661};

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
  move(SLOW,SLOW);
} 
//righ turn : green
void right(){
  move(-SLOW,-SLOW);
}
//spin in place : orange
void spinaround(){
  move(SLOW,SLOW,2*TURN_TIME);
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

void moveTillBlack(){
    while (lineFinder.readSensors() == S1_OUT_S2_OUT){
      float dist = detect();
      Serial.print("Distance:");
      Serial.println(dist);
      if (dist>13){ //if distance from right wall > 13cm, adjust right
      leftMotor.run(-SLOW);
      rightMotor.run(FAST);
      }
      else if (dist<8){ // if distance from right wall < 8 cm, adjust left
        leftMotor.run(-FAST);
        rightMotor.run(SLOW);
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


//Sensor Package
float detect(){
  pinMode(ULTRASONIC,OUTPUT);
  digitalWrite(ULTRASONIC, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC, LOW);
  pinMode(ULTRASONIC,INPUT);
  return pulseIn(ULTRASONIC,HIGH,TIMEOUT) * 0.01725;
}

int IR(){ 
  // starting state A0 LOW, A1 LOW , IR ON
  int high,low;
  low = analogRead(A2);
  delay(5);
  digitalWrite(A1,HIGH); // IR OFF
  delay(5);
  high = analogRead(A2);
  digitalWrite(A1,LOW); // IR ON, RESET
  return high - low;
}

float read_colour(int A0_VAL,int A1_VAL){
  float reading = 0;
  digitalWrite(A0,A0_VAL);
  digitalWrite(A1,A1_VAL);
  delay(200);
  reading = analogRead(A3);
  return reading;
  /*
  //get average of 5 readings
  for (int i = 0; i < 5; i++){
    reading += analogRead(A3);
    delay(10);
  }
  return (reading/5);
  */
}

void setBalance(){
  Serial.println("Put White Sample For Calibration ...");
  //delay for five seconds for getting sample ready
  for(int i = 0;i<=2;i++){
    whiteArray[i] = read_colour(conv[i][0],conv[i][1]);
    delay(200);
  }
  Serial.print(whiteArray[0] );
  Serial.print(" ");
  Serial.print(whiteArray[1] );
  Serial.print(" ");
  Serial.println(whiteArray[2] );
  Serial.println("Put Black Sample For Calibration ...");
  delay(5000); //delay for five seconds for getting sample ready and blue to the black array
  for(int i = 0;i<=2;i++){
    blackArray[i] = read_colour(conv[i][0],conv[i][1]);
    delay(200);
  }
  Serial.print(blackArray[0]);
  Serial.print(" ");
  Serial.print(blackArray[1]);
  Serial.print(" ");
  Serial.println(blackArray[2]);
  //delay another 5 seconds for getting ready colour objects
  Serial.println("Colour Sensor Is Ready.");
}
float red,green,blue;

void colourAction(){
  for (int c = 0; c <= 2;c++){
    colourArray[c] = ((read_colour(conv[c][0],conv[c][1]) - blackArray[c])/(whiteArray[c] - blackArray[c])) *  255;
    delay(200);
  } 
  red = colourArray[0];
  blue = colourArray[1];
  green = colourArray[2];
  Serial.print("red:");
  Serial.println(red);
  Serial.print("green: ");
  Serial.println(green);
  Serial.print("blue: ");
  Serial.println(blue);
  if ((red >= 200) && (blue >= 200) && (green >= 200)){
    Serial.println("white"); //END
  }
  else if (blue >= 200){
    Serial.println("blue"); //DOUBLE RIGHT
  }
  else if ((green >= 110) && (red <= 80)){ 
    Serial.println("green"); //RIGHT
  }
  else if (red >= 190){ 
    if ( green >= 130 ){
      Serial.println("orange");//180 TURN
    }else{
      Serial.println("red");//LEFT
    }
  }
  else {
    Serial.println("Burple");//DOUBLE LEFT
  }
}

//MAIN

void setup()
{
  Serial.begin(9600);
  pinMode(ULTRASONIC,OUTPUT);
  digitalWrite(ULTRASONIC, LOW);
  pinMode(A0,OUTPUT);
  pinMode(A1,OUTPUT);
  digitalWrite(A0,LOW);
  digitalWrite(A1,LOW);
  ///setBalance();
}

void loop(){
  Serial.println(IR());
  delay(1000);
}

void loop_colour(){//testing colour calibration
  //Serial.println(IR());
  colourAction();
  delay(1000);
}

void final_loop() {
  moveTillBlack();
  delay(1000);
  colourAction();
}

void loopwwwwww(){
  digitalWrite(A1,HIGH);
  Serial.print("IR value: ");
  delay(200);
  Serial.println(analogRead(A2));
  digitalWrite(A0,LOW);
  //Serial.print("LDR value: ");
  //Serial.println(analogRead(A3));
  delay(1000);
}

//run naive loop with no colour sensing
void loopRRR() {
  moveTillBlack();
  left();
  moveTillBlack();
  left();
  moveTillBlack();
  doubleright();
  moveTillBlack();
  spinaround(); 
  moveTillBlack();
  doubleleft();
  moveTillBlack();
  right();
  moveTillBlack();
  right();
  moveTillBlack();
  while(1){}
}
