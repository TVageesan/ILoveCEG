//imports
#include "MeMCore.h"
 
//pin assign
MeLineFollower lineFinder(PORT_2); 
MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
MeBuzzer buzzer;
#define ULTRASONIC 12
#define IR_PIN A2
#define LDR_PIN A3

//constants
#define TURN_TIME 420
#define FWD_TIME 600
#define TIMEOUT 1550
#define FAST  245
#define TURN  160
#define SLOW  200


//variables
//frequency and duration of buzzer tones: imperial march song
int frequencies[] = {
    440, 440, 440, 349, 523, 440, 349, 523, 440, 659, 659, 659, 698, 523,
    415, 349, 523, 440, 880, 440, 440, 880, 831, 784, 740, 698, 740, 466,
    622, 587, 554, 523, 494, 523, 349, 415, 349, 415, 523, 440, 523, 659,
    880, 440, 440, 880, 831, 784, 740, 698, 740, 466, 622, 587, 554, 523,
    494, 523, 349, 415, 349, 523, 440, 349
    };

int durations[] = {
    500, 500, 500, 376, 126, 500, 376, 126, 1000, 500, 500, 500, 376, 126,
    500, 376, 126, 1000, 500, 376, 126, 500, 376, 126, 166, 166, 166, 250,
    500, 376, 126, 166, 166, 166, 250, 500, 376, 126, 500, 376, 126, 1000,
    500, 376, 126, 166, 166, 166, 250, 500, 376, 126, 166, 166, 166, 250,
    500, 376, 126, 500, 376, 126, 500, 500
    };
float red,green,blue; // for color detector
int states[4][2] = {{HIGH,HIGH},{HIGH,LOW},{LOW,HIGH},{LOW,LOW}}; // Flash LED red,green,blue,off
float colourArray[3] = {0,0,0}; //RGB values
float whiteArray[3] = {848,835,841}; // LDR baseline voltages: white
float blackArray[3] = {554,610,601}; // LDR baseline voltages: black
float baseline,detected; // for IR readings
int dist,correct,IR_val; // for movement
//prototype functions used later in movement
float US_detect();
int IR_detect();

void song(){ // to play song at end() condition
  for (int i = 0; i < 60;i++){
    buzzer.tone(frequencies[i], durations[i]);
  }
}

//MOVEMENT PACKAGE
//basic movement
void stop(){
  leftMotor.run(0);
  rightMotor.run(0);
}

void power(int left, int right, int time){ //supply {left} power to left motor, {right} power for right motor for {time} ms
  leftMotor.run(left);
  rightMotor.run(right);
  delay(time);
  stop();
 
}

void forward(int time){ //move forward for {time} ms at {FAST} power
  power(-FAST,FAST,time);
  delay(500); // added delay after stop for stability of double turns
}

//colour actions

//left turn : red
void left(){
  power(TURN,TURN,TURN_TIME); // turn left for {TURN_TIME} ms
} 
//righ turn : green
void right(){
  power(-TURN,-TURN,TURN_TIME); // turn right for {TURN_TIME} ms
}
//spin in place : orange
void spinaround(){
  power(-TURN,-TURN,2*TURN_TIME); //right() for twice of {TURN_TIME}
}
//double left : purple
void doubleleft(){ 
  left();
  forward(FWD_TIME); //forward for FWD_TIME
  left();
}
//double right : lightblue
void doubleright(){
  right();
  forward(FWD_TIME); //forward for FWD_TIME
  right();
}
//end point : white
void end(){
  song();
  while(1){}; //END: enter infinite loop
}

//nude rotates for gentle adjustment
void nudgeLeft(){ 
  power(-TURN,-TURN,100);
}

void nudgeRight(){ 
  power(TURN,TURN,100);
}

//moveCalibrated helpers
void adjustLeft(){
  leftMotor.run(-SLOW);
  rightMotor.run(FAST);
}
void adjustRight(){
  leftMotor.run(-FAST);
  rightMotor.run(SLOW);
}

void straight(){
  leftMotor.run(-FAST);
  rightMotor.run(FAST);
}

//moves forward while using Ultrasound and IR to adjust left/right
void moveCalibrated(){
  dist = US_detect();
  IR_val = IR_detect();
  if (dist == 0){ //WALL MISSING -> USE IR
    if (IR_val <= 240){//WALL MISSING BOTH SIDES -> STRAIGHT
      straight();
    }
    else if (IR_val <= 260){ // 11-9cm from wall -> GO RIGHT
      adjustRight();
    }
    else if (IR_val <= 280){ //6-9cm  from wall-> STAY CENTER
      straight();
    }
    else { // 0-6cm  from wall-> GO LEFT
       adjustLeft();
    }
  }else{ //ULTRASOUND: PROPORTIONAL MOTOR CORRECTION
    correct = 20 * ( dist - 10 );
    leftMotor.run(correct - FAST);
    rightMotor.run(FAST + correct);
  }
}
//moves until it detects black, then rotates left/right to align onto line
void moveTillBlack(){ //relys mainly on 
    while (lineFinder.readSensors() == S1_OUT_S2_OUT){
      moveCalibrated(); //move using ultrasonic, IR to calibrate
    }
    
    stop();
    
    //Nudge if slightly off 
    while (lineFinder.readSensors() == S1_OUT_S2_IN){
      nudgeLeft();
    }
    while (lineFinder.readSensors() == S1_IN_S2_OUT){
      nudgeRight();
    }
}

//Sensor Package
float US_detect(){ //returns cm distance from ultrasonic sensor to wall
  pinMode(ULTRASONIC,OUTPUT); // ULTRASONIC PIN: PIN 12
  digitalWrite(ULTRASONIC, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC, LOW);
  pinMode(ULTRASONIC,INPUT);
  return pulseIn(ULTRASONIC,HIGH,TIMEOUT) * 0.01715; // time * 0.01715 -> simplified version of (time/2/10^6*343*100)
}

int IR_detect(){ //returns raw IR voltage value
  // starting state A0 LOW, A1 LOW = IR ON
  detected = analogRead(IR_PIN);
  delay(5);
  digitalWrite(A1,HIGH); // A0 LOW, A1 HIGH = IR OFF
  delay(5);
  baseline = analogRead(IR_PIN);
  digitalWrite(A1,LOW); // A0 LOW, A1 LOW = IR ON
  return (baseline - detected);
}

float read_colour(int A0_VAL,int A1_VAL){ //returns LDR voltage after flashing LED corresponding to {A0_VAL,A1_VAL}
  //flash LED colours according to truth table
  digitalWrite(A0,A0_VAL);
  digitalWrite(A1,A1_VAL);
  delay(200); //dely to allow voltage to stabilize
  return analogRead(LDR_PIN); //reads LDR voltage
}

void setBalance(){ //calibrates raw LDR values on white and black paper to use as greyDiff in RGB calculation
  Serial.println("Put White Sample For Calibration ...");
  for(int i = 0;i<=2;i++){
    whiteArray[i] = read_colour(states[i][0],states[i][1]);
    delay(200);
  }
  Serial.println("Put Black Sample For Calibration ...");
  delay(5000); //delay for five seconds for moving bot to the black sample
  for(int i = 0;i<=2;i++){
    blackArray[i] = read_colour(states[i][0],states[i][1]);
    delay(200);
  }
  Serial.println("Colour Sensor Is Ready.");
}

void colourDetect(){ //flashes RGB LEDs, reads corresponding LDR voltage, converts into RGB and stores in colourArray
  for (int c = 0; c <= 2;c++){
    colourArray[c] = ((read_colour(states[c][0],states[c][1]) - blackArray[c])/(whiteArray[c] - blackArray[c])) *  255;
    delay(200);
  }; 
  //DEBUG: prints values in colourArray
  Serial.print("red:");
  Serial.println(colourArray[0]);
  Serial.print("blue: ");
  Serial.println(colourArray[1]);
  Serial.print("green: ");
  Serial.println(colourArray[2]);
}

void colourAction(){ // reads colour and performs corresponding movement 
  delay(500); //small delay to allow colour reading to stabilize
  colourDetect(); // updates colourArray with current RGB readings
  red = colourArray[0];
  green = colourArray[1];
  blue = colourArray[2];
  if ((red >= 130) && (blue >= 200) && (green >= 200)){//white
    end();
  }
  else if (blue >= 200){//blue
    doubleright();
  }
  else if ((green >= 120) && (red <= 80)){ //green
    right();
  }
  else if ((red >= 100) && (blue <=100) ){ 
    if ( green >= 110 ){//orange
      spinaround();
    }else{//red
      left();
    }
  }
  else { //purple
    doubleleft();
  }
}
//MAIN

void setup()
{
  Serial.begin(9600);
  pinMode(ULTRASONIC,OUTPUT);

  pinMode(A0,OUTPUT);
  pinMode(A1,OUTPUT);
  pinMode(IR_PIN,INPUT);
  pinMode(LDR_PIN,INPUT);
  
  //default writes: LED OFF, ULTRASONIC OFF 
  digitalWrite(ULTRASONIC, LOW);
  digitalWrite(A0,LOW);
  digitalWrite(A1,LOW);
  //setBalance();
}   

void loop(){
  moveTillBlack();
  colourAction(); 
}


 
