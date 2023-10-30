//ReadMe;
// For Movement calibration:
// Try a longer ultrasound timeout to calibrate distance
// Test specifically cms value when ultrasound timeout is called -> maybe it doesnt compare to 0, leads to left turn instead of straight
// Test turning at diff speeds, establish max power turning time and forward times
// For colour calibration (!!!!READ ME SARAH!!!!)
// Variables you can tweak to make colour calibration work:
// RANGE: Set integer range for comparing colours
// ex. if expected number is 200, with range 10 we accept any number from 190 - 210
// can try expanding this range as far as possible before colours begin to mix, i.e. orange gets detected as red.
// COLOURS: Array of array of expected RGB values for a given colour
// use the names array to keep track of which rgb list is which. ex. "red" in index 0 of names arr is linked to {182,90,86} in index 0 of colours arr
// can rewrite the rgb values if you get a new RGB baseline during testing, though ideally our baseline shouldn't change much
// can also always verify with https://www.rapidtables.com/web/color/RGB_Color.html which rgb values is which colour
//ALSO: look for loop_IR below if you want to test IR code 
//imports
#include "MeMCore.h"

//pin assign
MeLineFollower lineFinder(PORT_2); 
MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
#define ULTRASONIC 12

//variables
#define TURN_TIME 400
#define TIMEOUT 5000
#define FAST  200
#define SLOW  160
#define RANGE 10

int conv[4][2] = {{HIGH,HIGH},{HIGH,LOW},{LOW,HIGH},{LOW,LOW}}; // Flash LED red, green, blue, off
float colourArray[3] = {0,0,0};
float whiteArray[3] = {0,0,0};
float blackArray[3] = {0,0,0};

const char *names[6] = {
  "red",
  "green",
  "orange",
  "purple",
  "blue",
  "white"
};

float colours[6][3] = {
  {182,90,86},
  {63,170,112},
  {221,165,126},
  {145,148,198},
  {97,174,226},
  {255,255,255}
};

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
  forward(500);
  left();
}
//double right :lightblue
void doubleright(){
  right();
  forward(500);
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
      leftMotor.run(-FAST);
      rightMotor.run(SLOW);
      }
      else if (dist<8){ // if distance from right wall < 8 cm, adjust left
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
  high = analogRead(A2);
  digitalWrite(A1,HIGH); // IR OFF
  delay(200);
  low = analogRead(A2);
  digitalWrite(A0,LOW); // IR ON, RESET
  return high - low;
}

int compareArr(){ //compares global arr colourArray and colours
  for (int i = 0; i < 6 ;i++){
    int match = 0;
    for (int c = 0; c < 3;c++){
      if ((colours[i][c] - RANGE) <= colourArray[c] <= (colours[i][c] + RANGE)){
        match += 1;
      };
    };
    if (match == 3) return i;
  };
  return -1;
};

float read_colour(int A0_VAL,int A1_VAL){
  float reading = 0;
  digitalWrite(A0,A0_VAL);
  digitalWrite(A1,A1_VAL);
  delay(200);
  //get average of 5 readings
  for (int i = 0; i < 5; i++){
    reading += analogRead(A3);
    delay(10);
  }
  return (reading/5);
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

void colourAction(){
  for (int c = 0; c <= 2;c++){
    colourArray[c] = ((read_colour(conv[c][0],conv[c][1]) - blackArray[c])/(whiteArray[c] - blackArray[c])) *  255;
    delay(200);
  } 
  Serial.println("Received colour arr");
  Serial.print("Red: ");
  Serial.print(colourArray[0]);
  Serial.print("Green: ");
  Serial.print(colourArray[1]);
  Serial.print("Blue: ");
  Serial.println(colourArray[2]);
  int result = compareArr();
  if (result == -1){
    Serial.println("ggs can't determine colour");
    return;
  }
  else{
    Serial.print("result ");
    Serial.println(result);
    Serial.print("colour ");
    Serial.println(names[result]);
  }
  /*
  switch (result) {
    case 0: //red
      left();
      break;
    case 1: //green
      right();
      break;
    case 2: //orange
      spinaround();
      break;
    case 3: //purple
      doubleleft();
      break;
    case 4: //blue
      doubleright();
      break;
    case 5: //white
      end();
      break;
  }

  */
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
  setBalance();
}

void loop(){//testing colour calibration
  colourAction();
  delay(1000);
}

void loop_IR(){ //copy this code into loop() to test IR
  Serial.println(IR());
  delay(1000);
}
void final_loop() {
  moveTillBlack();
  delay(1000);
  colourAction();
}

void loop_www(){
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
void loop_MOVE() {
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


