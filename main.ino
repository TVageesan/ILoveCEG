//imports
#include "MeMCore.h"

//pin assign
MeLineFollower lineFinder(PORT_2); 
MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
#define ULTRASONIC 12

//variables
#define TURN_TIME 330
#define TIMEOUT 1550
#define FAST  200
#define SLOW  160

float red,green,blue;
int conv[4][2] = {{HIGH,HIGH},{LOW,HIGH},{HIGH,LOW},{LOW,LOW}}; // Flash LED red, blue,green, off
float colourArray[3] = {0,0,0};
float whiteArray[3] = {872,789,801};
float blackArray[3] = {527,474,458};
float detect();
int IR();

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

void moveTillBlack(){ //relys mainly on 
    while (lineFinder.readSensors() == S1_OUT_S2_OUT){
      float dist = detect();
      int IR_VAL = IR();
      if (dist == 0){ //WALL MISSING -> USE IR
        if (IR_VAL <= 240){//WALL MISSING BOTH SIDES -> STRAIGHT
          straight();
        }
        else if (IR_VAL <= 260){ // 11-9cm -> GO RIGHT
          adjustRight();
        }
        else if (IR_VAL <= 280){ //6-9cm -> STAY CENTER
          straight();
        }
        else { // 0-6cm -> GO LEFT
          adjustLeft();
        }
      }
      else if (dist>13){ //if distance from right wall > 13cm, adjust right
      leftMotor.run(-SLOW);
      rightMotor.run(FAST);
      }
      else if (dist<8){ // if distance from right wall < 8 cm, adjust left
        leftMotor.run(-FAST);
        rightMotor.run(SLOW);
      }
      else{ //8<dist<13 keep going straight
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
  float cms =  pulseIn(ULTRASONIC,HIGH,TIMEOUT) * 0.01725;
  Serial.println(cms);
  return cms;
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
  int val = (high - low);
  Serial.println(val);
  return val;
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

//TODO: REPLACE SERIAL PRINTS WITH LED FLASHES
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
    end();
  }
  else if (blue >= 200){
    Serial.println("blue"); //DOUBLE RIGHT
    doubleright();
  }
  else if ((green >= 110) && (red <= 80)){ 
    Serial.println("green"); //RIGHT
    right();
  }
  else if (red >= 160){ 
    if ( green >= 100 ){
      Serial.println("orange");//180 TURN
      spinaround();
    }else{
      Serial.println("red");//LEFT
      left();
    }
  }
  else {
    Serial.println("Burple");//DOUBLE LEFT
    doubleleft();
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
  //setBalance();
}   

void loop(){
  moveTillBlack();
  delay(2000);
  colourAction();
  
}

void loop_colour(){//testing colour calibration
  moveTillBlack();
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

void setup_buzzer_imperial_march(){
    buzzer.tone(440, 500);
    buzzer.tone(440, 500);
    buzzer.tone(440, 500);
    buzzer.tone(349, 376);
    buzzer.tone(523, 126);
    
    buzzer.tone(440, 500);
    buzzer.tone(349, 376);
    buzzer.tone(523, 126);
    buzzer.tone(440, 1000);
    
    buzzer.tone(659, 500);
    buzzer.tone(659, 500);
    buzzer.tone(659, 500);
    buzzer.tone(698, 376);
    buzzer.tone(523, 126);
    
    buzzer.tone(415, 500);
    buzzer.tone(349, 376);
    buzzer.tone(523, 126);
    buzzer.tone(440, 1000);
    
    buzzer.tone(880, 500);
    buzzer.tone(440, 376);
    buzzer.tone(440, 126);
    buzzer.tone(880, 500);
    buzzer.tone(831, 376);
    buzzer.tone(784, 126);
    
    buzzer.tone(740, 166);
    buzzer.tone(698, 166);
    buzzer.tone(740, 166);
    delay(250);
    buzzer.tone(466, 250);
    buzzer.tone(622, 500);
    buzzer.tone(587, 376);
    buzzer.tone(554, 126);
    
    buzzer.tone(523, 166);
    buzzer.tone(494, 166);
    buzzer.tone(523, 166);
    delay(250);
    buzzer.tone(349, 250);
    buzzer.tone(415, 500);
    buzzer.tone(349, 376);
    buzzer.tone(415, 126);
    
    buzzer.tone(523, 500);
    buzzer.tone(440, 376);
    buzzer.tone(523, 126);
    buzzer.tone(659, 1000);
    
    buzzer.tone(880, 500);
    buzzer.tone(440, 376);
    buzzer.tone(440, 126);
    buzzer.tone(880, 500);
    buzzer.tone(831, 376);
    buzzer.tone(784, 126);
    
    buzzer.tone(740, 166);
    buzzer.tone(698, 166);
    buzzer.tone(740, 166);
    delay(250);
    buzzer.tone(466, 250);
    buzzer.tone(622, 500);
    buzzer.tone(587, 376);
    buzzer.tone(554, 126);

    buzzer.tone(523, 166);
    buzzer.tone(494, 166);
    buzzer.tone(523, 166);
    delay(250);
    buzzer.tone(349, 250);
    buzzer.tone(415, 500);
    buzzer.tone(349, 376);
    buzzer.tone(523, 126);
    
    buzzer.tone(440, 500);
    buzzer.tone(349, 376);
    buzzer.tone(523, 126);
    buzzer.tone(440, 1000);
    
}

void song_dududu(int buzzerPin){
  
  tone(buzzerPin, 220);
  delay(23);
  noTone(buzzerPin);

  tone(buzzerPin, 440);
  delay(47);
  noTone(buzzerPin);

  tone(buzzerPin, 554);
  delay(291);
  noTone(buzzerPin);

  tone(buzzerPin, 659);
  delay(47);
  noTone(buzzerPin);

  tone(buzzerPin, 1175);
  delay(58);
  noTone(buzzerPin);

  tone(buzzerPin, 1661);
  delay(93);
  noTone(buzzerPin);

  tone(buzzerPin, 784);
  delay(140);
  noTone(buzzerPin);

  tone(buzzerPin, 1109);
  delay(721);
  noTone(buzzerPin);

  tone(buzzerPin, 1319);
  delay(733);
  noTone(buzzerPin);

  tone(buzzerPin, 1760);
  delay(70);
  noTone(buzzerPin);

  tone(buzzerPin, 2093);
  delay(337);
  noTone(buzzerPin);

  tone(buzzerPin, 988);
  delay(35);
  noTone(buzzerPin);

  tone(buzzerPin, 220);
  delay(35);
  noTone(buzzerPin);

  tone(buzzerPin, 349);
  delay(35);
  noTone(buzzerPin);

  tone(buzzerPin, 1480);
  delay(47);
  noTone(buzzerPin);

  tone(buzzerPin, 1865);
  delay(116);
  noTone(buzzerPin);

  tone(buzzerPin, 622);
  delay(35);
  noTone(buzzerPin);

  tone(buzzerPin, 1245);
  delay(81);
  noTone(buzzerPin);

  tone(buzzerPin, 1397);
  delay(35);
  noTone(buzzerPin);

  tone(buzzerPin, 208);
  delay(47);
  noTone(buzzerPin);

  tone(buzzerPin, 2349);
  delay(163);
  noTone(buzzerPin);

  tone(buzzerPin, 330);
  delay(93);
  noTone(buzzerPin);

  tone(buzzerPin, 988);
  delay(407);
  noTone(buzzerPin);

  tone(buzzerPin, 1661);
  delay(570);
  noTone(buzzerPin);

  tone(buzzerPin, 220);
  delay(23);
  noTone(buzzerPin);

  tone(buzzerPin, 1397);
  delay(163);
  noTone(buzzerPin);

  tone(buzzerPin, 440);
  delay(279);
  noTone(buzzerPin);

  tone(buzzerPin, 1245);
  delay(35);
  noTone(buzzerPin);

  tone(buzzerPin, 220);
  delay(58);
  noTone(buzzerPin);

  tone(buzzerPin, 784);
  delay(267);
  noTone(buzzerPin);

  tone(buzzerPin, 1175);
  delay(70);
  noTone(buzzerPin);

  tone(buzzerPin, 1865);
  delay(605);
  noTone(buzzerPin);

  tone(buzzerPin, 208);
  delay(23);
  noTone(buzzerPin);

  tone(buzzerPin, 659);
  delay(430);
  noTone(buzzerPin);

  tone(buzzerPin, 220);
  delay(35);
  noTone(buzzerPin);

  tone(buzzerPin, 330);
  delay(81);
  noTone(buzzerPin);

  tone(buzzerPin, 1480);
  delay(116);
  noTone(buzzerPin);

  tone(buzzerPin, 3136);
  delay(35);
  noTone(buzzerPin);

  tone(buzzerPin, 233);
  delay(70);
  noTone(buzzerPin);

  tone(buzzerPin, 1245);
  delay(70);
  noTone(buzzerPin);
}
