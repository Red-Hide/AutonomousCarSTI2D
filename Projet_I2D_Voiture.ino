//Libraries
#include <L298N.h>
#include <NewPing.h>
#include <LIS3MDL.h>
#include <Servo_Hardware_PWM.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <string.h>


//Define pins for left motor (ENA)
const unsigned int MOTOR_DIRECTION_IN1 = 11;
const unsigned int MOTOR_DIRECTION_IN2 = 10;

//Define pins for right motor (ENB)  
const unsigned int MOTOR_RIGHT_IN3 = 8;
const unsigned int MOTOR_RIGHT_IN4 = 7;

//Define pins for Ultrasonic Sensor
#define trig_pin A0
#define echo_pin A1

//Define max distance for sensor
#define maximum_distance 200

//Define bluetooth pins
#define bt_rx 3
#define bt_tx 4

//Define Led pin
#define ledpin 12

//Define Sensor
NewPing sonar(trig_pin, echo_pin, maximum_distance);

//Define Servo
Servo myservo;

//Create object for the Magnetometer
LIS3MDL mag;

//Define Bluetooth pins
SoftwareSerial BT(bt_rx, bt_tx);

//Initiate Motors
L298N MOTOR_DIRECTION(MOTOR_DIRECTION_IN1, MOTOR_DIRECTION_IN2); // Backward == Right , Forward == Left 1000ms in opposite direction goes to neutral
L298N MOTOR_RIGHT(MOTOR_RIGHT_IN4, MOTOR_RIGHT_IN3); //Conecteurs inversés

// Motors
void goForward(void);
void goBackward(void);
void goLeftForward(void);
void goLeftBackward(void);
void LeftDirection(void);
void goRightForward(void);
void goRightBackward(void);
void RightDirection(void);
void DirectionStop(void);
void MoveStop(void);
void Stop(void);
void calculateHeadingError(void);
void correctHeading(void);
void GetHeading(void);
void GetDesiredHeading(void);

void BluetoothDebug(void);
void Bluetooth(void);
void Move(void);

void setupPins(void);
void Reset(void);

void LedState(void);

String bt_output;
char BTchar;

int heading;
int heading_threshold = 10;
int heading_error;
int desired_heading;
int distance;
int distance_threshold;
int readPing();
int lookRight();
int lookLeft();

const int intervalBT = 250;

unsigned long PreviousMillisBT = 0;
unsigned long CurrentMillis;

bool debug_mode;
bool autonomous;
bool got_desired_heading;
bool returnPath;
bool lights;

void setup() {
  myservo.attach(12);
  myservo.write(90); //Align the servo
  MOTOR_RIGHT.setSpeed(255); //Set Speed for RIGHT MOTOR
  MOTOR_DIRECTION.setSpeed(255); //Set Speed for the direction Motor
  setupPins(); //Setup all pins
  Serial.begin(9600); //Begin Serial connection
  BT.begin(9600);
  Wire.begin();
  returnPath = false;
  lights = false;
  autonomous = false;
  got_desired_heading = false,
  debug_mode = false;
  bt_output = "";
  distance_threshold = 20;
  Stop();
  LedState();
  mag.enableDefault();
  if (!mag.init())
  {
    BT.println("Failed to detect and initialize magnetometer!");
    Serial.println("Failed to detect and initialize magnetometer! ");
  }
}

void loop() {
  CurrentMillis = millis();
  if(mag.init()){
  GetHeading();
  GetDesiredHeading();
  }
  if(autonomous == true){
    Move();
  }

  if(CurrentMillis - PreviousMillisBT >= intervalBT){
    PreviousMillisBT = CurrentMillis;
    BluetoothDebug();

  }
}

void Move() {
  int distanceRight = 0; //Resets distance
  int distanceLeft = 0;
  int distance = readPing();

    
//Autonomous part of the code


      if (desired_heading != heading){
      calculateHeadingError();
    }
    // Correct the heading if needed
    if (abs(heading_error) <= abs(heading_threshold)){
      goForward();
    }
    else {
      correctHeading();
      goForward();
    }

    if(distance <= 20){
      Stop();
      delay(300);
      goBackward();
      delay(400);
      Stop();
      delay(300);
      distanceRight = lookRight();
      delay(300);
      distanceLeft = lookLeft();
      delay(300);
    /*if(mag.init()){
      if(distance >= distanceLeft){
        while(heading != (desired_heading - 47 <= heading <= desired_heading - 43)){
          goRightForward();
          GetHeading();
          BT.print("Heading : ") ; BT.println(heading);
        }
        Stop();
        delay(100);
        MOTOR_RIGHT.forwardFor(200);
        if(returnPath == true){
        while(heading != (desired_heading + 43 <= heading <= desired_heading + 47)){
          goLeftForward();
          GetHeading();
          BT.print("Heading : ") ; BT.println(heading);
        }
        Stop();
        delay(100);
        MOTOR_RIGHT.forwardFor(200);
        }
      }
      else if(distance >= distanceRight){
         while(heading != (desired_heading + 43 <= heading <= desired_heading + 47)){
          goLeftForward();
          GetHeading();
          BT.print("Heading : ") ; BT.println(heading);
        }
        Stop();
        delay(100);
        MOTOR_RIGHT.forwardFor(200);
        if(returnPath == true){
        while(heading != (desired_heading - 47 <= heading <= desired_heading - 43)){
          goRightForward();
          GetHeading();
          BT.print("Heading : ") ; BT.println(heading);
        }
        Stop();
        delay(100);
        MOTOR_RIGHT.forwardFor(200);
        }
      }
    }*/
      if(distance >= distanceLeft){
              goBackward();
              delay(600);
              goRightForward();
              delay(1500);
              Stop();       
      }
      else if(distance >= distanceRight){
              goBackward();
              delay(600);
              goLeftForward();
              delay(1500);
              Stop();
      }
    }
}

void Bluetooth() {    
  while(BT.available()>0){
    BTchar = BT.read();
    bt_output = bt_output + BTchar;
  }
    if(bt_output.indexOf("F") >= 0 && autonomous == false){
      goForward();
    } 
    if(bt_output.indexOf("B") >= 0 && autonomous == false){
      goBackward();
    }
    if(bt_output.indexOf("L") >=0 && autonomous == false){
      LeftDirection();
    }
    if(bt_output.indexOf("R") >=0 && autonomous == false){
      RightDirection();
    }
    if(bt_output.indexOf("D") >=0){
      debug_mode = !debug_mode;
    }
    if(bt_output.indexOf("H") >=0){
      got_desired_heading = !got_desired_heading;
    }
    if(bt_output.indexOf("A") >=0){
      autonomous = !autonomous;
      }
    if(bt_output.indexOf("I") >=0){
      lights = !lights;
      LedState();
      }
    if(bt_output.indexOf("P") >=0){
      returnPath = !returnPath;
      }
    if(bt_output.indexOf("S") >= 0){
      DirectionStop();
    }
    if(bt_output.indexOf("M") >= 0 ){
      MoveStop();
    }
    if(bt_output.indexOf("O") >= 0 ){
      Reset();
      LedState();
    }
    Serial.println(BTchar);
    Serial.println(bt_output);
}

void BluetoothDebug(){
      if(debug_mode == true){
    //BT.print("Heading : ") ; BT.println(heading);
    //BT.print("Heading error : "); BT.println(heading_error);
    //BT.print("BT Output : "); BT.println(bt_output);
    BT.print("Distance sensor : "); BT.println(readPing());
    //BT.print("Desired heading : "); BT.println(desired_heading);
    Serial.print("Heading : ") ; Serial.println(heading);/*
    Serial.print("Heading error : "); Serial.println(heading_error);
    Serial.print("BT Output : "); Serial.println(bt_output);
    Serial.print("Distance sensor : "); Serial.println(readPing());*/
  }
  bt_output = "";
}

void GetDesiredHeading(){
    if (got_desired_heading == false){
    desired_heading = heading;
    got_desired_heading = !got_desired_heading;
  }
}

void GetHeading(){
    //Magnetometer Code
    mag.read();

    heading = atan2(mag.m.y, mag.m.x) * 180 / M_PI;
}


void calculateHeadingError() {
 
  // Calculate the heading error
  heading_error = heading - desired_heading;
  if (heading_error > 180) {
      heading_error -= 360;
  }
  if (heading_error < -180) {
      heading_error += 360;
  }
}

void correctHeading(){  
  // Turn the vehicle until it is facing the correct
  // direction
  if (heading_error < -heading_threshold) {
    while (heading_error < -heading_threshold) {
      goRightForward();
      calculateHeadingError();
    }
  }
  else {
    while (heading_error > heading_threshold) {
      goLeftForward();
      calculateHeadingError();
    }
  }
}

//Code pour le capteur à Ultrasons/Servo

int lookRight(){  
  myservo.write(50);
  delay(500);
  int distance = readPing();
  delay(100);
  myservo.write(115);
  Serial.println("looking right");
  return distance;
}

int lookLeft(){
  myservo.write(170);
  delay(500);
  int distance = readPing();
  delay(100);
  myservo.write(115);
  Serial.println("looking left");
  return distance;
  delay(100);
}

int readPing(){
  int cm = sonar.ping_cm();
  if (cm==0){
    cm=250;
  }
  return cm;
}

void LedState(){
  if(lights == true){
  digitalWrite(ledpin, HIGH);
  }
  else if (lights == false){
    digitalWrite(ledpin, LOW);
  }
  Serial.println(lights);
}


//Code pour contrôler les moteurs
// Backward == Right , Forward == Left 1000ms in opposite direction goes to neutral
 
void goForward(){
  MOTOR_RIGHT.forward();
  Serial.print("Forward");
}

void goBackward(){
  MOTOR_RIGHT.backward();
  Serial.println("Backwards");
}
 
void goLeftForward(){
  MOTOR_RIGHT.forward();
  MOTOR_DIRECTION.forward();
}

void goLeftBackward(){
  MOTOR_RIGHT.backward();
  MOTOR_DIRECTION.forward();
}

void goRightForward(){
  MOTOR_RIGHT.forward();
  MOTOR_DIRECTION.backward();
}

void goRightBackward(){
  MOTOR_RIGHT.backward();
  MOTOR_DIRECTION.backward();
}

void LeftDirection(){
  MOTOR_DIRECTION.forward();
  Serial.println("going left");
}

void RightDirection(){
  MOTOR_DIRECTION.backward();
  Serial.println("going right");
}

void DirectionStop(){
  MOTOR_DIRECTION.stop();
  Serial.println("Direction stop");
}

void MoveStop(){
  MOTOR_RIGHT.stop();
  Serial.println("Move Stop");
}

void Stop(){
  MOTOR_RIGHT.stop();
  MOTOR_DIRECTION.stop();
  Serial.println("Stopped");
}

void Reset(){
  lights = false;
  got_desired_heading = false;
  autonomous = false;
  debug_mode = false;
  returnPath = false;
  Stop();
}

void setupPins(){

  //Configure HC-SR04 pins
   pinMode(trig_pin, OUTPUT);
   pinMode(echo_pin, INPUT);
  pinMode(bt_rx, INPUT);
  attachInterrupt(digitalPinToInterrupt(bt_rx),Bluetooth, RISING); // Thanks to JediRick in the Arduino Discord Server for heleping me figure out that CHANGE isn't working
  pinMode(ledpin, OUTPUT);
}
