#include <Arduino.h>
#include <Romi32U4.h>
#include "Timer.h"
#include <servo32u4.h>
#include <BlueMotor.h>
#include <wpi-32u4-lib.h>
#include <IRdecoder.h>
#include <ir_codes.h>
#include <Chassis.h>
#include <Rangefinder.h>


BlueMotor motor;

Chassis chassis;
//Rangefiner setup
Rangefinder rangefinder(2, 12); 
const int trigPin = 12;
const int echoPin = 2; 
double duration = 0;
double distance = 100;
double oldDistance = 10;
double two = 2.0;
double ultrasonicConstant = 10.0;
int rangeDelay = 100;
Timer rangeTimer(rangeDelay);

//IR sensor setup
const uint8_t IR_Detector_Pin = 14;
IRDecoder decoder(IR_Detector_Pin);

//Robot state setup
enum RobotState {
    romiMoveTo,
    stop,
    armGet,
    armPlace,
    openGripper,
    closeGripper,
    turnLeft,
    turnRight,
    TASKONE,
    lineFollow,
    task1,
    task2,
    task3,
    rotate,
    reset
};
enum RobotState robotState;



//Blue Motor setup
int motorEffort = 400;
double getAngle = 0;
double placeAngle = 0;
bool toDrop;
double targetDistance=10;
double firstDistance;
double secondDistance;

//Gripper servo setup
int servoPin = 5;
int linearPotPin = 18;
int servoStop = 1600;  
int servoJawDown = 1000; 
int servoJawUp = 2000;  
int printDelay = 500;
int linearPotVoltageADC = 500;
int jawOpenPotVoltageADC = 275;
int jawClosedPotVoltageADC = 930;

//Line sensor setup
double KP = 0.1;
int leftReflectance = 22;
int rightReflectance = 20;
int leftValue;
int rightValue;
int error;


Servo32U4 jawServo;

Timer printTimer(printDelay);


//function header
void handleKeyPress(int16_t keyPress);
void moveJawDown();
void moveJawUp();
void motorPos1();
void motorPos2();
void motorPos3();
void motorMoveTo();
void romiMovetoDistance(double targetDistance);
void linetracking();


void setup()
{
  Serial.begin(9600);
  rangefinder.init();
  motor.setup();
  motor.reset();
  decoder.init();
  chassis.init();
  delay(3000);
  jawServo.attach();
  Serial.print("Time (ms)");
  Serial.print("   ");
  Serial.println("Position");
  robotState = stop;
  delay(3000);
  double leftValue = analogRead(leftReflectance);
  double rightValue = analogRead(rightReflectance);
  double error = leftValue-rightValue;
  delay(20);
}


void loop()
{

  int16_t keyPress = decoder.getKeyCode();
  if(keyPress >= 0) handleKeyPress(keyPress);

  switch (robotState)
  {
  //raise the arm
  case armGet:
    Serial.println("armGet");
    if ((motor.getPosition() < (getAngle - 1)) || (motor.getPosition() > (getAngle + 1))){
      motor.moveTo(getAngle);
    }else{
      motor.setEffort(0);
      Serial.println("finish armGet");
      toDrop = false;
      targetDistance = firstDistance;
      delay(20);
      robotState = romiMoveTo;
          delay(20);
    }
  break;

  // move towards the target
  case romiMoveTo:
    //Serial.println("romiMoveTo");
    // this will switch to closeGripper 
    romiMovetoDistance(targetDistance);
  break;

  //Close the gripper
  case closeGripper: 
    Serial.println("close gripper");
    //close the gripper and start to turn
    moveJawUp();
  break;

  case rotate:
    // turn for 180 degrees
    Serial.println("rotate");
    chassis.setMotorEfforts(-70,70);
    delay(2450);
    chassis.setMotorEfforts(0,0);
    toDrop = true;
    targetDistance = secondDistance;
    robotState = romiMoveTo;
  break;

  case armPlace:
    Serial.println("armPlace");
    // lower the arm to place the panel
     if ((motor.getPosition() < (placeAngle - 1)) || (motor.getPosition() > (placeAngle + 1))){
      motor.moveTo(placeAngle);
    }else{
      // if done, go to openGripper
      Serial.println("finish armPlace");
      
      motor.setEffort(0);
      delay(20);
      robotState = openGripper;
    }
  break;

  //Open the gripper
  case openGripper: 
    moveJawDown();
  break;

  case lineFollow:
  linetracking();
  break;

  case task1:
  robotState = armGet;
  break;

  case task2:
  robotState = armGet;
  break;

  case task3:
  robotState = armGet;
  break;

  case reset:
  motor.moveTo(0);
  moveJawDown();
  break;


  //Default case, set everything to 0
  default:
    motor.setEffort(0);
    jawServo.writeMicroseconds(servoStop);
    chassis.setMotorEfforts(0,0);
    break;
  }

}


void handleKeyPress(int16_t keyPress){
  Serial.println("Key: " + String(keyPress));

  // if(keyPress == ENTER_SAVE) idle();

  switch (robotState)
  {
  case stop:
    robotState = stop;
    Serial.println("Robot State: " + String(robotState));
    if (keyPress == NUM_1){
      getAngle = 3240;
      placeAngle = 150;
      //task1: first moving to the house, and then to the staging area
      firstDistance = 14; //distance of to the house
      secondDistance = 10; //distance of to the staging area
      robotState = armGet;
      robotState = task1;
      Serial.println("Robot State: " + String(robotState));
    }
    if (keyPress == NUM_2){
      getAngle = 0;
      placeAngle = 3240;
      //task2: first move to the staging area, and then to the house
      firstDistance = 10;//distance of to the staging area
      secondDistance = 14;//distance of to the house
      robotState = armGet;
      robotState = task2;
      Serial.println("Robot State: " + String(robotState));
    }
    if (keyPress == NUM_3){
      getAngle = 0;
      placeAngle = 3240;
      robotState = armGet;
      robotState = task3;
      Serial.println("Robot State: " + String(robotState));
    }
    if (keyPress == VOLminus){
      robotState = openGripper;
      Serial.println("Robot State: " + String(robotState));
    }
    if (keyPress == VOLplus){
      robotState = closeGripper;
      Serial.println("Robot State: " + String(robotState));
    }
    if (keyPress == NUM_4){
      robotState = romiMoveTo;
      Serial.println("Robot State: " + String(robotState));
    }
    if (keyPress == NUM_5){
      robotState = lineFollow;
      Serial.println("Robot State: " + String(robotState));
    }
    if (keyPress == SETUP_BTN){
     robotState = reset;
     Serial.println("Robot State: " + String(robotState));
    }
    if (keyPress == NUM_9){
     robotState = rotate;
     Serial.println("Robot State: " + String(robotState));
    }
  break;

  case task1:
    if (keyPress == NUM_1){
     robotState = stop;
     Serial.println("Robot State: " + String(robotState));
    }
  break;

  default:
    break;
  }
}

void moveJawDown(){
  jawServo.writeMicroseconds(servoJawDown);
  Serial.println("Moving jaw down...");

  // Print out Pot voltages
  if (linearPotVoltageADC > jawOpenPotVoltageADC) {
    linearPotVoltageADC = analogRead(linearPotPin);
    if (printTimer.isExpired()) {
      Serial.print("linearPotVoltageADC: ");
      Serial.println(linearPotVoltageADC);
    }
  }
  else{
     // Stop servo
    jawServo.writeMicroseconds(servoStop);
    robotState = stop;
  }

  // Print current pot value
  Serial.print("Bottom linearPotVoltageADC Before Delay: ");
  Serial.println(linearPotVoltageADC);
}

void moveJawUp(){
  jawServo.writeMicroseconds(servoJawUp);
  Serial.println("Moving jaw up...");

  if (linearPotVoltageADC < jawClosedPotVoltageADC) {
 
    linearPotVoltageADC = analogRead(linearPotPin);

    if (printTimer.isExpired()) {
      Serial.print("linearPotVoltageADC: ");
      Serial.println(linearPotVoltageADC);
    }     
  }
  else{
    // Stop servo
    jawServo.writeMicroseconds(servoStop);
    delay(20);
    robotState = rotate;
  }

  // Print final pot value
  Serial.print("Final linearPotVoltageADC Before Delay: ");
  Serial.println(linearPotVoltageADC);
}


void romiMovetoDistance(double targetDistance){
   //clears trigin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    //sets high for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH); 
    
  distance = duration*0.034/two;

  if (distance >= targetDistance)
  {
   
    //clears trigin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    //sets high for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    //reads pin 
    duration = pulseIn(echoPin, HIGH); 
    
    distance = duration*0.034/two;
    if(distance > 30) {
    distance = 30;
    }
    Serial.println("distence " + String(distance));
    delay(20);
    
    if (rangeTimer.isExpired()){
      if(abs(oldDistance-distance)>10){
        distance = oldDistance;
        // chassis.setMotorEfforts(-ultrasonicConstant*(distance-5), -ultrasonicConstant*(distance-5));
        delay(20);
        // chassis.setMotorEfforts(-62,-50);
        linetracking();
      }
      else {
        oldDistance = distance;
        // chassis.setMotorEfforts(-ultrasonicConstant*(distance-5), -ultrasonicConstant*(distance-5));
        delay(20);
        // chassis.setMotorEfforts(-62,-50);
        linetracking();
      }
    }
    
  }
  // Stops the robot if it has reached the desired distance
  else{
    chassis.setMotorEfforts(0,0);
    
    if(toDrop == true){
      //if moving to the staging area, go to armPlace
      Serial.println("true, Open gripper");
      delay(50);
      robotState = armPlace;//openGripper
    }else{
      //if moving to the house, close the gripper to grab the panel
      Serial.println("false, close gripper");
      delay(50);
      robotState = closeGripper;//closeGripper
    }
    
  }

}


void linetracking(){
    //chassis.setMotorEfforts(100,100);
  leftValue = analogRead(leftReflectance);
  // Serial.print("Left Value: ");
  Serial.print(leftValue);
  Serial.print("         ");
  rightValue = analogRead(rightReflectance);
  // Serial.print("Right Value: ");
  Serial.println(rightValue);

  // Finding the difference between the
  // left and right sensor
  error = analogRead(leftReflectance)-analogRead(rightReflectance);

  // Right sensor is HIGH
  // Left wheel needs to be faster
  if(error > 100)
  {
    chassis.setMotorEfforts(-50 - abs((KP*error)), -50);
    Serial.print("error greater than 200; ");
    Serial.println(error);
  }
  // Left sensor is HIGH
  // Right wheel needs to be faster
  else if(error < -100)
  {
    chassis.setMotorEfforts(-50, -50 - abs((KP*error)));
    Serial.print("error less than 200; ");
    Serial.println(error);
  }
  // The sensors are the same
  // Wheels need to be the same
  else
  {
    // Serial.println("error between");
    chassis.setMotorEfforts(-60, -50);
    
  }
}