/*
 * Autonomous ice-cream truck robot that automatically releases cat food at specified intervals.
 * 
 * Source codes that were modified for use in this program:
 * Release of food code in releaseFood() function: https://www.instructables.com/Automatic-Arduino-Pet-Feeder/
 * Timer mechanism: https://forum.arduino.cc/t/arduino-temperature-sensor-sampling-in-every-5-minutes/151123/6
 * Autonomous robot movement: SIK Guide Circuit 5C
  */

#include <Servo.h>  //include the servo library
#define HALFMIN (1000UL * 30 * 1) //dispense food at every 30 second interval

unsigned long rolltime = millis() + HALFMIN; //count time to next food dispensing
unsigned long currentMillis = 0;  //current time, as determined by value of millis() in each iteration of loop()
bool wasStopped = false; //record whether switch was previously in the off position

const int buzzerPin = 4;  //use pin 4 to control the piezzo buzzer
const int switchPin = 7;  //use pin 7 to control switch that turns the robot on and off

//use pins 5 and 6 to control ultrasonic distance sensor
const int trigPin = 6;
const int echoPin = 5;

float distance = 0; //store measurement calculated by the distance sensor

Servo myservo;  //create a servo object

//the right motor will be controlled by the motor A pins on the motor driver.
const int AIN1 = 13;  //control pin 1 on the motor driver for the right motor
const int AIN2 = 12;  //control pin 2 on the motor driver for the right motor
const int PWMA = 11;  //speed control pin on the motor driver for the right motor

//the left motor will be controlled by the motor B pins on the motor driver.
const int PWMB = 10;  //speed control pin on the motor driver for the left motor
const int BIN2 = 9; //control pin 2 on the motor driver for the left motor
const int BIN1 = 8; //control pin 1 on the motor driver for the left motor

//robot behaviour variables
unsigned long backupTime = 500; //length of time that the robot will back up when it senses an object
unsigned long turnTime = 700; //length of time that the robot will turn once it has backed up
unsigned long takePause = 200;  //length of time robot will pause

void setup() {
  pinMode(trigPin, OUTPUT); //sends ultrasonic pulses out from the distance sensor
  pinMode(echoPin, INPUT);  //senses when the pulses reflect back to the distance sensor

  pinMode(switchPin, INPUT_PULLUP); //senses whether the switch is flipped


  //Set the motor control pins as outputs.
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  
  Serial.begin (9600);  //set up a serial connection with the computer
  pinMode(buzzerPin, OUTPUT); //set the buzzer pin to output
  myservo.attach(3);  //use pin 3 to control the servo
  myservo.write(0); //set servo to position 0
}

void loop() {

  //record current time
  currentMillis = millis();
 
  //detect the distance read by the distance sensor 
  distance = getDistance();  

  //if the switch is in the ON position
  if (digitalRead(switchPin) == LOW) { 

    //check if wasStopped is true, meaning switch had previously been in the off position
    if (wasStopped){ 

      //reset rolltime variable to current time plus food dispensing interval time
      rolltime = millis() + HALFMIN; 

      //reset value of wasStopped
      wasStopped = false;
    }
  
    //if distance sensor detects an obstacle and it is time to release food
    if ((distance <= 20) && ((long)(millis() - rolltime) >= 0)) {
      
      //stop robot movement for a moment
      while(millis() < (currentMillis + takePause)) {
        stopRobot();
      }

      //play song, release food and reset food release timer
      playSong();
      releaseFood();
      rolltime += HALFMIN;

      avoidObstacle();  //move away from obstacle

    //if an object is detected and food does not need to be released 
    } else if ((distance <= 20) && ((long)(millis() - rolltime) < 0)) {  

      //stop robot movement for a moment
      while(millis() < (currentMillis + takePause)) {
        stopRobot();
      }

      avoidObstacle();  //move away from obstacle

    //if no obstacle is detected and it is time to release food
    } else if ((distance > 20) && ((long)(millis() - rolltime) >= 0) ) { 
      
      //stop robot movement for a moment      
      while(millis() < (currentMillis + takePause)) { 
        stopRobot();
      }

      //play song, release food and reset food release timer
      playSong();
      releaseFood();
      rolltime += HALFMIN;
    
    //else drive forward as no obstacle is detected
    } else {  
      driveForward();
    }

  //stop robot if switch is in OFF position, set wasStopped to true
  } else {  
    stopRobot();
    wasStopped = true;
  }
  delay(50);  //wait 50 milliseconds between readings
  
}

//stops the robot's motors
void stopRobot() {
  rightMotor(0);
  leftMotor(0);
}

//drives the robot forward
void driveForward(){
  rightMotor(255); 
  leftMotor(-255); 
}

//avoids obstacle by driving robot backward followed by turning left
void avoidObstacle(){
  
  //back up while current time is less than start of loop() time + backupTime value
  while (millis() < (currentMillis + backupTime)) {
    rightMotor(-255);
    leftMotor(255);
  }
  
  //turn away from obstacle while current time is less than start of loop() time + turnTime value
  while (millis() < (currentMillis + backupTime + turnTime)) {
    rightMotor(255);
    leftMotor(255);
  }
}


//releases food by having servo motor momentarily move object blocking food container opening
void releaseFood(){
  myservo.write(180); //move object away from container opening
  delay(250);
  myservo.write(0); //move object back to cover container opening
}


//plays song sequence simulating an ice-cream truck song
void playSong() {
  tone(buzzerPin, 873, 100); //A5
  delay(175);
  tone(buzzerPin, 433, 100); //A4
  delay(175);
  tone(buzzerPin, 867, 700); //A5
  delay(175);  
  tone(buzzerPin, 886, 100); //A5
  delay(175);

  tone(buzzerPin, 434, 100); //B4
  delay(175);
  tone(buzzerPin, 867, 100); //A5
  delay(175);
  tone(buzzerPin, 647, 400); //E5
  delay(175);
  tone(buzzerPin, 108, 200); //A2
  delay(175);
  tone(buzzerPin, 576, 100); //D5
  delay(175);
  tone(buzzerPin, 144, 200); //D3
  delay(175);
  
  tone(buzzerPin, 215, 300); //A3
  delay(175);
  tone(buzzerPin, 715, 300); //F5
  delay(175);
  tone(buzzerPin, 647, 550); //E5
  delay(175);
 
  tone(buzzerPin, 215, 100); //A3
  delay(175);
  tone(buzzerPin, 645, 250); //E3
  delay(175);
  tone(buzzerPin, 861, 400); //A5
  delay(175);
  tone(buzzerPin, 966, 300); //B5
  delay(175);
  
  tone(buzzerPin, 216, 450); //A3
  delay(175);
  tone(buzzerPin, 1080, 500); //C#6
  delay(175);
  tone(buzzerPin, 976, 100); //B5
  delay(175);
  tone(buzzerPin, 107, 100); //A2
  delay(175);
  tone(buzzerPin, 881, 300); //A5
  delay(175);
  tone(buzzerPin, 959, 300); //A#5
  delay(175);
  tone(buzzerPin, 1074, 400); //C6
  delay(175);
  tone(buzzerPin, 960, 450); //B5
  delay(175);
}

//controls right motor, takes motor speed as input
void rightMotor(int motorSpeed){
 
  //if the motor should drive forward (positive speed)
  if (motorSpeed > 0){
    digitalWrite(AIN1, HIGH);  //set pin 1 to high
    digitalWrite(AIN2, LOW);  //set pin 2 to low
  }
  //if the motor should drive backward (negative speed)
  else if (motorSpeed < 0){
    digitalWrite(AIN1, LOW);  //set pin 1 to low
    digitalWrite(AIN2, HIGH);  //set pin 2 to high
  }
  //if the motor should stop
  else{
    digitalWrite(AIN1, LOW);  //set pin 1 to low
    digitalWrite(AIN2, LOW);  //set pin 2 to low
  }
  //now that the motor direction is set, drive it at the entered speed
  analogWrite(PWMA, abs(motorSpeed));                 
}

//controls left motor, takes motor speed as input
void leftMotor(int motorSpeed){
  
  //if the motor should drive forward (positive speed)
  if (motorSpeed > 0){
    digitalWrite(BIN1, HIGH);  //set pin 1 to high
    digitalWrite(BIN2, LOW);  //set pin 2 to low
  }
  //if the motor should drive backward (negative speed)
  else if (motorSpeed < 0){
    digitalWrite(BIN1, LOW);  //set pin 1 to low
    digitalWrite(BIN2, HIGH);  //set pin 2 to high
  }
  //if the motor should stop
  else{
    digitalWrite(BIN1, LOW);  //set pin 1 to low
    digitalWrite(BIN2, LOW);  //set pin 2 to low
  }
  //now that the motor direction is set, drive it at the entered speed
  analogWrite(PWMB, abs(motorSpeed));                 
}

//returns the distance measured by the HC-SR04 distance sensor
float getDistance(){
  
  float echoTime;  //variable to store the time it takes for a ping to bounce off an object
  float calculatedDistance;  //variable to store the distance calculated from the echo time

  //send out an ultrasonic pulse that's 10ms long
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  //use the pulsein command to see how long it takes for the pulse to bounce back to sensor
  echoTime = pulseIn(echoPin, HIGH);  
  
  //calculate distance of object that reflected the pulse (half bounce time multiplied by speed of sound)
  calculatedDistance = echoTime / 148.0;  

  return calculatedDistance;  //send back the distance that was calculated
}
