/*
**
**  Balancing boll robot using Makeblock modules and Arduino PID library
**  from Kreativadelar.se, Sweden
**
**  This code is written "quick & dirty" and should not be as a guide
**  in how to program an Arduino. Feel free to change code as you like
**  and share with your friends.
**
**  If you want to share your code changes, please e-mail them to
**  info@kreativadelar.se and we will put them on our web for other
**  customers to download.
**
**  (C) Kreativadelar.se 2015, Sweden, Patrik
**  http://www.kreativadelar.se
**
**  To use this code you need the following libraries: 
**  
**  Arduino PID Library which can be
**  downloaded free from https://github.com/br3ttb/Arduino-PID-Library/zipball/master
**  
**  Makeblock Library which can be  
**  downloaded free from https://github.com/Makeblock-official/Makeblock-Library/archive/master.zip
**  
**  We have used the PID Front End to find the PID parameters and there is
**  some extra code for that. All code that was possible to move we have moved to the PID_Front_End file.
**
**  Version 1.0, Initial public release, July 2015
**
**  This example code is in the public domain.
**
*/

#include <PID_v1.h>
#include <Makeblock.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Servo.h>

/********************************************
 * Settings
 ********************************************/

#define ServoAngleHorz    136
#define SetPointDistance  11

float Kp = 0.95;          //Initial Proportional Gain
float Ki = 0;             //Initial Integral Gain
float Kd = 0.7;           //Intitial Derivative Gain

MeUltrasonicSensor ultraSensor(PORT_7);     // Ultrasonic module can ONLY be connected to port 3, 4, 6, 7, 8 of base shield.
MePort port(PORT_6);                        // Define a port object to be used by the servo
Servo myServo;                              // create servo object to control a servo 
int servoPin =  port.pin1();                // attaches the servo on PORT_6 SLOT1 to the servo object

/********************************************
 * Variables 
 ********************************************/
                                                  
double Setpoint, Input, Output, ServoOutput;  
unsigned long serialTime;                                            // This will help us know when to talk with processing (PID Front End)

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);           // Initialize PID object, which is in the class PID.


/********************************************
 * Setup 
 ********************************************/
void setup() {
  // put your setup code here, to run once:
  delay(2000);
  myServo.attach(servoPin);                                          // Attaches the servo on servopin
  myServo.write(ServoAngleHorz);                                     // Move arm to horizontal position

  delay(1000);
  Serial.begin(9600);

  for(int i = 0; i<5; i++){
    Input = readPosition();                                          // Calls function readPosition() and sets the balls  
    delay(100);                                                      // position as the input to the PID algorithm                                                             
  }
  
  Setpoint = SetPointDistance;
  myPID.SetMode(AUTOMATIC);                                          // Set PID object myPID to AUTOMATIC 
  myPID.SetOutputLimits(-40,40);                                     // Set Output limits to -40 and 40 degrees. 
}

/********************************************
 * Loop
 ********************************************/
void loop() {

  Input = readPosition();                                           // Read ball distance from ultrasonic sensor
 
  myPID.Compute();                                                  // Computes Output in range of -40 to 40 degrees
  
  myServo.write(ServoAngleHorz+Output);                             // Writes value of Output to servo

  // send-receive with processing if it's time (PID Front End)
  if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }

  delay(100);                                                       // The minimal measure interval is 100 milliseconds for
}                                                                   // the ultrasonic sensor

/********************************************
 * Function to read sensor data
 ********************************************
 * Uses a simple running avrage filter
 * to smooth the value.
 * 
 * We also validat the value
 ********************************************/
double readPosition(){
  static double lastValidRead = 0; 
  static double values[] = {0,0,0,0,0};
  static int count = 0;
  static double value = 0;

  value = ultraSensor.distanceCm();

  // Check if valid distance 
  if(value < 40 && value > 0){
    lastValidRead = value;
    values[count] = value;
  }else{
    values[count] = lastValidRead;
  }

  count++;

  if(count > 4 ) count = 0; 

  return (values[0] + values[1] + values[2] + values[3] + values[4]) / 5; 
}

