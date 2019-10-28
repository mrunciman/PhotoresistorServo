#include <Servo.h>

//Use the Servo library to set up control of the servo motor
Servo myservo;  
float servoValue;
float prevServo = 77.5; // The servo limits are set arbitrarily at 15 - 150 degrees, 77.5 deg is the middle

//Setup variables for LDR and LED
int photoPin = 0;
float photoValue = 0;
float desiredPhoto = 625.0;
int photoError;

//Set PID gains
float Kp = 0.04;
float Ki = 0.00;
float Kd = 0.005;

float totalError = 0.0;
float prevError = 0.0;
float proportional = 0.0;
float integral = 0.0;
float derivative = 0.0;
float PID = 0.0;
float interpPID;

//Set variables for timing of program
unsigned long period = 10000; //period in microseconds
float samplingFreq = 0.01;
unsigned long prevTime;

void setup() {
  myservo.attach(5);
  Serial.begin(115200);
}

void loop() {
  //Read LDR value when interrupt is triggered 
  if (micros() - prevTime >= period){
    prevTime = prevTime + period;
    photoValue = analogRead(photoPin);
    //Serial.println(photoValue);

    photoError = desiredPhoto - photoValue;
    //Serial.println(photoError);
    
    totalError = totalError + photoError;
    proportional = photoError * Kp;
    integral = totalError * Ki;
    derivative = ((photoError - prevError) / samplingFreq) * Kd; // multiply by sampling freq instead of dividing by time
    prevError = photoError;
    PID = proportional + integral + derivative;
    //Serial.println(PID);

    interpPID = (135.0)*((PID)/1023.0); //Linear interpolation to convert PID error signal to angular position error
    //Serial.println(interpPID);
    servoValue = prevServo + interpPID;
    //Serial.println(servoValue);

    //Keep servo value within normal boundaries
    if (servoValue > 150.0){
      servoValue = 150.0;
    }
    else if (servoValue < 15.0){
      servoValue = 15.0;
    }
    //myservo.write(servoValue);
    prevServo = servoValue;
  }
}
