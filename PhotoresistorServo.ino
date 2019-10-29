/*
  Read signal from photoresistor and alter Servo angle using PID control.
*/

#include <Servo.h>            //To use a library it has to be included at the beginning of the program

//Use the Servo library to set up control of the servo motor
Servo myservo;                //This will allow us to directly send the angle we want the servo to move to
float servoValue;             //This will be the angle in degrees that the servo moves to
float prevServo = 77.5;       //Initial value for the servo to move to. The servo limits are set 
                              // arbitrarily at 15 - 150 degrees, 77.5 deg is the middle.

//Setup variables for LDR and LED
int photoPin = 0;             //Pin number for LDR signal from voltage divider
float photoValue = 0;         //The value of the signal from LDR
float desiredPhoto = 625.0;   //Our desired LDR value / set point
int photoError;               //The error between the set point and the current value

//Set PID gains
float Kp = 0.04;              //These gains multiply the P, I and D contributions to the 
float Ki = 0.00;              // control signal, so a value of 0.0 turns that
float Kd = 0.005;             // 'contribution' off.

float totalError = 0.0;       //A running total of the error, used for the integral control signal
float prevError = 0.0;        //The value of error calculated for the previous sample.
float proportional = 0.0;     //Initial values for the P, I and D control signals
float integral = 0.0;
float derivative = 0.0;
float PID = 0.0;              //The PID control signal, the sum of P, I and D
float interpPID;              //Linear interpolation of the PID value to convert it to an angle for the servo.
                              // This is an approximation to the real transfer function.

//Set variables for timing of program
unsigned long period = 10000; //The period, in microseconds
float periodSec = 0.01;    //The period in seconds
unsigned long prevTime;       //A running count of the time elapsed

//The setup code, which executes only once.
void setup() {
  myservo.attach(5);          //Tells the Servo library that the servo is connnected to pin 5
  Serial.begin(115200);       //Start Serial communications at baud rate of 115200, needed when 
                              // sending values by USB.
}

//The main loop of the program that will execute continuously.
//Note that lines "Serial.println(_____);" are used for debugging - use and add these to see 
// where things are going wrong.
void loop() {

  if (micros() - prevTime >= period){  //Check whether sufficient time has passed
    prevTime = prevTime + period;  //Add the time elapsed to the running total
    photoValue = analogRead(photoPin);  //Read signal from pin 0.
    //Serial.println(photoValue);

    photoError = desiredPhoto - photoValue;  //Calculate the error signal
    //Serial.println(photoError);

    //Calculate the error signal (insert your code here)
     
    //Calculate proportional term (insert your code here)

    //Calculate integral term (insert your code here)

    //Calculate derivative term  (insert your code here)

    PID = proportional + integral + derivative;  //Sum P, I and D to get control signal
    //Serial.println(PID);

    interpPID = (135.0)*((PID)/1023.0); //Linear interpolation to convert PID signal to angular position error
    //Serial.println(interpPID);
    servoValue = prevServo + interpPID;  //Move from current position by a given amount, determined by PID
    //Serial.println(servoValue);

    //Keep servo value within safe boundaries to protect physical motor - between 15 and 150 degrees.
    if (servoValue > 150.0){
      servoValue = 150.0;
    }
    else if (servoValue < 15.0){
      servoValue = 15.0;
    }
    myservo.write(servoValue);  //Move the servo motor to the new angular position.
    prevServo = servoValue;  //Save the new position for the next loop.
  }
}
