#include <MedianFilterLib.h>
#include <Servo.h>
#include <math.h>

Servo myservo;  // create servo object to control a servo
float servoValue;
float prevServo = 77.5; // The servo limits are set arbitrarily at 15 - 150 degrees, 77.5 deg is the middle

//Setup variables for LDR and LED
int photoPin = 0;
float photoValue = 0;
float photoVoltage;
long distEst;
float photoMed;
MedianFilter<float> medianFilter(10);
float desiredPhoto = 575.0;
int photoError;

//Set PID gains
float Kp = 0.1;
float Ki = 0.00;
float Kd = 0.01;

float totalError = 0.0;
float prevError = 0.0;
float proportional = 0.0;
float integral = 0.0;
float derivative = 0.0;
float PID = 0.0;
float interpPID;
float controlSignal;

//Setup Timer 2 and variables for ISR
long timerCount;
long frequency = 1000; // frequency of the interrupt
int reducedFreq = 100; // frequency at which to read photoresistor
int freqLimit = frequency/reducedFreq;
volatile int freqCounter = 0;
volatile bool freqFlag = false;

void setup() {
  noInterrupts();
  myservo.attach(5);
  Serial.begin(115200);

  //Initialize timer 2
  TCCR2A = 0;
  TCCR2B = 0;
  // Set timerCount to the correct value for our interrupt interval
  // preload timer (2^8 - 16MHz/(prescaler*freq))
  timerCount = 256 - 16000000/(64*frequency); //prescaler is 64, desired frequency is 1000 Hz, 8 bit timer has 256 steps
  TCNT2 = timerCount;  // preload timer
  TCCR2B |= (1 << CS22);    // prescaler = 64 - on timer 2, pull high clock select bit two high
  TIMSK2 |= (1 << TOIE2);   // enable timer overflow interrupt
  interrupts(); 
}


ISR(TIMER2_OVF_vect)
{
  interrupts(); //re-enable interrupts
  // preload the timer each cycle
  TCNT2 = timerCount;
  freqCounter = freqCounter + 1;
  if (freqCounter >= freqLimit){
    freqCounter = 0;
    freqFlag = true;
  }
}

void loop() {
  //Read LDR value when interrupt is triggered 
  if (freqFlag == true){
    photoValue = analogRead(photoPin);
    photoVoltage = 5*(photoValue/1023);
    distEst = 500*sqrt(2*((5/photoVoltage)-1));
    photoMed = medianFilter.AddValue(photoValue);
    Serial.println(photoMed);

    photoError = desiredPhoto - photoMed;
    //Serial.println(photoError);
    
    totalError = totalError + photoError;
    if (totalError > 1000){
      totalError = 1000;
    }
    else if (totalError < -1000){
      totalError = -1000;
    }

    proportional = photoError * Kp;
    integral = totalError * Ki;
    derivative = ((photoError - prevError) * reducedFreq) * Kd; // multiply by sampling freq instead of dividing by time
    prevError = photoError;
    PID = proportional + integral + derivative;
    //Serial.println(PID);

    interpPID = (135.0)*((PID)/1023.0); //Linear interpolation to convert PID error signal to angular position error
    //Serial.println(interpPID);
    servoValue = prevServo + interpPID;
    Serial.println(servoValue);

    //Keep servo value within normal boundaries
    if (servoValue > 150.0){
      servoValue = 150.0;
    }
    else if (servoValue < 15.0){
      servoValue = 15.0;
    }
    myservo.write(servoValue);
    prevServo = servoValue;
    freqFlag = false;
  }
}
