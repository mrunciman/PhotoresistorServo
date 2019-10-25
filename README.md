# Photoresistor, LED and Servo

In this example the servo motor will control the position of the LED/LDR sensor and we want it to maintain a specific distance from an object. The angular position of the servo depends on the reading taken from the sensor and we can use this to perform PID control.

A photoresistor (also known as a Light Dependent Resistor (LDR) or a photocell), an LED and a servo motor are connected to the microcontroller as follows:
![Schematic diagram](https://github.com/mrunciman/PhotoresistorServo/blob/master/Schematic.png "Schematic Diagram")

The LED and LDR will be used together so that when an object in front of the LED, the light from the LED reflects back to the photoresistor. The resistance of the LDR decreases with increased light exposure and a potential divider circuit is used in order to read a voltage corresponding to this varying resistance. The measured voltage gives us some indication of the distance from the LED/LDR pair to the object in front of it. This information is then used to update the servo's angular position to correct for any error between the distance to the object currently measured and the desired distance.

## In the Code:
We will need to include the Servo library to easily interface with the servo motor. Pulse Width Modulation (PWM) is used to control the angular position of a servo. The setup of this is handled for us by the library but be aware that the Servo library uses Timer1, this will be important if we want to use timers to generate interrupts.

Variables are initialised for the various components involved, taking care to use the correct types as different variables have different requirements on precision, size and even where they are used later on in the code.

In this example an overflow interrupt is used on timer2 so that events take place at 100 HZ consistently. This is configured in setup, as well as the connection of the servo on pin 5. 
An Interrupt Service Routine (ISR) is set up to determine what happens when an interrupt occurs.

In the main loop, if the 'flag' is set to let us know the correct time period has passed, the voltage across R1 is read and an error signal is calculated. This is then used to calculate the proportional, integral and derivative contributions to the PID control signal.
The control signal is converted to an angular position value that is output to the servo motor, once it has been checked that it is within a reasonable range.

The PID gains can then be tuned using a given method so that the servo motor behaves in a desired way e.g. low steady state error, acceptable overshoot, low rise time and settle time, good stability.







