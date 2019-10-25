# Photoresistor, LED and Servo

A photoresistor (also known as a Light Dependent Resistor (LDR) or a photocell), an LED and a servo motor are connected to the microcontroller. The LED and LDR will be used together so that when an object in front of the LED, the light from the LED reflects back to the photoresistor. The resistance of the LDR decreases with increased light exposure and a potential divider circuit is used in order to read a voltage corresponding to this varying resistance. The measured voltage gives us some indication of the distance from the LED/LDR pair to the object in front of it. 

In this example the servo motor will control the position of the LED/LDR sensor and we want it to maintain a specific distance from an object. The angular position of the servo depends on the reading taken from the sensor and we can use this to perform PID control.

First we need to connect the components as follows:
![Schematic diagram](https://github.com/mrunciman/PhotoresistorServo/blob/master/Schematic.png "Schematic Diagram")


