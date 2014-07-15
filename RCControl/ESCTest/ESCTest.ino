// Controlling a servo position using a potentiometer (variable resistor) 

#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
 
int potpin = 7;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin 
 
void setup() 
{ 
	Serial.begin(9600);
  myservo.attach(5);  // attaches the servo on pin 9 to the servo object 
} 
 
void loop() 
{ 
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023) 
  val = map(val, 0, 1023, 850, 1800);     // scale it to use it with the servo (value between 0 and 180) 
  Serial.println(val);				   // Out value to serial port.
  myservo.write(val);                  // sets the servo position according to the scaled value 
  delay(15);                           // waits for the servo to get there 
} 
