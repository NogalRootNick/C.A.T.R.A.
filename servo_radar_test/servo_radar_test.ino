#include <Servo.h>

Servo myservo;  // create servo object to control a servo

int val;    // variable to read the value from the analog pin

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop() {  
  myservo.write(90);                  // sets the servo position according to the scaled value
  delay(3500); 
  myservo.write(0);                  // sets the servo position according to the scaled value
  delay(3500);                           // waits for the servo to get there
}