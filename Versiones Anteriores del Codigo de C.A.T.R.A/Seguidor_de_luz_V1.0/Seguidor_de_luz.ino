#include <Servo.h>  

Servo horizontal; // Horizontal Servo Motor
int servohori = 180;  
int servohoriLimitHigh = 175;
int servohoriLimitLow = 5;

Servo vertical; // Vertical Servo Motor
int servovert = 45;  
int servovertLimitHigh = 100;
int servovertLimitLow = 1;

// LDR pin connections
int ldrlt = A1; // Bottom Left LDR
int ldrrt = A2; // Bottom Right LDR
int ldrld = A0; // Top Left LDR
int ldrrd = A3; // Top Right LDR

void setup() {
  horizontal.attach(8);
  vertical.attach(12);
  horizontal.write(180);
  vertical.write(45);
  delay(2500);
}

void loop() {
  int lt = analogRead(ldrlt); // Top left
  int rt = analogRead(ldrrt); // Top right
  int ld = analogRead(ldrld); // Bottom left
  int rd = analogRead(ldrrd); // Bottom right

  int dtime = 10;  
  int tol = 90; // Tolerance value for adjustment

  int avt = (lt + rt) / 2; // Average value of top sensors
  int avd = (ld + rd) / 2; // Average value of bottom sensors
  int avl = (lt + ld) / 2; // Average value of left sensors
  int avr = (rt + rd) / 2; // Average value of right sensors

  int dvert = avt - avd; // Difference between top and bottom
  int dhoriz = avl - avr; // Difference between left and right

  if (abs(dvert) > tol) {  
    if (avt > avd) {
      servovert = ++servovert;
      if (servovert > servovertLimitHigh) servovert = servovertLimitHigh;
    } else {
      servovert = --servovert;
      if (servovert < servovertLimitLow) servovert = servovertLimitLow;
    }
    vertical.write(servovert);
  }

  if (abs(dhoriz) > tol) {  
    if (avl > avr) {
      servohori = --servohori;
      if (servohori < servohoriLimitLow) servohori = servohoriLimitLow;
    } else {
      servohori = ++servohori;
      if (servohori > servohoriLimitHigh) servohori = servohoriLimitHigh;
    }
    horizontal.write(servohori);
  }

  delay(dtime);
}