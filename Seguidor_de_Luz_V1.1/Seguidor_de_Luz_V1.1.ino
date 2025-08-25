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

// Pin para la señal de hibernación al ESP32
#define HIBERNATION_SIGNAL_PIN 2

void setup() {
    horizontal.attach(8);
    vertical.attach(12);
    // Configura el pin de señal de hibernación como salida
    pinMode(HIBERNATION_SIGNAL_PIN, OUTPUT);
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

    // === Nueva lógica para la hibernación ===
    // Calcula la suma total de las lecturas para detectar la oscuridad
    int totalLight = lt + rt + ld + rd;
    int lightThreshold = 200; // Umbral de luz (ajustable)
    
    // Si la suma total es menor que el umbral, indica que no hay luz
    if (totalLight < lightThreshold) {
      // Pone el pin D2 en estado ALTO para que el ESP32 entre en hibernación
      digitalWrite(HIBERNATION_SIGNAL_PIN, HIGH);
    } else {
      // Si hay suficiente luz, asegura que el pin D2 esté en estado BAJO
      digitalWrite(HIBERNATION_SIGNAL_PIN, LOW);
      
      // === Lógica de movimiento del seguidor solar ===
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
    }
    delay(dtime);
}
