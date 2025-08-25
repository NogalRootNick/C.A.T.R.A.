#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define SDA0_PIN GPIO_NUM_21
#define SCL0_PIN GPIO_NUM_22

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(9600);
    if (!lox.begin()) {
    Serial.println(F("Error al iniciar el sensor VL53L0X"));
    while (1);
  }
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distancia (mm): ");
    Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println("Fuera de rango");
  }

  delay(100); // Esperar 100 milisegundos 
}