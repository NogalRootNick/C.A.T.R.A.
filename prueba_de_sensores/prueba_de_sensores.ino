#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define I2C_SDA1 21
#define I2C_SCL1 22

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(9600);
  Wire.begin(I2C_SDA1, I2C_SCL1);
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