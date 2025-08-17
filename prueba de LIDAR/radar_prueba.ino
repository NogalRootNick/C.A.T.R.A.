#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h> // Necesario para la librería del MPU6050 de Adafruit

// Objeto para el sensor de distancia VL53L0X
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Objeto para el giroscopio/acelerómetro MPU6050
Adafruit_MPU6050 mpu6050;

// Variables para almacenar los datos del MPU6050
sensors_event_t a, g, temp;
VL53L0X_RangingMeasurementData_t lox_timing; // Declare lox_timing here

void setup() {
  Serial.begin(9600);

  Wire.begin();

  if (!lox.begin()) {
    Serial.println("Error al inicializar el sensor VL53L0X.");
    while (1);
  }

  if (!mpu6050.begin()) {
    Serial.println("Error al inicializar el sensor MPU6050.");
    while (1);
  }

  mpu6050.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu6050.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu6050.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  lox.rangingTest(&lox_timing);
  if (lox_timing.RangeStatus != 0) {
    Serial.print("Error en la medición de distancia: ");
    Serial.println(lox_timing.RangeStatus);
  }
  uint16_t distance = lox.readRange();

  mpu6050.getEvent(&a, &g, &temp);

  Serial.print("Distancia (mm): ");
  Serial.print(distance);
  Serial.print(" | Aceleración (x, y, z): ");
  Serial.print(a.acceleration.x);
  Serial.print(", ");
  Serial.print(a.acceleration.y);
  Serial.print(", ");
  Serial.print(a.acceleration.z);
  Serial.print(" | Giroscopio (x, y, z): ");
  Serial.print(g.gyro.x);
  Serial.print(", ");
  Serial.print(g.gyro.y);
  Serial.print(", ");
  Serial.println(g.gyro.z);

  delay(1000);
}
