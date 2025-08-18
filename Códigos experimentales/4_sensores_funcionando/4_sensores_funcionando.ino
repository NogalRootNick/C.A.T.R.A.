#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Definimos los pines I2C para el ESP32-C3
// SDA -> GPIO8, SCL -> GPIO9
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

// Definimos los pines XSHUT para cada sensor con los nuevos GPIOs
const int XSHUT_PINS[] = {0, 1, 3, 4};
const int NUM_SENSORS = 4;

// Definimos las nuevas direcciones I2C para cada sensor
// Se evitan las direcciones prohibidas: 0x00, 0x1f, 0x3c, 0x14, 0x15, 0x33, 0x34
const uint8_t NEW_I2C_ADDRESSES[] = {0x30, 0x31, 0x32, 0x35};

// Creamos un arreglo de objetos de sensor
Adafruit_VL53L0X lox[NUM_SENSORS];

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("Iniciando configuracion de sensores...");

  // Inicializamos el bus I2C en los pines especificados
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // Ponemos todos los sensores en estado de apagado (reset)
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(XSHUT_PINS[i], OUTPUT);
    digitalWrite(XSHUT_PINS[i], LOW);
  }
  delay(100);

  // Inicializamos cada sensor individualmente y le asignamos una nueva direccion
  for (int i = 0; i < NUM_SENSORS; i++) {
    // Activamos un sensor a la vez
    digitalWrite(XSHUT_PINS[i], HIGH);
    delay(100);

    // Intentamos inicializar el sensor con la nueva direccion
    if (!lox[i].begin(NEW_I2C_ADDRESSES[i])) {
      Serial.print("Error al inicializar el sensor ");
      Serial.println(i);
      while(1);
    }
    
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" inicializado con exito en 0x");
    Serial.println(NEW_I2C_ADDRESSES[i], HEX);
  }

  Serial.println("Configuracion de todos los sensores completada.");
}

void loop() {
  // Bucle para rotar entre los 4 sensores
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("--------------------------------");
    Serial.print("\nLeyendo sensor ");
    Serial.print(i);
    Serial.print(" (Direccion: 0x");
    Serial.print(NEW_I2C_ADDRESSES[i], HEX);
    Serial.println(")");

    // Creamos un timestamp para medir 2 segundos sin bloquear el codigo
    unsigned long startTime = millis();
    while (millis() - startTime < 2000) {
      // Objeto para almacenar la medicion
      VL53L0X_RangingMeasurementData_t measure;

      // Usamos el setAddress para asegurar que estamos leyendo del sensor correcto
      lox[i].setAddress(NEW_I2C_ADDRESSES[i]);
      lox[i].rangingTest(&measure, false);

      if (measure.RangeStatus != 4) {
        Serial.print("  Distancia: ");
        Serial.print(measure.RangeMilliMeter);
        Serial.println(" mm");
      } else {
        Serial.println("  Fuera de rango o error.");
      }
      delay(100); // Pequeña pausa entre mediciones para no saturar
    }

    Serial.println("\n--------------------------------");
    Serial.println("Cambio de sensor");
    Serial.println("--------------------------------\n");
    delay(100);
  }
}