#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Crear objetos para los sensores VL53L0X
Adafruit_VL53L0X loxFront = Adafruit_VL53L0X();
Adafruit_VL53L0X loxLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X loxRight = Adafruit_VL53L0X();
Adafruit_VL53L0X loxInclined = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Inicializa VL53L0X y asigna direcciones I2C únicas
  initVL53L0X(loxFront, 0x30);
  initVL53L0X(loxLeft, 0x31);
  initVL53L0X(loxRight, 0x32);
  initVL53L0X(loxInclined, 0x33);
}

void loop() {
  // Lee datos de los sensores VL53L0X
  int distanceFront = readVL53L0X(loxFront);
  int distanceLeft = readVL53L0X(loxLeft);
  int distanceRight = readVL53L0X(loxRight);
  int distanceInclined = readVL53L0X(loxInclined);

  Serial.print("Distancia Frente: ");
  Serial.print(distanceFront);
  Serial.print(" mm, Izquierda: ");
  Serial.print(distanceLeft);
  Serial.print(" mm, Derecha: ");
  Serial.print(distanceRight);
  Serial.print(" mm, Inclinado: ");
  Serial.println(distanceInclined);

  // Lógica de decisión basada en las lecturas de los sensores
  if (distanceFront > 0 && distanceInclined > 0) { // Verifica que las lecturas sean válidas
    if ((distanceInclined - distanceFront) < 50) { // Ajusta el umbral según sea necesario
      Serial.println("Obstáculo es escalable. Avanzar.");
      enviarComandoMotor('A'); // Comando 'A' para avanzar
    } else {
      Serial.println("Obstáculo no es escalable. Buscar dirección libre.");
      if (distanceLeft > distanceRight) {
        Serial.println("Girar a la izquierda.");
        enviarComandoMotor('L'); // Comando 'L' para girar a la izquierda
      } else {
        Serial.println("Girar a la derecha.");
        enviarComandoMotor('R'); // Comando 'R' para girar a la derecha
      }
    }
  } else {
    Serial.println("No hay obstáculo detectado.");
    enviarComandoMotor('A'); // Comando 'A' para avanzar
  }

  delay(500);
}

void initVL53L0X(Adafruit_VL53L0X &lox, uint8_t address) {
  if (!lox.begin(address)) {
    Serial.print(F("No se pudo encontrar VL53L0X en dirección 0x"));
    Serial.println(address, HEX);
    while (1);
  }
  Serial.print(F("VL53L0X iniciado en dirección 0x"));
  Serial.println(address, HEX);
}

int readVL53L0X(Adafruit_VL53L0X &lox) {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter;
  } else {
    return -1; // Error en la medición
  }
}

