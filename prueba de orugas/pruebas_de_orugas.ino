#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <SPI.h>
#include <RF24.h>

Adafruit_VL53L0X loxFront = Adafruit_VL53L0X();
Adafruit_VL53L0X loxLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X loxRight = Adafruit_VL53L0X();
Adafruit_VL53L0X loxInclined = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor; // Objeto para el sensor del LIDAR

#define LIDAR_SHUT_PIN 6 // Pin para controlar XSHUT del VL53L0X
#define SAMPLE_SIZE 5

RF24 radio(5, 4); // CE, CSN
const byte address[6] = "00001";

unsigned long startTime;
float totalRotationTime = 15000.0; // Tiempo en milisegundos (15 segundos)

union FloatToBytes {
  float fVal;
  byte bVal[sizeof(float)];
};

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

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Configuración del pin XSHUT del VL53L0X
  pinMode(LIDAR_SHUT_PIN, OUTPUT);
  digitalWrite(LIDAR_SHUT_PIN, LOW); // Mantener apagado al inicio

  Serial.println(F("Iniciando sensores VL53L0X..."));

  if (!loxFront.begin(0x30)) {
    Serial.println(F("Error al inicializar VL53L0X frontal"));
    while (1);
  }
  Serial.println(F("VL53L0X frontal iniciado"));

  if (!loxLeft.begin(0x31)) {
    Serial.println(F("Error al inicializar VL53L0X izquierdo"));
    while (1);
  }
  Serial.println(F("VL53L0X izquierdo iniciado"));

  if (!loxRight.begin(0x32)) {
    Serial.println(F("Error al inicializar VL53L0X derecho"));
    while (1);
  }
  Serial.println(F("VL53L0X derecho iniciado"));

  if (!loxInclined.begin(0x33)) {
    Serial.println(F("Error al inicializar VL53L0X inclinado"));
    while (1);
  }
  Serial.println(F("VL53L0X inclinado iniciado"));

  // Configuración del módulo de radio
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();
}

int filterMedian(int readings[], int size) {
  int sorted[size];
  memcpy(sorted, readings, size * sizeof(int));
  for (int i = 0; i < size - 1; i++) {
    for (int j = i + 1; j < size; j++) {
      if (sorted[i] > sorted[j]) {
        int temp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = temp;
      }
    }
  }
  return sorted[size / 2];
}

//void moveForward() {}
//void stopMotors() {}
//void turnLeft() {}
//void turnRight() {}

void scanWithLidar() {
  // Encender el VL53L0X
  digitalWrite(LIDAR_SHUT_PIN, HIGH);
  delay(100); // Esperar a que el sensor se inicie

  // Iniciar el sensor VL53L0X
  if (!sensor.begin()) {
    Serial.println("Error al inicializar VL53L0X para el LIDAR");
  }
  // sensor.startRangeContinuous(); // No necesitamos modo continuo aquí

  // LIDAR.write(80); // Asumo que LIDAR es un servo, descomentar si lo usas
  startTime = millis();
  float currentAngle = 0.0; // Inicializar el ángulo

  while (currentAngle < 360.0) {
    unsigned long elapsedTime = millis() - startTime;
    currentAngle = (elapsedTime / totalRotationTime) * 360.0;
    if (currentAngle > 360.0) currentAngle = 360.0; // Asegurar no exceder 360

    // Realizar una lectura única del sensor
    VL53L0X_RangingMeasurementData_t measure;
    sensor.rangingTest(&measure, false);
    int distance = -1;

    if (measure.RangeStatus != 4) {
      distance = measure.RangeMilliMeter;
      Serial.print("Angle: ");
      Serial.print(currentAngle);
      Serial.print(", Distance: ");
      Serial.println(distance);
    } else {
      Serial.println("VL53L0X timeout or invalid measurement!");
    }

    // Transmitir los datos del LIDAR por radio (ángulo como float)
    FloatToBytes angleConverter;
    angleConverter.fVal = currentAngle;

    byte data[sizeof(float) + sizeof(int)]; // 4 bytes para float, 2 bytes para int (distance)
    memcpy(data, angleConverter.bVal, sizeof(float));
    data[sizeof(float)] = distance & 0xFF;
    data[sizeof(float) + 1] = (distance >> 8) & 0xFF;
    radio.write(data, sizeof(data));

    delay(10); // Pequeño retraso
  }

  // Detener el servo si lo estás usando
  // LIDAR.write(90);
  delay(100);

  // Apagar el VL53L0X
  digitalWrite(LIDAR_SHUT_PIN, LOW);
}

void loop() {
  int distanceFront_raw = readVL53L0X(loxFront);
  int distanceLeft_raw = readVL53L0X(loxLeft);
  int distanceRight_raw = readVL53L0X(loxRight);
  int distanceInclined = readVL53L0X(loxInclined);

  static int front_readings[SAMPLE_SIZE];
  static int left_readings[SAMPLE_SIZE];
  static int right_readings[SAMPLE_SIZE];
  static int reading_index = 0;

  front_readings[reading_index] = distanceFront_raw;
  left_readings[reading_index] = distanceLeft_raw;
  right_readings[reading_index] = distanceRight_raw;
  reading_index = (reading_index + 1) % SAMPLE_SIZE;

  int front = filterMedian(front_readings, SAMPLE_SIZE);
  int left = filterMedian(left_readings, SAMPLE_SIZE);
  int right = filterMedian(right_readings, SAMPLE_SIZE);

  Serial.print("Distancia Frente: ");
  Serial.print(front);
  Serial.print(" mm, Izquierda: ");
  Serial.print(left);
  Serial.print(" mm, Derecha: ");
  Serial.print(right);
  Serial.print(" mm, Inclinado: ");
  Serial.println(distanceInclined);

  // Algoritmo de navegación basado en reglas mejorado
  if (front < 20 && front != -1) {
    if (left > right && left != -1 && right != -1) {
      // turnLeft();
      Serial.println("Girar a la izquierda (frontal cerca)");
    } else if (right != -1) {
      // turnRight();
      Serial.println("Girar a la derecha (frontal cerca)");
    } else {
      // stopMotors();
      Serial.println("Detener (frontal cerca, sin clara dirección)");
    }
  } else if (left < 15 && left != -1) {
    // turnRight();
    Serial.println("Girar a la derecha (izquierda cerca)");
  } else if (right < 15 && right != -1) {
    // turnLeft();
    Serial.println("Girar a la izquierda (derecha cerca)");
  } else {
    // moveForward();
    Serial.println("Avanzar");
  }

  delay(1000); // Ajusta el tiempo de avance según sea necesario
  // stopMotors(); // El robot se detiene

  scanWithLidar(); // Activar el LIDAR y hacer una toma

  delay(100);
}