#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Servo.h>

// Crear instancias para el sensor y el servo
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Servo servoMotor;

// Pin del servo
const int servoPin = 9; // Cambia este pin según tu configuración

// Tiempo total para un giro completo (en milisegundos)
const int giroCompletoMs = 15000;

void setup() {
  Serial.begin(115200);

  // Configurar el servo
  servoMotor.attach(servoPin);

  // Iniciar comunicación con el sensor VL53L0X
  if (!lox.begin()) {
    Serial.println("¡Error al iniciar el VL53L0X!");
    while (1);
  }
  Serial.println("VL53L0X iniciado correctamente.");
}

void loop() {
  scanWithLidar();
  delay(10000); // Esperar un tiempo antes de la siguiente medición
}


void scanWithLidar() {
  Serial.println("Iniciando escaneo con LIDAR...");

  // Iniciar medición continua
  lox.startRangeContinuous();
  Serial.println("Sensor en modo de medición continua.");

  // Hacer girar el servo
  long startTime = millis();
  servoMotor.write(90); // Gira el servo continuamente

  // Realizar medición continua mientras el servo gira
  while (millis() - startTime < giroCompletoMs) {
    // Leer distancia mientras el servo gira
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);

    if (measure.RangeStatus == 0) {
      // Distancia válida, imprimirla junto con el tiempo para calcular el ángulo
      long elapsedTime = millis() - startTime;
      float angle = (360.0 * elapsedTime) / giroCompletoMs;
      Serial.print("Ángulo: ");
      Serial.print(angle);
      Serial.print("°, Distancia: ");
      Serial.print(measure.RangeMilliMeter);
      Serial.println(" mm");
    } else {
      Serial.println("Error en la medición.");
    }
  }

  // Detener el servo
  servoMotor.write(0); // Detener el servo
  Serial.println("Servo detenido.");

  // Detener mediciones del sensor (FALTA ESTA LÍNEA EN EL ORIGINAL)
  lox.stopRangeContinuous();
  Serial.println("Medición del sensor detenida.");
}