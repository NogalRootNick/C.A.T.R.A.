// Incluye las librerías necesarias
#include <ESP32Servo.h>       // Librería específica para servos en ESP32
#include <Wire.h>             // Para comunicación I2C
#include <Adafruit_VL53L0X.h> // Para el sensor de distancia VL53L0X
#include <Adafruit_MPU6050.h> // Para el acelerómetro/giroscopio MPU6050
#include <Adafruit_Sensor.h>  // Librería base para sensores Adafruit

// Define los pines I2C para el ESP32
// Se han cambiado a GPIO 18 (SDA) y GPIO 19 (SCL) según tu solicitud.
#define I2C_SDA_PIN 18
#define I2C_SCL_PIN 19

// Define el pin para el servo
// Se usará el pin 25, como se indica en tu código original.
#define SERVO_PIN 25

// Crea un objeto para el sensor VL53L0X
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Crea un objeto para el sensor MPU6050
Adafruit_MPU6050 mpu;

// Crea un objeto para el servo
// Usamos la clase Servo de la librería ESP32Servo
Servo myservo;

// Variables para la integración del giroscopio (orientación estimada)
// Este será el ángulo principal para el cálculo de coordenadas.
// Ahora se basa en la rotación del eje Y del giroscopio.
float current_gyro_angle_y = 0.0; // Ángulo estimado en el eje Y (pitch/roll, dependiendo de la orientación física del sensor), en grados
unsigned long last_update_time = 0; // Tiempo de la última lectura para calcular el delta de tiempo

void setup() {
  // Inicializa la comunicación serial a 115200 baudios
  Serial.begin(115200); // Velocidad de baudios configurada a 115200 para coincidir con Python
  while (!Serial) {
    delay(10); // Espera a que el puerto serial se conecte. Necesario para Leonardo/Micro/Zero
  }

  // Comentarios iniciales para depuración, puedes eliminarlos si no los necesitas
  Serial.println("Inicializando sistema...");
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Serial.println("Bus I2C inicializado.");

  Serial.println("Inicializando VL53L0X...");
  if (!lox.begin()) {
    Serial.println(F("¡Error al iniciar el VL53L0X! Comprueba el cableado."));
    while (1);
  }
  Serial.println("VL53L0X inicializado correctamente.");

  // --- Establecer el presupuesto de tiempo para el VL53L0X ---
  // Un presupuesto de tiempo más largo puede mejorar la precisión a costa de la velocidad.
  // Para rangos cortos, 20000 microsegundos (20ms) es un buen punto de partida.
  lox.setMeasurementTimingBudgetMicroSeconds(20000); 
  // Los períodos de pulso VCSEL también influyen en el rango y la precisión.
  // Estos valores son comunes para optimizar el rendimiento de corto alcance.
  lox.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18); 
  lox.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14); 
  Serial.println("VL53L0X configurado para corto alcance y precisión mejorada.");


  Serial.println("Inicializando MPU6050...");
  if (!mpu.begin()) {
    Serial.println("¡Error al iniciar el MPU6050! Comprueba el cableado.");
    while (1);
  }
  Serial.println("MPU6050 inicializado correctamente.");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  myservo.attach(SERVO_PIN);
  myservo.write(90); // Servo de 360 grados girando
  Serial.println("Servo de rotación continua establecido a 90 grados para girar.");

  last_update_time = millis();
  Serial.println("Sensores listos para operar. Imprimiendo solo coordenadas (X, Y) si hay detección válida.");
}

void loop() {
  // Calcula el delta de tiempo para la integración del giroscopio
  unsigned long current_time = millis();
  float dt = (current_time - last_update_time) / 1000.0; // Delta de tiempo en segundos
  last_update_time = current_time;

  // --- Lectura del VL53L0X ---
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  float distance_mm = 0.0;
  bool valid_detection = false; // Bandera para indicar si la detección es válida

  // --- Imprime el estado RAW del VL53L0X para depuración ---
  // Ahora imprime el Status y la Distancia en mm.
  Serial.print("RAW:");
  Serial.print(measure.RangeStatus); // Imprime el Status
  Serial.print(",");                  // Separador
  Serial.print(measure.RangeMilliMeter); // Imprime la Distancia
  Serial.println(" mm");             // Añade " mm" y un salto de línea

  // Lógica de validación de la distancia:
  // 1. El estado del rango NO DEBE ser 4 (fuera de rango).
  //    Esto incluye estados como 0 (sin error), 1, 2, 3 (señal deficiente, etc.).
  // 2. La distancia debe ser MAYOR o IGUAL a 4.5 mm (umbral mínimo).
  // 3. La distancia debe ser MENOR o IGUAL a 150 mm (umbral máximo).
  if (measure.RangeStatus != 4 && measure.RangeMilliMeter >= 4.5 && measure.RangeMilliMeter <= 150) {
    distance_mm = (float)measure.RangeMilliMeter;
    valid_detection = true; // La detección es válida
  } else {
    // Si no cumple las condiciones, no hay detección válida para el mapa
    distance_mm = 0.0; // Se mantiene en 0.0, aunque no se usará para imprimir
    valid_detection = false;
  }

  // --- Lectura del MPU6050 ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Integra el valor del giroscopio en el eje Y para obtener una estimación del ángulo.
  current_gyro_angle_y += (g.gyro.y * (180.0 / PI)) * dt;

  // Normaliza el ángulo para mantenerlo entre 0 y 360 grados
  if (current_gyro_angle_y >= 360.0) {
    current_gyro_angle_y -= 360.0;
  } else if (current_gyro_angle_y < 0.0) {
    current_gyro_angle_y += 360.0;
  }

  // --- Cálculo de Coordenadas Cartesianas (X, Y) ---
  float angle_radians = current_gyro_angle_y * (PI / 180.0);

  float coord_x = distance_mm * cos(angle_radians);
  float coord_y = distance_mm * sin(angle_radians);

  // --- Control del Servo ---
  myservo.write(90); // Mantiene el servo de 360 grados girando

  // --- Imprime solo los valores de las Coordenadas X, Y, SOLO SI HAY UNA DETECCIÓN VÁLIDA ---
  if (valid_detection) {
    Serial.print("COORD:"); // Prefijo para identificar las coordenadas
    Serial.print(coord_x, 2); // Imprime X con 2 decimales
    Serial.print(",");        // Separador
    Serial.print(coord_y, 2); // Imprime Y con 2 decimales
    Serial.println();         // Salto de línea para la siguiente lectura
  }

  delay(50); // Pequeña pausa
}
