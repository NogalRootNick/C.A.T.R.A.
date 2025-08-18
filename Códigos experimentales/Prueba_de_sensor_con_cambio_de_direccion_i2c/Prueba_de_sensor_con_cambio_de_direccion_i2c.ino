#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Definimos los pines I2C para el ESP32-C3
// SDA -> GPIO8
// SCL -> GPIO9
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

// Definimos el pin XSHUT conectado al sensor
const int XSHUT_PIN = 3;

// Creamos el objeto del sensor de distancia
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  // Inicializamos la comunicación serial para depuración
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println("Prueba de sensor VL53L0X con cambio de direccion.");

  // Configuramos el pin XSHUT como salida
  pinMode(XSHUT_PIN, OUTPUT);
  
  // Inicializamos el bus I2C en los pines especificados
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // Mantenemos el sensor en estado de apagado (reset)
  digitalWrite(XSHUT_PIN, LOW);
  delay(100);

  // Activamos el sensor
  digitalWrite(XSHUT_PIN, HIGH);
  delay(100);

  // Intentamos inicializar el sensor con la nueva direccion (0x30)
  // El .begin() con el argumento es lo que le asigna la nueva direccion
  if (!lox.begin(0x30)) {
    Serial.println(F("Error al inicializar el sensor. Verifique las conexiones."));
    while(1);
  }
  
  Serial.println(F("Sensor VL53L0X inicializado y con nueva direccion (0x30)."));
}

void loop() {
  // Objeto para almacenar la medicion
  VL53L0X_RangingMeasurementData_t measure;

  // Realizamos una medicion
  lox.rangingTest(&measure, false);

  // Verificamos si la medicion fue exitosa
  if (measure.RangeStatus != 4) {  // 4 significa 'Range Valid'
    Serial.print("Distancia: ");
    Serial.print(measure.RangeMilliMeter);
    Serial.println(" mm");
  } else {
    Serial.println(" Fuera de rango o error.");
  }

  // Esperamos un momento antes de la siguiente medicion
  delay(100);
}