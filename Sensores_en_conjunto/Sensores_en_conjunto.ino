#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Definiciones de pines GPIO para los pines XSHUT
#define XSHUT_0_0 GPIO_NUM_4 // Sensor 0 en Bus 0
#define XSHUT_0_1 GPIO_NUM_5 // Sensor 1 en Bus 0
#define XSHUT_1_0 GPIO_NUM_23 // Sensor 0 en Bus 1
#define XSHUT_1_1 GPIO_NUM_33 // Sensor 1 en Bus 1

// Direcciones I2C asignadas (deben ser únicas y diferentes de 0x29)
#define ADDR_0_0 0x30
#define ADDR_0_1 0x31
#define ADDR_1_0 0x32
#define ADDR_1_1 0x35

// Pines I2C para Bus 0 (Wire)
#define SDA0_PIN GPIO_NUM_21
#define SCL0_PIN GPIO_NUM_22

// Pines I2C para Bus 1 (Wire1)
#define SDA1_PIN GPIO_NUM_18
#define SCL1_PIN GPIO_NUM_19

// Instancias de TwoWire para los dos buses I2C
TwoWire I2C_Bus0 = TwoWire(0); // Para Wire
TwoWire I2C_Bus1 = TwoWire(1); // Para Wire1

// Objetos para los sensores VL53L0X
Adafruit_VL53L0X lox_sensor_0_0 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_sensor_0_1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_sensor_1_0 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_sensor_1_1 = Adafruit_VL53L0X();

// Estructura para almacenar datos de medición
VL53L0X_RangingMeasurementData_t measure_0_0;
VL53L0X_RangingMeasurementData_t measure_0_1;
VL53L0X_RangingMeasurementData_t measure_1_0;
VL53L0X_RangingMeasurementData_t measure_1_1;

void setup() {
  Serial.begin(115200);
  while (!Serial); // Esperar a que el puerto serial se abra

  Serial.println(F("Inicializando sensores VL53L0X..."));

  // Configurar pines XSHUT como OUTPUT y ponerlos LOW para reiniciar todos los sensores
  pinMode(XSHUT_0_0, OUTPUT);
  pinMode(XSHUT_0_1, OUTPUT);
  pinMode(XSHUT_1_0, OUTPUT);
  pinMode(XSHUT_1_1, OUTPUT);

  digitalWrite(XSHUT_0_0, LOW);
  digitalWrite(XSHUT_0_1, LOW);
  digitalWrite(XSHUT_1_0, LOW);
  digitalWrite(XSHUT_1_1, LOW);
  delay(100); // Dar tiempo a los sensores para apagarse completamente

  // Inicializar los buses I2C
  I2C_Bus0.begin(SDA0_PIN, SCL0_PIN, 400000); // Bus 0 en 400kHz (Fast Mode)
  I2C_Bus1.begin(SDA1_PIN, SCL1_PIN, 400000); // Bus 1 en 400kHz (Fast Mode)

  // --- Proceso de asignación de direcciones para el Bus 0 ---

  // Activar Sensor_0_0 y asignar dirección
  Serial.println(F("Configurando Sensor_0_0 (Bus 0)..."));
  digitalWrite(XSHUT_0_0, HIGH);
  delay(10); // Permitir que el sensor se encienda
  if (!lox_sensor_0_0.begin(VL53L0X_I2C_ADDR, false, &I2C_Bus0)) {
    Serial.println(F("Fallo al iniciar Sensor_0_0."));
    while (1);
  }
  lox_sensor_0_0.setAddress(ADDR_0_0);
  Serial.print(F("Sensor_0_0 inicializado con dirección: 0x"));
  Serial.println(ADDR_0_0, HEX);

  // Activar Sensor_0_1 y asignar dirección
  Serial.println(F("Configurando Sensor_0_1 (Bus 0)..."));
  digitalWrite(XSHUT_0_1, HIGH);
  delay(10); // Permitir que el sensor se encienda
  if (!lox_sensor_0_1.begin(VL53L0X_I2C_ADDR, false, &I2C_Bus0)) {
    Serial.println(F("Fallo al iniciar Sensor_0_1."));
    while (1);
  }
  lox_sensor_0_1.setAddress(ADDR_0_1);
  Serial.print(F("Sensor_0_1 inicializado con dirección: 0x"));
  Serial.println(ADDR_0_1, HEX);

  // --- Proceso de asignación de direcciones para el Bus 1 ---

  // Activar Sensor_1_0 y asignar dirección
  Serial.println(F("Configurando Sensor_1_0 (Bus 1)..."));
  digitalWrite(XSHUT_1_0, HIGH);
  delay(10); // Permitir que el sensor se encienda
  if (!lox_sensor_1_0.begin(VL53L0X_I2C_ADDR, false, &I2C_Bus1)) {
    Serial.println(F("Fallo al iniciar Sensor_1_0."));
    while (1);
  }
  lox_sensor_1_0.setAddress(ADDR_1_0);
  Serial.print(F("Sensor_1_0 inicializado con dirección: 0x"));
  Serial.println(ADDR_1_0, HEX);

  // Activar Sensor_1_1 y asignar dirección
  Serial.println(F("Configurando Sensor_1_1 (Bus 1)..."));
  digitalWrite(XSHUT_1_1, HIGH);
  delay(10); // Permitir que el sensor se encienda
  if (!lox_sensor_1_1.begin(VL53L0X_I2C_ADDR, false, &I2C_Bus1)) {
    Serial.println(F("Fallo al iniciar Sensor_1_1."));
    while (1);
  }
  lox_sensor_1_1.setAddress(ADDR_1_1);
  Serial.print(F("Sensor_1_1 inicializado con dirección: 0x"));
  Serial.println(ADDR_1_1, HEX);

  Serial.println(F("\nTodos los sensores VL53L0X inicializados exitosamente."));

  // Iniciar el modo de medición continua para todos los sensores
  lox_sensor_0_0.startRangeContinuous();
  lox_sensor_0_1.startRangeContinuous();
  lox_sensor_1_0.startRangeContinuous();
  lox_sensor_1_1.startRangeContinuous();
  Serial.println(F("Iniciando mediciones continuas..."));
}

void loop() {
  Serial.print(F("S0_0: "));
  if (lox_sensor_0_0.isRangeComplete()) {
    // Usar rangingTest para llenar la estructura measure_0_0
    lox_sensor_0_0.rangingTest(&measure_0_0, false);
    if (measure_0_0.RangeStatus != 4) { // 4 = fuera de rango
      Serial.print(measure_0_0.RangeMilliMeter);
    } else {
      Serial.print(F("Fuera de rango"));
    }
  } else {
    Serial.print(F("Esperando..."));
  }
  Serial.print(F(" | S0_1: "));
  if (lox_sensor_0_1.isRangeComplete()) {
    // Usar rangingTest para llenar la estructura measure_0_1
    lox_sensor_0_1.rangingTest(&measure_0_1, false);
    if (measure_0_1.RangeStatus != 4) {
      Serial.print(measure_0_1.RangeMilliMeter);
    } else {
      Serial.print(F("Fuera de rango"));
    }
  } else {
    Serial.print(F("Esperando..."));
  }
  Serial.print(F(" | S1_0: "));
  if (lox_sensor_1_0.isRangeComplete()) {
    // Usar rangingTest para llenar la estructura measure_1_0
    lox_sensor_1_0.rangingTest(&measure_1_0, false);
    if (measure_1_0.RangeStatus != 4) {
      Serial.print(measure_1_0.RangeMilliMeter);
    } else {
      Serial.print(F("Fuera de rango"));
    }
  } else {
    Serial.print(F("Esperando..."));
  }
  Serial.print(F(" | S1_1: "));
  if (lox_sensor_1_1.isRangeComplete()) {
    // Usar rangingTest para llenar la estructura measure_1_1
    lox_sensor_1_1.rangingTest(&measure_1_1, false);
    if (measure_1_1.RangeStatus != 4) {
      Serial.print(measure_1_1.RangeMilliMeter);
    } else {
      Serial.print(F("Fuera de rango"));
    }
  } else {
    Serial.print(F("Esperando..."));
  }
  Serial.println(F(" mm"));

  delay(50); // Pequeño retardo para estabilidad y evitar sobrecarga del Serial
}
