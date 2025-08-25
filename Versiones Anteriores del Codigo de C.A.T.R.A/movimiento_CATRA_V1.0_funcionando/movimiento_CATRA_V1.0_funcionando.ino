#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- I2C PINES Y CONFIGURACIONES GENERALES ---
// Pines I2C para el ESP32-C3 Super Mini
// SDA -> GPIO8, SCL -> GPIO9
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define I2C_ADDR 0x34

// --- CONFIGURACIONES DEL CONTROLADOR DE MOTORES I2C ---
#define MOTOR_TYPE_ADDR 0x14
#define MOTOR_ENCODER_POLARITY_ADDR 0x15
#define MOTOR_FIXED_SPEED_ADDR 0x33
#define MOTOR_TYPE_JGB37_520_12V_110RPM 3

uint8_t MotorType = MOTOR_TYPE_JGB37_520_12V_110RPM;
uint8_t MotorEncoderPolarity = 0;

uint8_t car_forward[4] = {0, 233, 0, 23};
uint8_t car_stop[4] = {0, 0, 0, 0};

bool WireWriteDataArray(uint8_t reg, uint8_t *val, unsigned int len) 
{
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  for(unsigned int i = 0; i < len; i++) {
    Wire.write(val[i]);
  }
  if ( Wire.endTransmission() != 0 ) {
    return false;
  }
  return true;
}

// --- CONFIGURACIONES DE LOS SENSORES VL53L0X ---
const int XSHUT_PINS[] = {0, 1, 3, 4};
const int NUM_SENSORS = 4;
const uint8_t NEW_I2C_ADDRESSES[] = {0x30, 0x31, 0x32, 0x35};

Adafruit_VL53L0X lox[NUM_SENSORS];

// --- OBJETO DEL SENSOR MPU6050 ---
Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("Iniciando...");

  // --- INICIALIZACION DEL BUS I2C Y PERIFERICOS ---
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(200);
  
  // Inicializacion del controlador de motores
  WireWriteDataArray(MOTOR_TYPE_ADDR, &MotorType, 1);
  delay(5);
  WireWriteDataArray(MOTOR_ENCODER_POLARITY_ADDR, &MotorEncoderPolarity, 1);
  Serial.println("Controlador de motores inicializado.");

  // Inicializacion del MPU6050
  if (!mpu.begin()) {
    Serial.println("Error al encontrar el MPU6050. Verifique las conexiones!");
    while (1);
  }
  Serial.println("MPU6050 inicializado correctamente!");
  
  // Inicializacion de los sensores VL53L0X
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(XSHUT_PINS[i], OUTPUT);
    digitalWrite(XSHUT_PINS[i], LOW);
  }
  delay(100);

  for (int i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(XSHUT_PINS[i], HIGH);
    delay(100);
    if (!lox[i].begin(NEW_I2C_ADDRESSES[i])) {
      Serial.print("Error al inicializar el sensor VL53L0X ");
      Serial.println(i);
      while(1);
    }
    Serial.print("Sensor VL53L0X ");
    Serial.print(i);
    Serial.print(" listo en 0x");
    Serial.println(NEW_I2C_ADDRESSES[i], HEX);
  }

  Serial.println("Todos los sensores estan configurados.");
  delay(1000);
}

void loop() {
  // --- LOGICA DE MOVIMIENTO INDEPENDIENTE ---
  Serial.println("ORDEN: Mover adelante.");
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_forward, 4);
  delay(5000);

  Serial.println("ORDEN: Detener robot.");
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_stop, 4);
  delay(5000);
  
  // --- LECTURA SECUENCIAL DE SENSORES VL53L0X ---
  VL53L0X_RangingMeasurementData_t measure;
  Serial.println("\n--- Lectura de Sensores de Distancia ---");
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    lox[i].setAddress(NEW_I2C_ADDRESSES[i]);
    lox[i].rangingTest(&measure, false);
    
    if (measure.RangeStatus != 4) {
      Serial.print("  Sensor ");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(measure.RangeMilliMeter);
      Serial.println(" mm");
    } else {
      Serial.print("  Sensor ");
      Serial.print(i);
      Serial.println(": Fuera de rango o error.");
    }
    delay(50); 
  }
  
  // --- LECTURA DEL SENSOR MPU6050 ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.println("\n--- Lectura de MPU6050 ---");
  Serial.print("  Aceleracion X: ");
  Serial.print(a.acceleration.x);
  Serial.print(" Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(" Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("  Giroscopio X: ");
  Serial.print(g.gyro.x);
  Serial.print(" Y: ");
  Serial.print(g.gyro.y);
  Serial.print(" Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("  Temperatura: ");
  Serial.print(temp.temperature);
  Serial.println(" C");

  Serial.println("----------------------------------------\n");
  delay(100);
}