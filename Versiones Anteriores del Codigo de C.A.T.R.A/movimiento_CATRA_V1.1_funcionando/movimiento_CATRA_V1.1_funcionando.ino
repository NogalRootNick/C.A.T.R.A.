#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- UART PINES PARA COMUNICACION CON ESP32 PRINCIPAL (LIDAR) ---
#define LIDAR_UART_RX_PIN 7
#define LIDAR_UART_TX_PIN 10
HardwareSerial &lidarSerial = Serial1;

// --- I2C PINES Y CONFIGURACIONES GENERALES ---
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

// Valores ajustados a int8_t para evitar errores de conversion
int8_t car_forward_full_speed[4] = {0, 127, 0, 127};
int8_t car_forward[4] = {0, -100, 0, 100};
int8_t car_stop[4] = {0, 0, 0, 0};
int8_t car_turnleft[4] = {0, -23, 0, 100};
int8_t car_turnright[4] = {0, 100, 0, -23};
int8_t car_retreat[4] = {0, -100, 0, 100};

// Funcion modificada para aceptar int8_t y evitar errores de conversion
bool WireWriteDataArray(uint8_t reg, int8_t *val, unsigned int len)
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

// --- ESTADO DEL ROBOT ---
bool canMove = false;

// Prototipo de la función de movimiento
void handleMovement();

void setup() {
  Serial.begin(115200);
  // ELIMINADA la linea 'while (!Serial) { delay(10); }' para que el robot funcione sin PC
  Serial.println("Iniciando...");

  lidarSerial.begin(115200, SERIAL_8N1, LIDAR_UART_RX_PIN, LIDAR_UART_TX_PIN);
  Serial.println("UART para comunicacion con el LIDAR iniciado.");
  
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(200);
  
  WireWriteDataArray(MOTOR_TYPE_ADDR, (int8_t*)&MotorType, 1);
  delay(5);
  WireWriteDataArray(MOTOR_ENCODER_POLARITY_ADDR, (int8_t*)&MotorEncoderPolarity, 1);
  Serial.println("Controlador de motores inicializado.");

  if (!mpu.begin()) {
    Serial.println("Error al encontrar el MPU6050. Verifique las conexiones!");
    while (1);
  }
  Serial.println("MPU6050 inicializado correctamente!");
  
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

  lidarSerial.println("READY");
  Serial.println("Enviado 'READY' al maestro. Esperando 'FINISHED'...");
}

void loop() {
  if (lidarSerial.available()) {
    String command = lidarSerial.readStringUntil('\n');
    command.trim();
    if (command.equals("FINISHED")) {
      canMove = true;
      Serial.println("Comando 'FINISHED' recibido. Robot puede moverse.");
    }
  }

  if (canMove) {
    handleMovement();
    canMove = false;
    Serial.println("Movimiento completado. Robot detenido.");
    lidarSerial.println("READY");
    Serial.println("Enviado 'READY' al maestro. Esperando 'FINISHED'...");
  }
}

void handleMovement() {
  Serial.println("\n--- Robot moviendose ---");
  VL53L0X_RangingMeasurementData_t measure;
  int distancias[4];
  
  Serial.println("Camino despejado. Moviendo adelante por 3 segundos.");
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_forward, 4);
  delay(3000);
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_stop, 4);
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    lox[i].setAddress(NEW_I2C_ADDRESSES[i]);
    lox[i].rangingTest(&measure, false);
    distancias[i] = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 9999;
    Serial.print("Sensor VL53L0X ");
    Serial.print(i);
    Serial.print(": Distancia = ");
    Serial.print(distancias[i]);
    Serial.println(" mm");
  }
  Serial.println("Lecturas de sensores completadas.");
  
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_stop, 4);
}