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
int8_t car_forward_full_speed[4] = {0, -127, 0, 127};
int8_t car_forward[4] = {0, -100, 0, 100};
int8_t car_stop[4] = {0, 0, 0, 0};
int8_t car_turnleft[4] = {0, 23, 0, -100};
int8_t car_turnright[4] = {0, -100, 0, 23};
int8_t car_retreat[4] = {0, 100, 0, -100};

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
// Mapeo explícito de pines y direcciones I2C a la posición del sensor en el robot:
// XSHUT_PINS[0] -> Pin GPIO 0 -> Sensor Frontal
// XSHUT_PINS[1] -> Pin GPIO 1 -> Sensor Izquierdo
// XSHUT_PINS[2] -> Pin GPIO 3 -> Sensor Derecho
// XSHUT_PINS[3] -> Pin GPIO 4 -> Sensor Inclinado (para escalada)
const int XSHUT_PINS[] = {0, 1, 3, 4};
const int NUM_SENSORS = 4;
const uint8_t NEW_I2C_ADDRESSES[] = {0x30, 0x31, 0x32, 0x35};

Adafruit_VL53L0X lox[NUM_SENSORS];

// --- OBJETO DEL SENSOR MPU6050 ---
Adafruit_MPU6050 mpu;

// --- PARAMETROS DE NAVEGACION ---
const int DISTANCIA_OBSTACULO_MM = 300;
const float UMBRAL_ESCALABILIDAD = 0.9;
const float UMBRAL_GIRO_Z = 0.1;
const unsigned long RETREAT_DURATION_MS = 1000;
const unsigned long TURN_DURATION_MS = 1000;
const unsigned long FORWARD_DURATION_MS = 5000;
// --- PARAMETRO DE DETECCION DE ESTANCAMIENTO ---
const float UMBRAL_ACELERACION_X = 0.5; // g's
const unsigned long STUCK_TIMEOUT_MS = 1000;

// --- ESTADO DEL ROBOT ---
bool canMove = false;
unsigned long action_start_time = 0;
unsigned long stuck_timer = 0;
bool isLidarTurn = false;
enum RobotState { IDLE, MOVING_FORWARD, CLIMBING, RETREATING, TURNING };
RobotState currentState = IDLE;

void read_sensors(int *distancias);

void setup() {
  Serial.begin(115200);
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

  // Enviar READY al inicio para que el maestro comience el primer ciclo
  lidarSerial.println("READY");
  Serial.println("Enviado 'READY' al maestro. Esperando 'FINISHED'...");
}

void read_sensors(int *distancias) {
    VL53L0X_RangingMeasurementData_t measure;
    for (int i = 0; i < NUM_SENSORS; i++) {
        lox[i].setAddress(NEW_I2C_ADDRESSES[i]);
        lox[i].rangingTest(&measure, false);
        distancias[i] = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 9999;
    }
}

void loop() {
  // Lógica de comunicación con el maestro
  if (lidarSerial.available()) {
    String command = lidarSerial.readStringUntil('\n');
    command.trim();
    if (command.equals("FINISHED")) {
      canMove = true;
      Serial.println("Comando 'FINISHED' recibido. Robot puede moverse.");
      currentState = IDLE;
    }
  }

  // Máquina de estados para el movimiento del robot
  switch (currentState) {
    case IDLE:
      if (canMove) {
        Serial.println("\n--- Robot moviendose ---");
        int distancias[4];
        read_sensors(distancias);
        
        if (distancias[0] < DISTANCIA_OBSTACULO_MM) {
          Serial.println("Obstaculo frontal detectado. Analizando escalabilidad...");
          float ratio = (float)distancias[0] / distancias[3];
          if (ratio < UMBRAL_ESCALABILIDAD) {
            Serial.println("Obstaculo escalable. Iniciando escalada.");
            WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_forward_full_speed, 4);
            action_start_time = millis();
            stuck_timer = millis();
            currentState = CLIMBING;
          } else {
            Serial.println("Obstaculo no escalable. Tomando decision de giro...");
            if (distancias[1] > distancias[2]) {
              WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_turnleft, 4);
            } else {
              WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_turnright, 4);
            }
            action_start_time = millis();
            currentState = TURNING;
          }
        } else {
          Serial.println("Camino despejado. Moviendo adelante.");
          WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_forward, 4);
          action_start_time = millis();
          currentState = MOVING_FORWARD;
        }
        canMove = false; // Solo permite que la logica de movimiento se ejecute una vez
      }
      break;

    case CLIMBING:
      sensors_event_t g, a;
      mpu.getEvent(&a, &g, NULL);
      
      // Chequeo de nivelacion (escalada exitosa)
      if (g.gyro.z > -UMBRAL_GIRO_Z && g.gyro.z < UMBRAL_GIRO_Z) {
        Serial.println("Pendiente superada. Deteniendo motores.");
        WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_stop, 4);
        lidarSerial.println("READY");
        currentState = IDLE;
        break; // Sale del switch
      }
      
      // Chequeo de estancamiento (acelerometro)
      if (a.acceleration.x < UMBRAL_ACELERACION_X) {
        if (millis() - stuck_timer > STUCK_TIMEOUT_MS) {
          Serial.println("Atascado! Sin aceleracion detectada. Retrocediendo.");
          WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_retreat, 4);
          action_start_time = millis();
          currentState = RETREATING;
          break; // Sale del switch
        }
      } else {
        stuck_timer = millis(); // Reinicia el temporizador si detecta aceleracion
      }
      break;

    case RETREATING:
      if (millis() - action_start_time >= RETREAT_DURATION_MS) {
        Serial.println("Fin del retroceso. Tomando decision de giro...");
        int distancias[4];
        read_sensors(distancias);
        if (distancias[1] > distancias[2]) {
          WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_turnleft, 4);
        } else {
          WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_turnright, 4);
        }
        action_start_time = millis();
        currentState = TURNING;
      }
      break;
    
    case TURNING:
      if (millis() - action_start_time >= TURN_DURATION_MS) {
        Serial.println("Fin del giro. Deteniendo motores.");
        WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_stop, 4);
        lidarSerial.println("READY");
        currentState = IDLE;
      }
      break;
    
    case MOVING_FORWARD:
      if (millis() - action_start_time >= FORWARD_DURATION_MS) {
        Serial.println("Fin del movimiento adelante. Deteniendo motores.");
        WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_stop, 4);
        lidarSerial.println("READY");
        currentState = IDLE;
      }
      break;
  }
}