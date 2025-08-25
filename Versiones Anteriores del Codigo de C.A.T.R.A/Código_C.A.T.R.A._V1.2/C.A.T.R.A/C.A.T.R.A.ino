// librerías necesarias
#include <ESP32Servo.h> // Librería específica para servos en ESP32
#include <Wire.h> // Para comunicación I2C (usada por ambos sistemas)
#include <Adafruit_VL53L0X.h> // Para el sensor de distancia VL53L0X
#include <Adafruit_MPU6050.h> // Para el acelerómetro/giroscopio MPU6050
#include <Adafruit_Sensor.h> // Librería base para sensores Adafruit

// --- LIBRERÍAS PARA NRF24L01 ---
#include <SPI.h>
#include <RF24.h>

// --- NUEVAS LIBRERÍAS PARA SENSORES AHT20 Y BMP280 ---
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>

// --- Definiciones para el Sistema LIDAR ---
#define LIDAR_I2C_SDA_PIN 18
#define LIDAR_I2C_SCL_PIN 19
#define SERVO_PIN 25

// --- Definiciones para el Sistema de Movimiento del Robot ---
#define MOTOR_I2C_SDA_PIN 21
#define MOTOR_I2C_SCL_PIN 22
#define MOTOR_CONTROLLER_I2C_ADDR 0x34 // Dirección I2C del controlador de motores

// Direcciones de registros para el controlador de motores
#define ADC_BAT_ADDR 0x00
#define MOTOR_TYPE_ADDR 0x14
#define MOTOR_ENCODER_POLARITY_ADDR 0x15
#define MOTOR_FIXED_PWM_ADDR 0x1F
#define MOTOR_FIXED_SPEED_ADDR 0x33
#define MOTOR_ENCODER_TOTAL_ADDR 0x3C

#define MOTOR_TYPE_JGB37_520_12V_110RPM 3

// --- DEFINICIONES PARA NRF24L01 (HSPI) ---
#define NRF_CE_PIN 32// Pin CE del NRF24L01 conectado al GPIO32
#define NRF_CSN_PIN 15 // ¡¡¡IMPORTANTE!!! Verifica y ajusta este pin. GPIO15 es HSPI_CS0.

// Pines estándar para el bus HSPI en ESP32
#define HSPI_SCLK_PIN 14
#define HSPI_MISO_PIN 12
#define HSPI_MOSI_PIN 13

// --- Objetos Globales ---
// Sistema LIDAR
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_MPU6050 mpu;
Servo myservo;

// --- NUEVOS OBJETOS PARA SENSORES ---
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;

// Sistema de Movimiento del Robot


// NRF24L01
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const byte address[6] = "00001"; // Dirección de la tubería de transmisión, debe coincidir con el receptor

// Estructura para los datos a transmitir (actualizada)
struct LidarDataPacket {
    float coord_x;
    float coord_y;
    uint8_t range_status;
    uint16_t range_millimeter;
    float temperature;
    float humidity;
    float pressure;
};

// --- Variables Globales ---
// LIDAR
float current_gyro_angle_y = 0.0;
unsigned long last_lidar_update_time = 0;

// Robot (controlador de motores)
uint8_t MotorType = MOTOR_TYPE_JGB37_520_12V_110RPM;
uint8_t MotorEncoderPolarity = 0;
uint8_t car_forward[4] = {0, 233, 0, 23};
uint8_t car_stop[4] = {0, 0, 0, 0};

// Control de turnos y estado del robot
bool isLidarTurn = true;
unsigned long robot_action_timer = 0;
int robot_action_state = 0;

const unsigned long ROBOT_FORWARD_DURATION_MS = 2000;
const unsigned long ROBOT_STOPPED_DURATION_MS = 1000;
const unsigned long LIDAR_OPERATING_DURATION_MS = 1500;
unsigned long lidar_start_time = 0;

/**
 * @brief array de datos de un registro específico en el dispositivo I2C del controlador de motores.
 */
bool MotorController_WriteDataArray(uint8_t reg, uint8_t *val, unsigned int len) {
    Wire1.beginTransmission(MOTOR_CONTROLLER_I2C_ADDR);
    Wire1.write(reg);
  for (unsigned int i = 0; i < len; i++) {
        Wire1.write(val[i]);
     }
    if (Wire1.endTransmission() != 0) {
       Serial.println("Error en la transmisión I2C al controlador de motores.");
         return false;
     }
     return true;
}

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    Serial.println("Inicializando sistema integrado (LIDAR, Robot y NRF24L01)...");

// --- Inicialización I2C para LIDAR y Sensores (AHT20/BMP280) ---
    Wire.begin(LIDAR_I2C_SDA_PIN, LIDAR_I2C_SCL_PIN);
    Serial.println("Bus I2C del LIDAR (Wire) inicializado en SDA=" + String(LIDAR_I2C_SDA_PIN) + ", SCL=" + String(LIDAR_I2C_SCL_PIN));

// --- Inicialización I2C para Controlador de Motores ---
    Wire1.begin(MOTOR_I2C_SDA_PIN, MOTOR_I2C_SCL_PIN);
    Serial.println("Bus I2C del Controlador de Motores (Wire1) inicializado en SDA=" + String(MOTOR_I2C_SDA_PIN) + ", SCL=" + String(MOTOR_I2C_SCL_PIN));
    delay(200);

// --- Inicialización Sensores LIDAR y MPU6050 ---
    Serial.println("Inicializando VL53L0X...");
    if (!lox.begin(0x29, false, &Wire)) {
       Serial.println(F("¡Error al iniciar el VL53L0X! Comprueba el cableado en el bus I2C0."));
         while (1);
    }
    Serial.println("VL53L0X inicializado correctamente.");
    lox.setMeasurementTimingBudgetMicroSeconds(20000);
    lox.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    lox.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);

    Serial.println("Inicializando MPU6050...");
    if (!mpu.begin(0x68, &Wire)) {
        Serial.println("¡Error al iniciar el MPU6050! Comprueba el cableado en el bus I2C0.");
        while (1);
    }
    Serial.println("MPU6050 inicializado correctamente.");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // --- NUEVA INICIALIZACIÓN SENSORES AHT20 y BMP280 ---
   Serial.println("Inicializando AHT20...");
    if (!aht.begin(&Wire)) {
         Serial.println("¡Error al iniciar el AHT20! Comprueba el cableado.");
        while (1);
    }

    Serial.println("AHT20 inicializado correctamente.");

    Serial.println("Inicializando BMP280...");
    if (!bmp.begin(0x77)) {
         Serial.println("¡Error al iniciar el BMP280! Comprueba el cableado.");
        while (1);
     }
    bmp.setSampling(Adafruit_BMP280::MODE_FORCED, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_500);
    Serial.println("BMP280 inicializado correctamente.");

    myservo.attach(SERVO_PIN);
    myservo.write(0);
    Serial.println("Servo del LIDAR inicializado y detenido (0 grados).");

// --- Inicialización Controlador de Motores ---
    Serial.println("Inicializando Controlador de Motores...");
    if (!MotorController_WriteDataArray(MOTOR_TYPE_ADDR, &MotorType, 1)) {
        Serial.println("Error configurando tipo de motor.");
     }
    delay(5);
    if (!MotorController_WriteDataArray(MOTOR_ENCODER_POLARITY_ADDR, &MotorEncoderPolarity, 1)) {
        Serial.println("Error configurando polaridad del encoder.");
    }
    Serial.println("Controlador de Motores configurado.");

// --- Inicialización NRF24L01 ---
    Serial.println("Inicializando NRF24L01 en HSPI...");
    Serial.println("NRF CE: " + String(NRF_CE_PIN) + ", CSN: " + String(NRF_CSN_PIN));
    Serial.println("HSPI SCLK: " + String(HSPI_SCLK_PIN) + ", MISO: " + String(HSPI_MISO_PIN) + ", MOSI: " + String(HSPI_MOSI_PIN));
    SPI.begin(HSPI_SCLK_PIN, HSPI_MISO_PIN, HSPI_MOSI_PIN);
    if (!radio.begin()) {
        Serial.println(F("¡Error al iniciar el NRF24L01! Comprueba el cableado y los pines."));
    } else {
        radio.setChannel(115);
        radio.setPALevel(RF24_PA_MAX);
        radio.setDataRate(RF24_250KBPS);
        radio.openWritingPipe(address);
        radio.stopListening();
        Serial.println("NRF24L01 inicializado correctamente como transmisor.");
    }

    last_lidar_update_time = millis();
    isLidarTurn = true;
    robot_action_state = 0;
    Serial.println("Sistema listo. LIDAR, sensores ambientales y Robot operarán en turnos. Datos se transmitirán por NRF24L01.");
}

void runLidarSystem() {
    Serial.println("--- Turno del LIDAR y Sensores Ambientales ---");
    unsigned long current_time = millis();
    float dt = (current_time - last_lidar_update_time) / 1000.0;
    last_lidar_update_time = current_time;

    if (lidar_start_time == 0) {
        myservo.write(90);
        Serial.println("LIDAR: Servo iniciando giro (90 grados).");
        lidar_start_time = current_time;
    }

// --- Lectura de datos LIDAR ---
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);

    float distance_mm = 0.0;
    bool valid_detection = false;
    float coord_x = 0.0f;
    float coord_y = 0.0f;

    if (measure.RangeStatus != 4 && measure.RangeMilliMeter >= 4.5 && measure.RangeMilliMeter <= 1500) {
        distance_mm = (float)measure.RangeMilliMeter;
        valid_detection = true;
    }

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    current_gyro_angle_y += (g.gyro.y * (180.0 / PI)) * dt;

    if (current_gyro_angle_y >= 360.0) current_gyro_angle_y -= 360.0;
    else if (current_gyro_angle_y < 0.0) current_gyro_angle_y += 360.0;

    if (valid_detection) {
        float angle_radians = current_gyro_angle_y * (PI / 180.0);
        coord_x = distance_mm * cos(angle_radians);
        coord_y = distance_mm * sin(angle_radians);
    }
    Serial.print("LIDAR: X="); Serial.print(coord_x, 2); Serial.print(", Y="); Serial.println(coord_y, 2);

// --- NUEVA LECTURA DE SENSORES AHT20 y BMP280 ---
    sensors_event_t humidity_event, temp_event_aht;
    aht.getEvent(&humidity_event, &temp_event_aht);

    float pressure = bmp.readPressure() / 100.0F;
    float temp_bmp = bmp.readTemperature();

    float average_temp = (temp_event_aht.temperature + temp_bmp) / 2.0;

    Serial.println("--------------------");
    Serial.print("Temperatura promedio: ");
    Serial.print(average_temp, 2); // Imprimir con 2 decimales
    Serial.println(" °C");

    Serial.print("Humedad: ");
    Serial.print(humidity_event.relative_humidity, 2);
    Serial.println(" %");

    Serial.print("Presion: ");
    Serial.print(pressure, 2);
    Serial.println(" hPa");

// --- Transmisión NRF24L01 ---
    LidarDataPacket data_to_send;
    data_to_send.coord_x = coord_x;
    data_to_send.coord_y = coord_y;
    data_to_send.range_status = measure.RangeStatus;
    data_to_send.range_millimeter = measure.RangeMilliMeter;
    data_to_send.temperature = temp_event_aht.temperature;
    data_to_send.humidity = humidity_event.relative_humidity;
    data_to_send.pressure = pressure;

    bool report = radio.write(&data_to_send, sizeof(data_to_send));
    if (report) {
    } else {
        Serial.println(F("NRF: Transmisión de datos fallida."));
    }
// --- Fin Transmisión NRF24L01 ---

    if (current_time - lidar_start_time >= LIDAR_OPERATING_DURATION_MS) {
        myservo.write(0);
        Serial.println("LIDAR: Servo detenido (0 grados). Turno finalizado.");
        lidar_start_time = 0;
        isLidarTurn = false;
    }
}

void runRobotSystem() {
    unsigned long current_time = millis();

    switch (robot_action_state) {
      case 0:
          if (!isLidarTurn) {
                Serial.println("--- Turno del Robot ---");
                Serial.println("ROBOT: Iniciando movimiento hacia adelante.");
                MotorController_WriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_forward, 4);
                robot_action_timer = current_time;
                robot_action_state = 1;
          }
           break;

        case 1:
          if (current_time - robot_action_timer >= ROBOT_FORWARD_DURATION_MS) {
              Serial.println("ROBOT: Deteniendo motores.");
              MotorController_WriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_stop, 4);
              robot_action_timer = current_time;
              robot_action_state = 2;
              }
            break;

        case 2:
            if (current_time - robot_action_timer >= ROBOT_STOPPED_DURATION_MS) {
                Serial.println("ROBOT: Pausa finalizada. Cediendo control a LIDAR.");
                robot_action_state = 0;
                isLidarTurn = true;
                }
            break;
    }
}

void loop() {
    if (isLidarTurn) {
        runLidarSystem();
    } else {
        runRobotSystem();
      }
}