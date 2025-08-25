#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include "esp_sleep.h"
#include <SPI.h>
#include <RF24.h>

// --- Definiciones para el Sistema LIDAR ---
#define LIDAR_I2C_SDA_PIN 18
#define LIDAR_I2C_SCL_PIN 19
#define SERVO_PIN 25

// --- Definiciones para la Comunicacion con el ESP32 Esclavo ---
#define SLAVE_UART_RX_PIN 16
#define SLAVE_UART_TX_PIN 17
HardwareSerial slaveSerial(2);

// --- DEFINICIONES PARA NRF24L01 (HSPI) ---
#define NRF_CE_PIN 32
#define NRF_CSN_PIN 15
#define HSPI_SCLK_PIN 14
#define HSPI_MISO_PIN 12
#define HSPI_MOSI_PIN 13

// --- Objetos Globales ---
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Servo LIDARservo;
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const byte address[6] = "00001";

// Estructura para los datos a transmitir
struct LidarDataPacket {
    float coord_x;
    float coord_y;
    uint8_t range_status;
    uint16_t range_millimeter;
    float temperature_c;
    float humidity_rh;
    float pressure_hpa;
};

// Variables para comunicacion entre hilos
LidarDataPacket currentData;
TaskHandle_t TaskServo;
bool isLidarTurn = false; // Se inicia en 'false'
// --- Duracion del ciclo del LIDAR cambiada a 5 segundos ---
const unsigned long LIDAR_OPERATING_DURATION_MS = 5000; 
unsigned long lidar_start_time = 0;

// PIN para la senal de hibernacion
#define HIBERNATION_SIGNAL_PIN 33
const unsigned long HIBERNATION_DELAY_MS = 5000;
unsigned long highSignalStartTime = 0;

void goIntoDeepSleep() {
    Serial.println("Senal de hibernacion detectada. Entrando en Deep Sleep...");
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, LOW);
    esp_deep_sleep_start();
}

void TaskServocode( void * pvParameters ){
  Serial.println("Tarea del Servo en ejecucion en el Nucleo 1.");
  LIDARservo.attach(SERVO_PIN);
  LIDARservo.write(0); // Detiene el servo al inicio
  for(;;){
    if (isLidarTurn) {
        LIDARservo.write(90); // Gira el servo
    } else {
        LIDARservo.write(0); // Detiene el servo
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    Serial.println("Inicializando sistema completo...");

    pinMode(HIBERNATION_SIGNAL_PIN, INPUT);
    Wire.begin(LIDAR_I2C_SDA_PIN, LIDAR_I2C_SCL_PIN);
    SPI.begin(HSPI_SCLK_PIN, HSPI_MISO_PIN, HSPI_MOSI_PIN);

    Serial.println("Inicializando sensores...");
    if (!lox.begin(0x29, false, &Wire) || !aht.begin() || !bmp.begin()) {
        Serial.println(F("Error al iniciar los sensores. Comprueba el cableado."));
        while (1);
    }
    Serial.println("Sensores inicializados correctamente.");
    bmp.setSampling(Adafruit_BMP280::MODE_FORCED, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_500);

    Serial.println("Inicializando NRF24L01...");
    if (!radio.begin()) {
        Serial.println(F("Error al iniciar el NRF24L01!"));
    } else {
        radio.setChannel(115);
        radio.setPALevel(RF24_PA_MAX);
        radio.setDataRate(RF24_250KBPS);
        radio.openWritingPipe(address);
        radio.stopListening();
        Serial.println("NRF24L01 inicializado correctamente como transmisor.");
    }
    slaveSerial.begin(115200, SERIAL_8N1, SLAVE_UART_RX_PIN, SLAVE_UART_TX_PIN);
    Serial.println("Comunicacion serial con el ESP32 esclavo lista.");
    
    xTaskCreatePinnedToCore(
                    TaskServocode,
                    "TaskServo",
                    4096,
                    NULL,
                    1,
                    &TaskServo,
                    1);
    
    // El maestro espera la primera señal READY para iniciar el primer ciclo
    Serial.println("Esperando la primera señal 'READY' del esclavo...");
}

void runLidarSystem() {
    Serial.println("--- Turno del LIDAR y Sensores Ambientales ---");
    
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);

    float distance_mm = 0.0;
    bool valid_detection = false;
    
    if (measure.RangeStatus != 4 && measure.RangeMilliMeter >= 4.5 && measure.RangeMilliMeter <= 1500) {
        distance_mm = (float)measure.RangeMilliMeter;
        valid_detection = true;
    }
    
    float angle_radians = 0.0;
    currentData.coord_x = valid_detection ? distance_mm * cos(angle_radians) : 0.0f;
    currentData.coord_y = valid_detection ? distance_mm * sin(angle_radians) : 0.0f;
    currentData.range_status = measure.RangeStatus;
    currentData.range_millimeter = measure.RangeMilliMeter;

    sensors_event_t humidity, temp_aht;
    aht.getEvent(&humidity, &temp_aht);
    bmp.takeForcedMeasurement();
    float temp_bmp = bmp.readTemperature();
    currentData.temperature_c = (temp_aht.temperature + temp_bmp) / 2.0;
    currentData.humidity_rh = humidity.relative_humidity;
    currentData.pressure_hpa = bmp.readPressure() / 100.0F;

    Serial.print("LIDAR: X="); Serial.print(currentData.coord_x, 2); Serial.print(", Y="); Serial.println(currentData.coord_y, 2);
    Serial.print("Distancia (Raw): "); Serial.print(currentData.range_millimeter); Serial.println(" mm");
    Serial.println("--- Lecturas de Sensores Ambientales ---");
    Serial.print("Temperatura Promedio: "); Serial.print(currentData.temperature_c); Serial.println(" C");
    Serial.print("Humedad AHT20: "); Serial.print(currentData.humidity_rh); Serial.println(" %");
    Serial.print("Presion BMP280: "); Serial.print(currentData.pressure_hpa); Serial.println(" hPa");

    bool report = radio.write(&currentData, sizeof(currentData));
    if (report) {
    } else {
        Serial.println(F("NRF: Transmision de datos fallida."));
    }
    
    // La logica para el fin del ciclo se movio al loop()
}

void loop() {
    if (digitalRead(HIBERNATION_SIGNAL_PIN) == HIGH) {
        if (highSignalStartTime == 0) {
            highSignalStartTime = millis();
        } else if (millis() - highSignalStartTime >= HIBERNATION_DELAY_MS) {
            goIntoDeepSleep();
        }
    } else {
        highSignalStartTime = 0;
    }

    if (isLidarTurn) {
        runLidarSystem();
        if (millis() - lidar_start_time >= LIDAR_OPERATING_DURATION_MS) {
            isLidarTurn = false;
            slaveSerial.println("FINISHED");
            Serial.println("LIDAR: Mensaje 'FINISHED' enviado al ESP32 esclavo.");
        }
    } else {
        if (slaveSerial.available()) {
            String command = slaveSerial.readStringUntil('\n');
            command.trim();
            if (command.equals("READY")) {
                isLidarTurn = true;
                lidar_start_time = millis(); 
                Serial.println("LIDAR: Recibida senal 'READY' del esclavo. Reiniciando ciclo.");
            }
        }
    }
}