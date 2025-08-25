// Librerias necesarias
#include <ESP32Servo.h>       // Libreria especifica para servos en ESP32
#include <Wire.h>             // Para comunicacion I2C
#include <Adafruit_VL53L0X.h> // Para el sensor de distancia VL53L0X
#include <Adafruit_MPU6050.h> // Para el acelerometro/giroscopio MPU6050
#include <Adafruit_Sensor.h>  // Libreria base para sensores Adafruit
#include <SPI.h>              // Para la comunicacion SPI (usada por el NRF24L01)
#include <RF24.h>             // Para el modulo de radio NRF24L01
#include <Adafruit_AHTX0.h>   // Para el sensor de temperatura y humedad AHT20
#include <Adafruit_BMP280.h>    // Para el sensor de presion y temperatura BMP280

// --- Definiciones de pines para el bus I2C del LIDAR y sensores ---
#define LIDAR_I2C_SDA_PIN 18
#define LIDAR_I2C_SCL_PIN 19
#define SERVO_PIN 25

// --- Definiciones de pines para la comunicacion con el NRF24L01 (HSPI) ---
#define NRF_CE_PIN 32   // Pin CE del NRF24L01
#define NRF_CSN_PIN 15  // Pin CSN del NRF24L01

// Pines estandar para el bus HSPI en ESP32
#define HSPI_SCLK_PIN 14
#define HSPI_MISO_PIN 12
#define HSPI_MOSI_PIN 13

// --- Objetos Globales ---
// Sistema LIDAR
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_MPU6050 mpu;
Servo myservo;

// Sensores ambientales
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;

// Modulo NRF24L01
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const byte address[6] = "00001"; // Direccion de la tuberia de transmision, debe coincidir con el receptor

// Estructura para los datos a transmitir
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

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    Serial.println("Inicializando sistema LIDAR, sensores ambientales y NRF24L01...");

// --- Inicializacion I2C para LIDAR y Sensores (AHT20/BMP280) ---
    Wire.begin(LIDAR_I2C_SDA_PIN, LIDAR_I2C_SCL_PIN);
    Serial.println("Bus I2C principal (Wire) inicializado en SDA=" + String(LIDAR_I2C_SDA_PIN) + ", SCL=" + String(LIDAR_I2C_SCL_PIN));
    // Se ha aumentado el retardo para dar mas tiempo a los dispositivos I2C
    delay(500);

// --- Inicializacion de los Sensores ---
    Serial.println("Inicializando VL53L0X...");
    if (!lox.begin(0x29, false, &Wire)) {
        Serial.println(F("¡Error al iniciar el VL53L0X! Comprueba el cableado."));
        while (1);
    }
    Serial.println("VL53L0X inicializado correctamente.");
    lox.setMeasurementTimingBudgetMicroSeconds(20000);
    lox.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    lox.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);

    Serial.println("Inicializando MPU6050...");
    if (!mpu.begin(0x68, &Wire)) {
        Serial.println("¡Error al iniciar el MPU6050! Comprueba el cableado.");
        while (1);
    }
    Serial.println("MPU6050 inicializado correctamente.");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

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
    myservo.write(90); // Mover el servo a 90 grados una vez al inicio
    Serial.println("Servo del LIDAR inicializado y moviendose (90 grados).");

// --- Inicializacion NRF24L01 ---
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
    Serial.println("Sistema listo. Las lecturas se enviaran continuamente a traves del NRF24L01.");
}

void runLidarAndSensorSystem() {
    unsigned long current_time = millis();
    float dt = (current_time - last_lidar_update_time) / 1000.0;
    last_lidar_update_time = current_time;

    // --- Lectura de datos LIDAR ---
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    float distance_mm = 0.0;
    bool valid_detection = false;
    float coord_x = 0.0f;
    float coord_y = 0.0f;

    // Se imprime la informacion bruta del sensor para propositos de depuracion.
    Serial.print("LIDAR RAW: Estado=");
    Serial.print(measure.RangeStatus);
    Serial.print(", Distancia=");
    Serial.print(measure.RangeMilliMeter);
    Serial.println(" mm");


    if (measure.RangeStatus != 4 && measure.RangeMilliMeter >= 4.5 && measure.RangeMilliMeter <= 200) {
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
        // Las coordenadas se imprimen solo si la deteccion es valida.
        Serial.print("LIDAR: X=");
        Serial.print(coord_x, 2);
        Serial.print(", Y=");
        Serial.println(coord_y, 2);
    } else {
        // Se informa que la deteccion no es valida.
        Serial.println("LIDAR: Deteccion invalida, no se calculan coordenadas.");
    }

    // --- Lectura de sensores ambientales AHT20 y BMP280 ---
    sensors_event_t humidity_event, temp_event_aht;
    aht.getEvent(&humidity_event, &temp_event_aht);

    float pressure = bmp.readPressure() / 100.0F;
    float temp_bmp = bmp.readTemperature();
    float average_temp = (temp_event_aht.temperature + temp_bmp) / 2.0;

    Serial.println("--------------------");
    Serial.print("Temperatura promedio: ");
    Serial.print(average_temp, 2);
    Serial.println(" °C");

    Serial.print("Humedad: ");
    Serial.print(humidity_event.relative_humidity, 2);
    Serial.println(" %");

    Serial.print("Presion: ");
    Serial.print(pressure, 2);
    Serial.println(" hPa");

    // --- Transmision NRF24L01 ---
    LidarDataPacket data_to_send;
    data_to_send.coord_x = coord_x;
    data_to_send.coord_y = coord_y;
    data_to_send.range_status = measure.RangeStatus;
    data_to_send.range_millimeter = measure.RangeMilliMeter;
    data_to_send.temperature = average_temp;
    data_to_send.humidity = humidity_event.relative_humidity;
    data_to_send.pressure = pressure;

    bool report = radio.write(&data_to_send, sizeof(data_to_send));
    if (!report) {
        Serial.println(F("NRF: Transmision de datos fallida."));
    }
}

void loop() {
    runLidarAndSensorSystem();
    delay(100);
}
