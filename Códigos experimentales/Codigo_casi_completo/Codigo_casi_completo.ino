// Incluye las librerías necesarias
#include <ESP32Servo.h>      // Librería específica para servos en ESP32
#include <Wire.h>            // Para comunicación I2C (usada por ambos sistemas)
#include <Adafruit_VL53L0X.h> // Para el sensor de distancia VL53L0X
#include <Adafruit_MPU6050.h> // Para el acelerómetro/giroscopio MPU6050
#include <Adafruit_Sensor.h>  // Librería base para sensores Adafruit

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

// --- Objetos Globales ---
// Sistema LIDAR
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_MPU6050 mpu;
Servo myservo;

// Sistema de Movimiento del Robot
// TwoWire Wire1 = TwoWire(1); // ¡ELIMINADA! Wire1 ya está definido por el núcleo ESP32.

// --- Variables Globales ---
// LIDAR
float current_gyro_angle_y = 0.0;
unsigned long last_lidar_update_time = 0;

// Robot (controlador de motores)
uint8_t MotorType = MOTOR_TYPE_JGB37_520_12V_110RPM;
uint8_t MotorEncoderPolarity = 0;
uint8_t car_forward[4] = {0, 233, 0, 23};
uint8_t car_stop[4] = {0, 0, 0, 0};
// uint8_t car_turnleft[4] = {0, 20, 0, 20}; 
// uint8_t car_retreat[4] = {0, 23, 0, 233}; 

// Control de turnos y estado del robot
bool isLidarTurn = true; // Inicia con el turno del LIDAR

// Para la máquina de estados de runRobotSystem()
unsigned long robot_action_timer = 0;
int robot_action_state = 0; // 0: listo para iniciar, 1: avanzando, 2: deteniéndose/pausado

// Duraciones para las acciones del robot (ajustables)
const unsigned long ROBOT_FORWARD_DURATION_MS = 2000; // Avanzar por 2 segundos
const unsigned long ROBOT_STOPPED_DURATION_MS = 1000; // Pausa después de detenerse por 1 segundo (cuando el LIDAR podría operar)

// Duración del turno del LIDAR (ajustable)
const unsigned long LIDAR_OPERATING_DURATION_MS = 1500; // El LIDAR girará y medirá por 1.5 segundos
unsigned long lidar_start_time = 0; // Se usará para medir el tiempo que el LIDAR está activo

/**
 * @brief Escribe un array de datos a un registro específico en el dispositivo I2C del controlador de motores.
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
    Serial.println("Inicializando sistema integrado (LIDAR y Robot alternando)...");

    Wire.begin(LIDAR_I2C_SDA_PIN, LIDAR_I2C_SCL_PIN);
    Serial.println("Bus I2C del LIDAR (Wire) inicializado en SDA=" + String(LIDAR_I2C_SDA_PIN) + ", SCL=" + String(LIDAR_I2C_SCL_PIN));

    Wire1.begin(MOTOR_I2C_SDA_PIN, MOTOR_I2C_SCL_PIN);
    Serial.println("Bus I2C del Controlador de Motores (Wire1) inicializado en SDA=" + String(MOTOR_I2C_SDA_PIN) + ", SCL=" + String(MOTOR_I2C_SCL_PIN));
    delay(200);

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

    myservo.attach(SERVO_PIN);
    myservo.write(0); // Asegurarse de que el servo esté detenido al inicio
    Serial.println("Servo del LIDAR inicializado y detenido (0 grados).");

    Serial.println("Inicializando Controlador de Motores...");
    if (!MotorController_WriteDataArray(MOTOR_TYPE_ADDR, &MotorType, 1)) {
        Serial.println("Error configurando tipo de motor.");
    }
    delay(5);
    if (!MotorController_WriteDataArray(MOTOR_ENCODER_POLARITY_ADDR, &MotorEncoderPolarity, 1)) {
        Serial.println("Error configurando polaridad del encoder.");
    }
    Serial.println("Controlador de Motores configurado.");

    last_lidar_update_time = millis();
    isLidarTurn = true; // El LIDAR comienza
    robot_action_state = 0; // Estado inicial para la secuencia del robot
    Serial.println("Sistema listo. LIDAR y Robot operarán en turnos.");
}

void runLidarSystem() {
    Serial.println("--- Turno del LIDAR ---");
    unsigned long current_time = millis();
    float dt = (current_time - last_lidar_update_time) / 1000.0;
    last_lidar_update_time = current_time;

    // --- Lógica de giro del LIDAR ---
    // Iniciar el giro si es la primera vez en este turno del LIDAR
    if (lidar_start_time == 0) { 
        myservo.write(90); // El servo empieza a girar continuamente
        Serial.println("LIDAR: Servo iniciando giro (90 grados).");
        lidar_start_time = current_time; // Registra el tiempo de inicio del giro del LIDAR
    }

    // Tomar lectura mientras el servo gira
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); 

    float distance_mm = 0.0;
    bool valid_detection = false;

    // Imprimir RAW (opcional, puede ser muy verboso)
    // Serial.print("LIDAR RAW:");
    // Serial.print(measure.RangeStatus);
    // Serial.print(",");
    // Serial.print(measure.RangeMilliMeter);
    // Serial.println(" mm");

    if (measure.RangeStatus != 4 && measure.RangeMilliMeter >= 4.5 && measure.RangeMilliMeter <= 150) {
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
        float coord_x = distance_mm * cos(angle_radians);
        float coord_y = distance_mm * sin(angle_radians);
        Serial.print("LIDAR COORD:");
        Serial.print(coord_x, 2);
        Serial.print(",");
        Serial.print(coord_y, 2);
        Serial.println();
    }
    
    // Verificar si el tiempo de operación del LIDAR ha terminado
    if (current_time - lidar_start_time >= LIDAR_OPERATING_DURATION_MS) {
        myservo.write(0); // Detener el servo del LIDAR (usando 0 grados)
        Serial.println("LIDAR: Servo detenido (0 grados). Turno finalizado.");
        lidar_start_time = 0; // Resetear para el siguiente turno
        isLidarTurn = false; // Ceder el turno al robot
    }
    // Si no ha terminado el tiempo, el LIDAR sigue su turno y el loop lo llamará de nuevo
}

void runRobotSystem() {
    // Serial.println("--- Turno del Robot ---"); // Puede ser muy verboso si se imprime en cada ciclo del estado
    unsigned long current_time = millis();

    switch (robot_action_state) {
        case 0: // Listo para iniciar (esperando el turno del robot)
            Serial.println("ROBOT: Listo para iniciar. Esperando turno del robot.");
            // Si el LIDAR terminó su turno, el robot puede empezar a moverse.
            if (!isLidarTurn) { 
                Serial.println("ROBOT: Iniciando movimiento hacia adelante.");
                MotorController_WriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_forward, 4);
                robot_action_timer = current_time;
                robot_action_state = 1; // Cambiar al estado de "avanzando"
            }
            break;

        case 1: // Robot avanzando, verificar duración
            if (current_time - robot_action_timer >= ROBOT_FORWARD_DURATION_MS) {
                Serial.println("ROBOT: Deteniendo motores.");
                MotorController_WriteDataArray(MOTOR_FIXED_SPEED_ADDR, car_stop, 4);
                robot_action_timer = current_time; // Reiniciar temporizador para la pausa
                robot_action_state = 2; // Cambiar al estado de "detenido/pausando"
            }
            break;

        case 2: // Robot detenido, verificar duración de la pausa
            if (current_time - robot_action_timer >= ROBOT_STOPPED_DURATION_MS) {
                Serial.println("ROBOT: Pausa finalizada. Cediendo control a LIDAR.");
                robot_action_state = 0; // Reiniciar estado para la próxima vez
                isLidarTurn = true;     // Ceder el turno al LIDAR
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