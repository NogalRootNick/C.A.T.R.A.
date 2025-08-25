// --- Librerías necesarias para Arduino Uno ---
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h> // Para algunas definiciones útiles como RF24_PA_MAX, etc. (aunque RF24.h suele incluirlas)

// --- Definiciones para NRF24L01 en Arduino Uno ---
// Conexiones típicas para Arduino Uno y NRF24L01:
// NRF24L01  | Arduino Uno
// GND       | GND
// VCC       | 3.3V (¡IMPORTANTE! No 5V directamente)
// CE        | Pin digital 9
// CSN       | Pin digital 10 (Este es el pin SS por defecto para SPI)
// SCK       | Pin digital 13 (SCK del SPI)
// MOSI      | Pin digital 11 (MOSI del SPI)
// MISO      | Pin digital 12 (MISO del SPI)

#define NRF_CE_PIN 9  // Pin CE del NRF24L01 conectado al digital pin 9
#define NRF_CSN_PIN 10 // Pin CSN del NRF24L01 conectado al digital pin 10 (SPI SS)

// --- Objeto Global NRF24L01 ---
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

// --- Dirección de la tubería de recepción ---
// Debe coincidir EXACTAMENTE con la dirección de transmisión de tu ESP32.
const byte address[6] = "00001";

// --- Estructura para los datos a recibir ---
// ¡Debe ser idéntica a la estructura definida en tu ESP32!
struct LidarDataPacket {
    float coord_x;
    float coord_y;
    uint8_t range_status;
    uint16_t range_millimeter;
};

// --- Objeto para almacenar los datos recibidos ---
LidarDataPacket receivedData;

void setup() {
    Serial.begin(9600); // Velocidad del monitor serial para depuración
    while (!Serial) {
        ; // Espera a que el puerto serial esté disponible (solo para placas con USB nativo)
    }
    Serial.println("Inicializando receptor NRF24L01...");

    // --- Inicialización NRF24L01 ---
    // La librería RF24 se encarga de inicializar SPI automáticamente
    if (!radio.begin()) {
        Serial.println(F("¡Error al iniciar el NRF24L01! Comprueba el cableado."));
        // Aquí podrías añadir un bucle infinito o un LED de error
        while (1);
    } else {
        radio.setChannel(115);            // Mismo canal que el transmisor (ESP32)
        radio.setPALevel(RF24_PA_MAX);    // Mismo nivel de potencia que el transmisor
        radio.setDataRate(RF24_250KBPS);  // Misma velocidad de datos que el transmisor (RF24_250KBPS para mayor alcance)
        radio.openReadingPipe(1, address); // Abre una tubería de lectura con la dirección definida
        radio.startListening();           // Pone el módulo en modo receptor
        Serial.println("NRF24L01 inicializado correctamente como receptor.");
        Serial.println("Esperando datos del ESP32...");
    }
}

void loop() {
    // Comprueba si hay datos disponibles para ser leídos
    if (radio.available()) {
        // Lee los datos y los almacena en la estructura 'receivedData'
        radio.read(&receivedData, sizeof(receivedData));

        Serial.println("--- Datos LIDAR Recibidos ---");
        Serial.print("Coordenada X: ");
        Serial.print(receivedData.coord_x, 2); // Imprime con 2 decimales
        Serial.println(" mm");

        Serial.print("Coordenada Y: ");
        Serial.print(receivedData.coord_y, 2); // Imprime con 2 decimales
        Serial.println(" mm");

        Serial.print("Estado del Rango (RAW): ");
        Serial.println(receivedData.range_status);

        Serial.print("Distancia (RAW): ");
        Serial.print(receivedData.range_millimeter);
        Serial.println(" mm");

        Serial.println("-----------------------------");
    }
    // Pequeño retardo para evitar sobrecargar el puerto serial y permitir otras tareas
    delay(5);
}
