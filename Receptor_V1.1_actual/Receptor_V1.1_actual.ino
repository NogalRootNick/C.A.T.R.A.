// Librerias necesarias
#include <SPI.h>
#include <RF24.h>

// --- DEFINICIONES PARA NRF24L01 ---
#define NRF_CE_PIN 9
#define NRF_CSN_PIN 10

// Objeto del NRF24L01
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const byte address[6] = "00001";

// Estructura para los datos a recibir
struct LidarDataPacket {
    float coord_x;
    float coord_y;
    uint8_t range_status;
    uint16_t range_millimeter;
    float temperature_c;
    float humidity_rh;
    float pressure_hpa;
};

// Variable para almacenar los datos recibidos
LidarDataPacket receivedData;

void setup() {
    Serial.begin(9600);
    while (!Serial);

    if (!radio.begin()) {
        Serial.println(F("Error al iniciar el NRF24L01!"));
        while (1);
    }
    
    radio.setChannel(115);
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.openReadingPipe(1, address);
    radio.startListening();

    Serial.println("Receptor NRF24L01 listo. Escuchando...");
}

void loop() {
    if (radio.available()) {
        radio.read(&receivedData, sizeof(receivedData));

        Serial.print("LIDAR COORD: X=");
        Serial.print(receivedData.coord_x, 2);
        Serial.print(" Y=");
        Serial.print(receivedData.coord_y, 2);
        Serial.print(" Temp: ");
        Serial.print(receivedData.temperature_c, 1);
        Serial.print(" Hum: ");
        Serial.print(receivedData.humidity_rh, 1);
        Serial.print(" Pres: ");
        Serial.print(receivedData.pressure_hpa, 2);
        Serial.print(" RAW: ");
        Serial.print(receivedData.range_millimeter);
        Serial.println();
    }
}