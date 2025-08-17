#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <RF24.h>

// --- Configuración de la red Wi-Fi ---
const char* ssid = "NogalRootNick";
const char* password = "alaska123";

// --- Configuración del servidor TCP ---
const int serverPort = 8888;
WiFiServer server(serverPort);
WiFiClient client;

#define NRF_CE_PIN  3   // Conecta CE del NRF24L01 a GPIO3
#define NRF_CSN_PIN 7   // Conecta CSN del NRF24L01 a GPIO7

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

const byte address[6] = "00001";

struct LidarDataPacket {
    float coord_x;
    float coord_y;
    uint8_t range_status;
    uint16_t range_millimeter;
};
const uint16_t MAX_DISTANCE_MM = 225;

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    Serial.println("Inicializando Estación Base (Receptor NRF24L01 y Servidor Wi-Fi)...");

    Serial.println("Conectando a la red Wi-Fi: " + String(ssid) + "...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nConectado a la red Wi-Fi.");
    Serial.print("Dirección IP del ESP32 (Estación Base): ");
    Serial.println(WiFi.localIP());

    server.begin();
    Serial.print("Servidor TCP iniciado en el puerto: ");
    Serial.println(serverPort);

    SPI.begin();
    Serial.println("\nSPI inicializado para ESP32-C3 SuperMini.");
    Serial.println("Verifica tus conexiones NRF24L01:");
    Serial.println("   CE   -> GPIO " + String(NRF_CE_PIN));
    Serial.println("   CSN  -> GPIO " + String(NRF_CSN_PIN));
    Serial.println("   MOSI -> GPIO 7");
    Serial.println("   MISO -> GPIO 2");
    Serial.println("   SCK  -> GPIO 6");

    // --- Inicialización del módulo NRF24L01 ---
    if (!radio.begin()) {
        Serial.println("¡ERROR CRÍTICO! NRF24L01 no responde. Comprueba el cableado (VCC, GND, SPI, CE, CSN) y el capacitor de 10-100uF.");
        while (1) {} // Detiene el programa si el NRF24L01 no inicializa
    } else {
        Serial.println("NRF24L01 detectado e inicializado. Detalles de configuración:");
        radio.printDetails();

        radio.setChannel(115);
        radio.setPALevel(RF24_PA_LOW);
        radio.setDataRate(RF24_250KBPS);

        radio.openReadingPipe(0, address);
        radio.startListening();

        Serial.println("NRF24L01 configurado correctamente como receptor.");
        Serial.println("Esperando datos del robot vía nRF24L01 y clientes TCP...");
    }
}

void loop() {
    if (!client || !client.connected()) {
        client = server.available();
        if (client) {
            Serial.println("Nuevo cliente TCP conectado desde: " + client.remoteIP().toString());
        }
    }

    if (radio.available()) {
        LidarDataPacket receivedData;

        radio.read(&receivedData, sizeof(receivedData));

        if (receivedData.range_millimeter <= MAX_DISTANCE_MM) {
            String dataString = "X:" + String(receivedData.coord_x, 2) +
                                ",Y:" + String(receivedData.coord_y, 2) +
                                ",Status:" + String(receivedData.range_status) +
                                ",Range_mm:" + String(receivedData.range_millimeter);

            Serial.println("--- Datos NRF24L01 Recibidos y ENVIADOS a Python ---");
            Serial.print("  Coordenada X: "); Serial.println(receivedData.coord_x, 2);
            Serial.print("  Coordenada Y: "); Serial.println(receivedData.coord_y, 2);
            Serial.print("  Estado de Rango: "); Serial.println(receivedData.range_status);
            Serial.print("  Distancia (mm): "); Serial.println(receivedData.range_millimeter);
            Serial.println("  (ENVIADO POR TCP: " + dataString + ")");
            Serial.println("--------------------------------------------------");

            if (client && client.connected()) {
                client.print("LIDAR_DATA_TCP:");
                client.println(dataString);
            } else {
                Serial.println("No hay cliente TCP conectado, datos LIDAR filtrados NO enviados a Python.");
            }
        } else {
            Serial.println("--- Dato NRF24L01 RECIBIDO, PERO DESCARTADO (FUERA DE RANGO) ---");
            Serial.print("  Coordenada X: "); Serial.println(receivedData.coord_x, 2);
            Serial.print("  Coordenada Y: "); Serial.println(receivedData.coord_y, 2);
            Serial.print("  Estado de Rango: "); Serial.println(receivedData.range_status);
            Serial.print("  Distancia (mm): "); Serial.println(receivedData.range_millimeter);
            Serial.println("  (Motivo: Distancia > " + String(MAX_DISTANCE_MM) + "mm)");
            Serial.println("----------------------------------------------------------------");
        }
    }

    if (client && client.connected() && client.available()) {
        String clientMessage = client.readStringUntil('\n');
        Serial.println("Recibido de TCP Client (Python): " + clientMessage);
    }

    delay(5);
}
