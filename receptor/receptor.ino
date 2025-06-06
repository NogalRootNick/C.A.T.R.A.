#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <RF24.h> // RF24.h ya incluye lo necesario, nRF24L01.h no es estrictamente necesario aquí.

// --- Configuración de la red Wi-Fi ---
const char* ssid = "NogalRootNick";       // ¡IMPORTANTE! Reemplaza con el nombre de tu red Wi-Fi
const char* password = "alaska123"; // ¡IMPORTANTE! Reemplaza con la contraseña de tu red Wi-Fi

// --- Configuración del servidor TCP ---
const int serverPort = 8888; // Puerto que usará el servidor TCP
WiFiServer server(serverPort);
WiFiClient client; // Objeto para el cliente conectado (la app Python)

// --- Pines nRF24L01 para ESP32-C3 SuperMini ---
// ¡VERIFICA ESTOS PINOS SEGÚN TU CONEXIÓN FÍSICA!
// Basado en tu último código, estos son los pines que usas:
#define NRF_CE_PIN  3   // Conecta CE del NRF24L01 a GPIO3
#define NRF_CSN_PIN 7   // Conecta CSN del NRF24L01 a GPIO7

// Objeto RF24: Asume que la librería RF24 usará el objeto SPI global por defecto.
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

// La dirección de comunicación DEBE coincidir con la del transmisor del robot
const byte address[6] = "00001";

// --- ESTRUCTURA DE DATOS PARA RECIBIR ---
// ¡ESTA DEBE SER IDÉNTICA A LA DEL TRANSMISOR DEL ROBOT!
// Si el transmisor cambia su estructura, esta también debe cambiar.
struct LidarDataPacket {
    float coord_x;
    float coord_y;
    uint8_t range_status;       // RAW: VL53L0X_RangingMeasurementData_t::RangeStatus
    uint16_t range_millimeter; // RAW: VL53L0X_RangingMeasurementData_t::RangeMilliMeter
};
// ----------------------------------------

// =======================================================
// === NUEVA LÍNEA: UMBRAL DE DISTANCIA MÁXIMA PARA ENVIAR ===
// =======================================================
const uint16_t MAX_DISTANCE_MM = 225; // Las mediciones por encima de este valor NO se enviarán a Python.
                                     // Puedes ajustar este valor fácilmente para la competencia.


void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); } // Espera a que el puerto serial esté listo

    Serial.println("Inicializando Estación Base (Receptor NRF24L01 y Servidor Wi-Fi)...");

    // --- Configuración e Inicialización Wi-Fi ---
    Serial.println("Conectando a la red Wi-Fi: " + String(ssid) + "...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nConectado a la red Wi-Fi.");
    Serial.print("Dirección IP del ESP32 (Estación Base): ");
    Serial.println(WiFi.localIP()); // ¡IMPORTANTE! Anota esta IP para tu script Python

    server.begin(); // Iniciar el servidor TCP
    Serial.print("Servidor TCP iniciado en el puerto: ");
    Serial.println(serverPort);

    // --- Configuración e Inicialización SPI para NRF24L01 ---
    // SPI.begin() sin parámetros usa los pines SPI por defecto del ESP32-C3:
    // MOSI: GPIO7, MISO: GPIO2, SCLK: GPIO6
    SPI.begin();
    Serial.println("\nSPI inicializado para ESP32-C3 SuperMini.");
    Serial.println("Verifica tus conexiones NRF24L01:");
    Serial.println("   CE   -> GPIO " + String(NRF_CE_PIN));
    Serial.println("   CSN  -> GPIO " + String(NRF_CSN_PIN));
    Serial.println("   MOSI -> GPIO 7"); // ¡Importante! Asegúrate de que el pin MOSI del nRF24L01 vaya a GPIO7 del ESP32-C3
    Serial.println("   MISO -> GPIO 2");
    Serial.println("   SCK  -> GPIO 6");

    // --- Inicialización del módulo NRF24L01 ---
    if (!radio.begin()) {
        Serial.println("¡ERROR CRÍTICO! NRF24L01 no responde. Comprueba el cableado (VCC, GND, SPI, CE, CSN) y el capacitor de 10-100uF.");
        while (1) {} // Detiene el programa si el NRF24L01 no inicializa
    } else {
        Serial.println("NRF24L01 detectado e inicializado. Detalles de configuración:");
        radio.printDetails(); // ¡Muy útil para depurar!

        // **Configuración del NRF24L01 (¡DEBE COINCIDIR EXACTAMENTE CON EL TRANSMISOR!)**
        radio.setChannel(115);          // Canal de comunicaación
        radio.setPALevel(RF24_PA_LOW);  // Nivel de potencia (LOW para receptor es eficiente)
        radio.setDataRate(RF24_250KBPS); // Velocidad de datos

        // Abre una tubería de lectura (pipe 0, es común usar 0 para la principal)
        radio.openReadingPipe(0, address); // Se usa pipe 0 en el transmisor por defecto, mejor usar el mismo.
        radio.startListening();          // Pone el módulo en modo de escucha

        Serial.println("NRF24L01 configurado correctamente como receptor.");
        Serial.println("Esperando datos del robot vía nRF24L01 y clientes TCP...");
    }
}

void loop() {
    // --- Manejo de la conexión del cliente TCP (App Python) ---
    if (!client || !client.connected()) { // Si no hay cliente conectado o se desconectó
        client = server.available(); // Intenta aceptar un nuevo cliente
        if (client) {
            Serial.println("Nuevo cliente TCP conectado desde: " + client.remoteIP().toString());
        }
    }

    // --- Verificar y Leer Datos del NRF24L01 ---
    if (radio.available()) {
        LidarDataPacket receivedData; // Instancia de la estructura para almacenar los datos

        // Lee los datos disponibles directamente en la estructura
        radio.read(&receivedData, sizeof(receivedData));

        // =======================================================================
        // === INICIO DE LA LÓGICA DE FILTRADO Y MENSAJES DE DEPURACIÓN EXACTOS ===
        // =======================================================================
        if (receivedData.range_millimeter <= MAX_DISTANCE_MM) {
            // Los datos están dentro del rango permitido (<= 150mm), procesar y enviar a Pythona
            String dataString = "X:" + String(receivedData.coord_x, 2) +
                                ",Y:" + String(receivedData.coord_y, 2) + // Quitamos el espacio para consistencia
                                ",Status:" + String(receivedData.range_status) +
                                ",Range_mm:" + String(receivedData.range_millimeter);

            // Imprimir los datos recibidos y confirmación de envío por Serial (solo si se envían a Python)
            Serial.println("--- Datos NRF24L01 Recibidos y ENVIADOS a Python ---");
            Serial.print("  Coordenada X: "); Serial.println(receivedData.coord_x, 2);
            Serial.print("  Coordenada Y: "); Serial.println(receivedData.coord_y, 2);
            Serial.print("  Estado de Rango: "); Serial.println(receivedData.range_status);
            Serial.print("  Distancia (mm): "); Serial.println(receivedData.range_millimeter);
            Serial.println("  (ENVIADO POR TCP: " + dataString + ")");
            Serial.println("--------------------------------------------------");

            if (client && client.connected()) {
                client.print("LIDAR_DATA_TCP:"); // Prefijo para que Python sepa qué tipo de mensaje es
                client.println(dataString);
            } else {
                Serial.println("No hay cliente TCP conectado, datos LIDAR filtrados NO enviados a Python.");
            }
        } else {
            // Los datos están fuera del rango permitido (> 150mm), solo imprimir en Serial, NO ENVIAR A PYTHON
            Serial.println("--- Dato NRF24L01 RECIBIDO, PERO DESCARTADO (FUERA DE RANGO) ---");
            Serial.print("  Coordenada X: "); Serial.println(receivedData.coord_x, 2);
            Serial.print("  Coordenada Y: "); Serial.println(receivedData.coord_y, 2);
            Serial.print("  Estado de Rango: "); Serial.println(receivedData.range_status);
            Serial.print("  Distancia (mm): "); Serial.println(receivedData.range_millimeter);
            Serial.println("  (Motivo: Distancia > " + String(MAX_DISTANCE_MM) + "mm)");
            Serial.println("----------------------------------------------------------------");
        }
        // =======================================================================
        // === FIN DE LA LÓGICA DE FILTRADO Y MENSAJES DE DEPURACIÓN EXACTOS ===
        // =======================================================================
    }

    // --- Verificar si hay datos recibidos del cliente TCP (si tu app Python envía algo) ---
    if (client && client.connected() && client.available()) {
        String clientMessage = client.readStringUntil('\n'); // Lee el mensaje del cliente TCP
        Serial.println("Recibido de TCP Client (Python): " + clientMessage); // Imprime el mensaje en el Serial USB
    }

    delay(5); // Pequeño retraso para evitar ciclos de lectura muy rápidos y liberar CPU
}
