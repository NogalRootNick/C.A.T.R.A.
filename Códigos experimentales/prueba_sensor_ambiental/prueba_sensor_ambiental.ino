#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>

Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;

void setup() {
  Serial.begin(9600); // Velocidad de baudios para Arduino UNO

  // Inicializa el bus I2C
  Wire.begin();

  Serial.println("Probando modulo AHT20 + BMP280...");

  // Inicializa el sensor AHT20
  if (!aht.begin()) {
    Serial.println("AHT20 no encontrado. Verifique las conexiones.");
    while (1) delay(10);
  }
  Serial.println("AHT20 encontrado y listo.");

  // Inicializa el sensor BMP280
  // La direccion I2C por defecto es 0x76. Algunos modulos usan 0x77.
  if (!bmp.begin(0x77)) {
    Serial.println("BMP280 no encontrado. Verifique las conexiones o la direccion I2C.");
    while (1) delay(10);
  }
  Serial.println("BMP280 encontrado y listo.");
}

void loop() {
  // --- Lectura de datos del AHT20 ---
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

  // --- Lectura de datos del BMP280 ---
  // Nota: El BMP280 necesita un modo de medicion para leer los datos
  bmp.takeForcedMeasurement();
  float pressure = bmp.readPressure();
  float temp_bmp = bmp.readTemperature();

  Serial.println("--------------------");
  Serial.print("AHT20 | Temp: ");
  Serial.print(temp.temperature);
  Serial.print(" °C | Humedad: ");
  Serial.print(humidity.relative_humidity);
  Serial.println(" %");

  Serial.print("BMP280 | Temp: ");
  Serial.print(temp_bmp);
  Serial.print(" °C | Presion: ");
  Serial.print(pressure / 100.0F); // Convertir de Pa a hPa
  Serial.println(" hPa");

  delay(5000); // Espera 5 segundos antes de la proxima lectura
}