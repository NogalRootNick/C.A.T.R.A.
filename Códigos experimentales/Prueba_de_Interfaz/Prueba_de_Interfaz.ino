// Pines para sensores (opcional, depende de tu hardware)
// #include <DHT.h>
// #define DHTPIN 2
// #define DHTTYPE DHT22
// DHT dht(DHTPIN, DHTTYPE);

void setup() {
  // Inicia la comunicación serial
  Serial.begin(9600);
  
  // Si usas un sensor DHT, inícialo
  // dht.begin();
}

void loop() {
  // --- 1. Lectura de Sensores (reemplaza con tus lecturas reales) ---
  
  // Simula la lectura de coordenadas X e Y
  float x_coord = random(-1000, 1000) / 10.0;
  float y_coord = random(-1000, 1000) / 10.0;

  // Simula la lectura de temperatura y humedad
  float temperature = 20.0 + (random(0, 100) / 20.0); // Temp entre 20.0 y 25.0
  float humidity = 50.0 + (random(0, 100) / 10.0);    // Humedad entre 50.0 y 60.0
  
  // Simula la lectura de presión y un valor "RAW"
  float pressure = 1013.25 + (random(-50, 50) / 10.0);
  int raw_value = random(0, 1023);

  // --- 2. Envío de Datos por Puerto Serie (estilo println) ---
  
  // Envía los datos pieza por pieza. Serial.print() no añade un salto de línea.
  Serial.print("LIDAR COORD: X=");
  Serial.print(x_coord, 2); // El '2' indica 2 decimales
  Serial.print(" Y=");
  Serial.print(y_coord, 2);
  Serial.print(" Temp: ");
  Serial.print(temperature, 1); // El '1' indica 1 decimal
  Serial.print(" Hum: ");
  Serial.print(humidity, 1);
  Serial.print(" Pres: ");
  Serial.print(pressure, 2);
  Serial.print(" RAW: ");
  Serial.print(raw_value);

  // Espera un tiempo antes de la siguiente lectura
  delay(1000); // Envía datos cada segundo
}