#include <stdlib.h> // Para rand() y srand()
#include <time.h>   // Para time()

void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(0)); // Inicializa el generador de números aleatorios usando la lectura de un pin analógico no conectado
}

void loop() {
  for (float i = 0.100; i < 1000000.10; i++) {
    float numero_aleatorio = random(0, 1000); // Genera un número aleatorio entre 0 y 100
    Serial.println(numero_aleatorio);
  }
  delay(5000); // Espera 1 segundo antes de repetir
}