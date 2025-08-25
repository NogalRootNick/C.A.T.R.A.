// Variables para las coordenadas
float x = 0.0;
float y = 0.0;

void setup() {
  Serial.begin(9600);
  Serial.println("INICIANDO IMPRESIÓN DE COORDENADAS");
  Serial.println("---------------------------------");
}

void loop() {
  // Bucle para iterar y mostrar el incremento
  // El bucle se ejecutará 20 veces
  for (int i = 0; i < 20; i++) {
    // Incrementar cada coordenada en 0.43
    x = x + 0.43; 
    y = y + 0.43;

    // Imprimir el resultado en el formato deseado
    Serial.print("coords: ");
    Serial.print("x: ");
    Serial.print(x, 2); // El '2' especifica el número de decimales a mostrar
    Serial.print(" | y: ");
    Serial.println(y, 2);

    // Pequeña pausa para que se pueda leer la salida
    delay(500); 
  }

  // Bucle infinito para detener el programa una vez que el bucle for termine
  while (true) {
    // No hacer nada
  }
}