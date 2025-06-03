/*
|==========================|
|  CODIGO VIEJO DE TESTEO  |
|==========================|

float x = 1.4;
float y = 1.02;

void setup() {
  Serial.begin(9600);
  delay(50);
}

void loop() {
  for (float coordValue = 0.00; coordValue < 10.00; coordValue += 0.1) {
    Serial.print("Coordenadas: ");
    Serial.print(coordValue);
    Serial.print(" , ");
    Serial.println(coordValue);

    delay(100);
  }
  delay(1000);
}*/


/*
|==========================|
|  CODIGO NUEVO DE TESTEO  |
|==========================|
*/

float x = 0.55;
float y = 0.23;

float coords[2] = {x++, y++};

void setup() {
  Serial.begin(9600);
  Serial.println("TEST DEL LIDAR INICIADO");
  delay(1000);
}

void loop() {
  Serial.println(" ");
  Serial.print(coords[0]);
  Serial.print(" , ");
  Serial.print(coords[1]);
  Serial.println(" ");
  delay(500);
}
