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
}