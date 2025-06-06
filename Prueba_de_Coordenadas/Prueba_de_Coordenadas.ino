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
