#include <Arduino.h>
#include "esp_sleep.h"

const char* battery_status[] = {"hibernacion", "iniciado de nuevo"};
#define WAKEUP_PIN 0

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  
  if (cause == ESP_SLEEP_WAKEUP_UNDEFINED) {
    Serial.println("Estado: " + String(battery_status[0]));
    Serial.println("Configurando wakeup...");
    esp_sleep_enable_ext0_wakeup((gpio_num_t)WAKEUP_PIN, 1);
  } 
  else {
    Serial.println("Estado: " + String(battery_status[1]));
    
    switch (cause) {
      case ESP_SLEEP_WAKEUP_EXT0:
        Serial.println("Motivo: Despertado por GPIO " + String(WAKEUP_PIN));
        break;
      case ESP_SLEEP_WAKEUP_TIMER:
        Serial.println("Motivo: Despertado por temporizador");
        break;
      default:
        Serial.println("Motivo: Desconocido");
    }
  }

  Serial.println("Entrando en hibernación en 5 segundos...");
  delay(5000);
  esp_deep_sleep_start();
}

void loop() {
}