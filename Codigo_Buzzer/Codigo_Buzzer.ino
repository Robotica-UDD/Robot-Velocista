#define BUZZER_PIN 5  // Pin del buzzer

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);  // Configura el pin como salida
}

void loop() {
  // Genera un tono de 2000 Hz por 1 segundo
  tone(BUZZER_PIN, 2000);  // 2000 Hz de frecuencia
  delay(1000);             // Mantiene el tono durante 1 segundo
  
  // Detiene el tono
  noTone(BUZZER_PIN);      // Apaga el tono
  delay(1000);             // Silencio por 1 segundo
}
