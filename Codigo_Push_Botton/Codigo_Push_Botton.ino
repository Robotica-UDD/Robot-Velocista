#define BOTON 21  // Pin donde está conectado el botón

void setup() {
  pinMode(BOTON, INPUT_PULLUP); // Configura el pin con resistencia pull-up interna
  Serial.begin(115200);         // Inicia la comunicación serial
}

void loop() {
  int estado = digitalRead(BOTON); // Lee el estado del botón
  
  if (estado == LOW) { // Si el botón está presionado (LOW debido a pull-up)
    Serial.println("Botón presionado");
  } else {
    Serial.println("Botón no presionado");
  }
  
  delay(200); // Pequeño retraso para evitar rebotes
}
