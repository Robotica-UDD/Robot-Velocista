// Pines definidos para el puente H
#define PWMA 13  // Pin para señal PWM motor A
#define INA1 14  // Pin de dirección motor A
#define INA2 27  // Pin de dirección motor A

#define PWMB 12  // Pin para señal PWM motor B
#define INB1 26  // Pin de dirección motor B
#define INB2 25  // Pin de dirección motor B

// Velocidad inicial de los motores (0-255)
int velocidadA = 128; // 50% de la velocidad máxima motor A
int velocidadB = 128; // 50% de la velocidad máxima motor B

void setup() {
  // Configuración de pines como salida para el motor A
  pinMode(PWMA, OUTPUT);
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);

  // Configuración de pines como salida para el motor B
  pinMode(PWMB, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(INB2, OUTPUT);

  // Inicializamos ambos motores apagados
  detenerMotorA();
  detenerMotorB();
}

void loop() {
  // Ejemplo de control de ambos motores
  avanzarMotores(velocidadA, velocidadB); // Avanzar ambos motores
  delay(2000);                            // Mantener por 2 segundos

  detenerMotores();                       // Detener ambos motores
  delay(1000);                            // Pausa de 1 segundo

  retrocederMotores(velocidadA, velocidadB); // Retroceder ambos motores
  delay(2000);                               // Mantener por 2 segundos

  detenerMotores();                       // Detener ambos motores
  delay(1000);                            // Pausa de 1 segundo
}

// Función para avanzar ambos motores
void avanzarMotores(int velA, int velB) {
  avanzarMotorA(velA);
  avanzarMotorB(velB);
}

// Función para retroceder ambos motores
void retrocederMotores(int velA, int velB) {
  retrocederMotorA(velA);
  retrocederMotorB(velB);
}

// Función para detener ambos motores
void detenerMotores() {
  detenerMotorA();
  detenerMotorB();
}

// Funciones específicas para motor A
void avanzarMotorA(int velocidad) {
  digitalWrite(INA1, HIGH); // Dirección hacia adelante
  digitalWrite(INA2, LOW);
  analogWrite(PWMA, velocidad); // Aplicar PWM
}

void retrocederMotorA(int velocidad) {
  digitalWrite(INA1, LOW);  // Dirección hacia atrás
  digitalWrite(INA2, HIGH);
  analogWrite(PWMA, velocidad); // Aplicar PWM
}

void detenerMotorA() {
  digitalWrite(INA1, LOW);
  digitalWrite(INA2, LOW);
  analogWrite(PWMA, 0); // Detener PWM
}

// Funciones específicas para motor B
void avanzarMotorB(int velocidad) {
  digitalWrite(INB1, HIGH); // Dirección hacia adelante
  digitalWrite(INB2, LOW);
  analogWrite(PWMB, velocidad); // Aplicar PWM
}

void retrocederMotorB(int velocidad) {
  digitalWrite(INB1, LOW);  // Dirección hacia atrás
  digitalWrite(INB2, HIGH);
  analogWrite(PWMB, velocidad); // Aplicar PWM
}

void detenerMotorB() {
  digitalWrite(INB1, LOW);
  digitalWrite(INB2, LOW);
  analogWrite(PWMB, 0); // Detener PWM
}
