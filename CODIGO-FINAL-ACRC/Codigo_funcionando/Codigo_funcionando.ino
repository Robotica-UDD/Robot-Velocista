#include <Arduino.h>
#include <ESC.h>

#define PIN_ESC 10

#include <Arduino.h>

// Frecuencias de las notas musicales (en Hz)
#define NOTE_B0 31
#define NOTE_C1 33
#define NOTE_CS1 35
#define NOTE_D1 37
#define NOTE_DS1 39
#define NOTE_E1 41
#define NOTE_F1 44
#define NOTE_FS1 46
#define NOTE_G1 49
#define NOTE_GS1 52
#define NOTE_A1 55
#define NOTE_AS1 58
#define NOTE_B1 62
#define NOTE_C2 65
#define NOTE_CS2 69
#define NOTE_D2 73
#define NOTE_DS2 78
#define NOTE_E2 82
#define NOTE_F2 87
#define NOTE_FS2 93
#define NOTE_G2 98
#define NOTE_GS2 104
#define NOTE_A2 110
#define NOTE_AS2 117
#define NOTE_B2 123
#define NOTE_C3 131
#define NOTE_CS3 139
#define NOTE_D3 147
#define NOTE_DS3 156
#define NOTE_E3 165
#define NOTE_F3 175
#define NOTE_FS3 185
#define NOTE_G3 196
#define NOTE_GS3 208
#define NOTE_A3 220
#define NOTE_AS3 233
#define NOTE_B3 247
#define NOTE_C4 262
#define NOTE_CS4 277
#define NOTE_D4 294
#define NOTE_DS4 311
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_FS4 370
#define NOTE_G4 392
#define NOTE_GS4 415
#define NOTE_A4 440
#define NOTE_AS4 466
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_CS5 554
#define NOTE_D5 587
#define NOTE_DS5 622
#define NOTE_E5 659
#define NOTE_F5 698
#define NOTE_FS5 740
#define NOTE_G5 784
#define NOTE_GS5 831
#define NOTE_A5 880
#define NOTE_AS5 932
#define NOTE_B5 988
#define NOTE_C6 1047
#define NOTE_CS6 1109
#define NOTE_D6 1175
#define NOTE_DS6 1245
#define NOTE_E6 1319
#define NOTE_F6 1397
#define NOTE_FS6 1480
#define NOTE_G6 1568
#define NOTE_GS6 1661
#define NOTE_A6 1760
#define NOTE_AS6 1865
#define NOTE_B6 1976
#define NOTE_C7 2093
#define NOTE_CS7 2217
#define NOTE_D7 2349
#define NOTE_DS7 2489
#define NOTE_E7 2637
#define NOTE_F7 2794
#define NOTE_FS7 2960
#define NOTE_G7 3136
#define NOTE_GS7 3322
#define NOTE_A7 3520
#define NOTE_AS7 3729
#define NOTE_B7 3951
#define NOTE_C8 4186
#define NOTE_CS8 4435
#define NOTE_D8 4699
#define NOTE_DS8 4978

// Duraciones de las notas
#define WHOLE 800
#define HALF 400
#define QUARTER 200
#define EIGHTH 100
#define SIXTEENTH 500

bool freno = false;

unsigned long ultimoTiempoExtremos = 0;
bool extremosBlancos = false;
uint16_t posicionAnterior = 7000;  // Inicialización con un valor central.

const int sensorPin = A1;
int contador = 0;
unsigned long ultimoTiempo = 0;
const unsigned long intervalo = 390;  // Intervalo de 300 ms

const int sensorPin2 = A2;
int contador2 = 2;
unsigned long ultimoTiempo2 = 0;
const unsigned long intervalo2 = 300;  // Intervalo de 300 ms

const int umb_lat = 70;
unsigned long ultimoBlanco = 0;
const unsigned long esperaBlanco = 390;  // Tiempo de espera en ms

unsigned long ultimoCambioContador2 = 0;    // Última vez que cambió el contador2
const unsigned long tiempoSinCambio = 950;  // 2 segundos

ESC EDF27(PIN_ESC);

// Pines del MUX para seleccionar canales
const int S0 = 9;
const int S1 = 10;
const int S2 = 12;
const int S3 = 8;
bool fueraDeLinea = false;

float poslast = 0;
const uint16_t umbralNegro = 450;  // Umbral bajo para determinar negro

// Pin del MUX que entrega las señales de los sensores
const int SIG = A0;

// Pines definidos para el puente H
#define PWMA 5
#define INA1 6
#define INA2 7
#define PWMB 3
#define INB1 2
#define INB2 4

const uint8_t SensorCount = 15;

// Arrays para almacenar valores de sensores y calibración
uint16_t sensorValues[SensorCount];
uint16_t sensorMin[SensorCount];
uint16_t sensorMax[SensorCount];

float KP = 1.3;
float KD = 10;
float KI = 0;
int vel = 200;
int veladelante = 255;
int velatras = 255;
int veladelante2 = 255;
int velatras2 = 255;

int error1 = 0;
int error2 = 0;
int error3 = 0;
int error4 = 0;
int error5 = 0;
int error6 = 0;

int proporcional = 0;
int integral = 0;
int derivativo = 0;
int diferencial = 0;
int last_prop = 0;
int setpoint = 7000;
int BUZZER_PIN = 11;
int BUTTON_PIN = A4;

unsigned long lastButtonPressTime = 0;           // Última vez que se presionó el botón
const unsigned long confirmationTimeout = 5000;  // Tiempo de espera para confirmar (5 segundos)
const int buttonThreshold = 500;                 // Umbral para detectar pulsación (ajustar según tu circuito)

uint8_t modoSeleccionado = 0;  // Modo inicial

struct RobotMode {
  float kp;
  float kd;
  float ki;
  int velocidadBase;
  int velocidadAdelante;
  int velocidadAtras;
};

RobotMode modos[] = {

  //kp, kd,ki,  vB, vF1, vF2, vM
  { 0.83, 25, 0.001, 180, 255, 255},       // Modo 1: Lento
  { 1.3, 10, 0.001, 200, 255, 255 },  // Modo 2: Normal
  { 1.3, 10, 0.001, 215, 255, 255 },  // Modo 3: Rápido
  { 1.3, 10, 0.001, 230, 255, 255 }   // Modo 3: Extremo
};





const uint8_t numModos = sizeof(modos) / sizeof(modos[0]);

void setup() {

  pinMode(BUTTON_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  tone(BUZZER_PIN, NOTE_B6, 200);
  delay(80);
  tone(BUZZER_PIN, NOTE_E7, 500);
  // Selección del modo
  seleccionarModo();

  // Configurar el modo seleccionado
  setModo(modoSeleccionado);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(INB2, OUTPUT);
  pinMode(sensorPin, INPUT);   // Definir pin como entrada
  pinMode(sensorPin2, INPUT);  // Definir pin como entrada

  Serial.begin(115200);
  delay(1000);

  for (uint8_t i = 0; i < SensorCount; i++) {
    sensorMin[i] = 1024;
    sensorMax[i] = 0;
  }
  tone(BUZZER_PIN, NOTE_B6, 200);
  delay(80);
  tone(BUZZER_PIN, NOTE_E7, 500);
  Serial.println("Iniciando calibración...");
  unsigned long startTime = millis();
  while (millis() - startTime < 8000) {
    for (uint8_t i = 0; i < SensorCount; i++) {
      selectMuxChannel(i);
      uint16_t value = analogRead(SIG);
      if (value < sensorMin[i]) sensorMin[i] = value;
      if (value > sensorMax[i]) sensorMax[i] = value;
    }
  }
}

void loop() {
  if (millis() - ultimoBlanco >= esperaBlanco && checkBlanco()) {
    Serial.println("blancoooooo");
    contador--;
    contador2--;
    ultimoBlanco = millis();
  }

  for (uint8_t i = 0; i < SensorCount; i++) {
    selectMuxChannel(i);
    uint16_t value = analogRead(SIG);
    value = constrain(value, sensorMin[i], sensorMax[i]);
    sensorValues[i] = map(value, sensorMin[i], sensorMax[i], 1000, 0);
  }

  int value = digitalRead(sensorPin);
  // int value2 = digitalRead(sensorPin2);

  verificarValue();
  // verificarValue2();

  // if (millis() - ultimoCambioContador2 > tiempoSinCambio) {
  //   Motor(-255, -255);
  //   delay(10);
  //   contador2++;
  //   ultimoCambioContador2 = millis();
  // }

  // if (contador2 % 3 == 0) {
  //   Motor(-255, -255);
  //   delay(30);
  //   contador2++;

  if (contador < 2) {
    PID();
  } else {
    delay(100);
    Motor(-255, -255);
    Motor(0, 0);
    while (true)
      ;
  }

  Serial.print(contador2);
  Serial.print("     ");
  Serial.print(contador);
  Serial.println("  ");
}

void selectMuxChannel(uint8_t channel) {
  digitalWrite(S0, channel & 0x01);
  digitalWrite(S1, (channel & 0x02) >> 1);
  digitalWrite(S2, (channel & 0x04) >> 2);
  digitalWrite(S3, (channel & 0x08) >> 3);
  delayMicroseconds(50);
}

uint16_t calculateLinePosition() {
  long weightedSum = 0;
  long totalValue = 0;
  uint16_t threshold = 280;

  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > threshold) {
      weightedSum += (long)sensorValues[i] * (i * 1000);
      totalValue += sensorValues[i];
    }
  }

  if (totalValue == 0) return 7000;
  return weightedSum / totalValue;
}

void PID() {
  uint16_t position = calculateLinePosition();

  // Detectar si todos los sensores están en negro
  if (checkNegro()) {
    fueraDeLinea = true;

    // Determinar el lado por el que se salió
    if (posicionAnterior < 1000 && posicionAnterior > 0) {
      // Salida por la derecha
      Motor(veladelante2, -velatras2);  // Corrección para la derecha
    } else if (posicionAnterior > 13000 && posicionAnterior < 14000) {
      // Salida por la izquierda
      Motor(-velatras2, veladelante2);  // Corrección para la izquierda
    }
    return;  // Salimos del PID ya que estamos corrigiendo la salida
  }
  // if (position >= 6000 && position < 8000) {
  //   vel = 255;
  // }

  // Si no está fuera de línea, ejecutamos el PID normal
  fueraDeLinea = false;

  proporcional = position - setpoint;
  derivativo = proporcional - last_prop;
  integral = error1 + error2 + error3 + error4 + error5 + error6;
  last_prop = proporcional;
  error6 = error5;
  error5 = error4;
  error4 = error3;
  error3 = error2;
  error2 = error1;
  error1 = proporcional;
  int diferencial = (proporcional * KP) + (derivativo * KD) + (integral * KI);
  if (diferencial > vel) diferencial = vel;
  else if (diferencial < -vel) diferencial = -vel;
  (diferencial < 0) ? Motor(vel, vel + diferencial) : Motor(vel - diferencial, vel);

  if (position <= 4500 && position >= 2000) {
    Motor(veladelante, -velatras);
  }
  if (position >= 9500 && position < 12000) {
    Motor(-velatras, veladelante);
  }
  if (position <= 2000) {
    Motor(veladelante2, -velatras2);
  }
  if (position >= 12000) {
    Motor(-velatras2, veladelante2);
  }


  // Actualizamos la posición anterior
  posicionAnterior = position;
}

void Motor(int left, int right) {
  Motoriz(left);
  Motorde(right);
}

void Motoriz(int value) {
  if (value >= 0) {
    digitalWrite(INB1, HIGH);
    digitalWrite(INB2, LOW);
  } else {
    digitalWrite(INB1, LOW);
    digitalWrite(INB2, HIGH);
    value = -value;
  }
  analogWrite(PWMB, constrain(value, 0, 255));
}

void verificarValue() {
  if (millis() - ultimoTiempo >= intervalo) {
    int value = analogRead(sensorPin);
    int value2 = analogRead(sensorPin2);
    if (value <= umb_lat && value2 > umb_lat) {
      contador++;
      ultimoTiempo = millis();
      ultimoCambioContador2 = millis();
    }
  }
}

void verificarValue2() {
  if (millis() - ultimoTiempo2 >= intervalo2) {
    int value2 = analogRead(sensorPin2);
    int value = analogRead(sensorPin);
    if (value2 <= umb_lat && value > umb_lat) {
      contador2++;
      ultimoTiempo2 = millis();
      ultimoCambioContador2 = millis();
    }
  }
}

void Motorde(int value) {
  if (value >= 0) {
    digitalWrite(INA1, HIGH);
    digitalWrite(INA2, LOW);
  } else {
    digitalWrite(INA1, LOW);
    digitalWrite(INA2, HIGH);
    value = -value;
  }
  analogWrite(PWMA, constrain(value, 0, 255));
}

bool checkBlanco() {
  int count = 0;
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > 450) {
      count++;
    }
  }
  return count >= 9;
}
// Variable global para umbral de detección de negro

// Función para verificar si todos los sensores están en negro
bool checkNegro() {
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > umbralNegro) {
      return false;  // Si algún sensor no está en negro, retornamos false
    }
  }
  return true;  // Todos los sensores están en negro
}

void seleccionarModo() {
  Serial.println("Selecciona el modo usando el botón.");
  Serial.println("Presiona el botón para cambiar de modo. Espera 5 segundos para confirmar.");

  while (true) {
    int analogValue = analogRead(BUTTON_PIN);
    bool buttonPressed = (analogValue > buttonThreshold);

    if (buttonPressed) {
      // Incrementar el modo y reiniciar el temporizador
      modoSeleccionado = (modoSeleccionado + 1) % numModos;
      lastButtonPressTime = millis();
      mostrarModoActual();

      // Beep corto para cambio de modo
      tone(BUZZER_PIN, 2000, 100);
      delay(50); // Frecuencia: 1000 Hz, duración: 200 ms
      tone(BUZZER_PIN, 2500, 100);
      delay(50); // Frecuencia: 1000 Hz, duración: 200 ms
      tone(BUZZER_PIN, 3000, 100); // Frecuencia: 1000 Hz, duración: 200 ms
      delay(300);  // Pausa para evitar múltiples incrementos por rebotes
    }

    // Confirmar la selección si han pasado 5 segundos sin interacción
    if (millis() - lastButtonPressTime > confirmationTimeout) {
      Serial.println("Modo confirmado.");

      // Beep largo para confirmación
      delay(1000);  // Pausa para asegurar que el beep largo termine
      noTone(BUZZER_PIN); // Asegura que el buzzer se apague completamente
      break;
    }
  }
}

  void mostrarModoActual() {
    Serial.print("Modo seleccionado: ");
    Serial.println(modoSeleccionado + 1);
    Serial.print("KP=");
    Serial.print(modos[modoSeleccionado].kp);
    Serial.print(", KD=");
    Serial.print(modos[modoSeleccionado].kd);
    Serial.print(", KI=");
    Serial.print(modos[modoSeleccionado].ki);
    Serial.print(", Velocidad base=");
    Serial.println(modos[modoSeleccionado].velocidadBase);
  }
  void setModo(uint8_t modo) {
    KP = modos[modo].kp;
    KD = modos[modo].kd;
    KI = modos[modo].ki;
    vel = modos[modo].velocidadBase;
    veladelante = modos[modo].velocidadAdelante;
    velatras = modos[modo].velocidadAtras;

    Serial.print("Modo seleccionado: ");
    Serial.println(modo + 1);
    Serial.print("KP=");
    Serial.print(KP);
    Serial.print(", KD=");
    Serial.print(KD);
    Serial.print(", KI=");
    Serial.print(KI);
    Serial.print(", Velocidad base=");
    Serial.println(vel);
  }