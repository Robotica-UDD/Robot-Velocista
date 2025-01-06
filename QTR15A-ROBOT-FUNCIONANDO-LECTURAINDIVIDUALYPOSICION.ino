#include <Arduino.h>

// Pines del MUX para seleccionar canales
const int S0 = 19;
const int S1 = 18;
const int S2 = 17;
const int S3 = 16;

// Pin del MUX que entrega las señales de los sensores
const int SIG = 32;  // Conexión común del MUX al ADC del ESP32

// Número total de sensores
const uint8_t SensorCount = 15;

// Arreglo para almacenar los valores de los sensores
uint16_t sensorValues[SensorCount];

// Arrays para los valores de calibración
uint16_t sensorMin[SensorCount];
uint16_t sensorMax[SensorCount];

void setup() {
  // Configuración de pines del MUX como salida
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  Serial.begin(115200);
  delay(1000);

  // Inicializar valores de calibración
  for (uint8_t i = 0; i < SensorCount; i++) {
    sensorMin[i] = 4095;  // Máximo valor ADC (12 bits)
    sensorMax[i] = 0;     // Mínimo valor ADC
  }

  Serial.println("Iniciando calibración...");

  // Calibración
  unsigned long startTime = millis();
  while (millis() - startTime < 8000)  // Calibrar durante 5 segundos
  {
    for (uint8_t i = 0; i < SensorCount; i++) {
      selectMuxChannel(i);  // Seleccionar canal en el MUX
      uint16_t value = analogRead(SIG);

      // Actualizar valores de calibración
      if (value < sensorMin[i])
        sensorMin[i] = value;
      if (value > sensorMax[i])
        sensorMax[i] = value;
    }
  }

  Serial.println("Calibración completada.");
  // Imprimir valores de calibración
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": Min=");
    Serial.print(sensorMin[i]);
    Serial.print(", Max=");
    Serial.println(sensorMax[i]);
  }
}

void loop() {
  // Leer valores calibrados de los sensores
  for (uint8_t i = 0; i < SensorCount; i++) {
    selectMuxChannel(i);  // Seleccionar canal en el MUX
    uint16_t value = analogRead(SIG);

    // Normalizar valor entre 0 y 1000
    value = constrain(value, sensorMin[i], sensorMax[i]);               // Ajustar al rango calibrado
    sensorValues[i] = map(value, sensorMin[i], sensorMax[i], 1000, 0);  // Invertir lectura
  }

  // Calcular la posición de la línea (de 0 a 5000)
  uint16_t position = calculateLinePosition();

  // Imprimir la posición calculada primero
  Serial.print("Posición: ");
  Serial.println(position);

  // Imprimir valores de los sensores
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print("S");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();

  delay(400);
}

// Función para seleccionar un canal en el MUX
void selectMuxChannel(uint8_t channel) {
  digitalWrite(S0, channel & 0x01);
  digitalWrite(S1, (channel & 0x02) >> 1);
  digitalWrite(S2, (channel & 0x04) >> 2);
  digitalWrite(S3, (channel & 0x08) >> 3);
  delayMicroseconds(50);  // Retardo para estabilización del MUX
}

// Función para calcular la posición de la línea
uint16_t calculateLinePosition() {
  long weightedSum = 0;
  long totalValue = 0;
  uint16_t threshold = 215; // Umbral para filtrar contribuciones bajas

  for (uint8_t i = 0; i < SensorCount; i++) {
    // Ignorar sensores con valores por debajo del umbral
    if (sensorValues[i] > threshold) {
      weightedSum += (long)sensorValues[i] * (i * 1000);
      totalValue += sensorValues[i];
    }
  }

  // Evitar división por cero
  if (totalValue == 0)
    return 0;

  return weightedSum / totalValue;
}
