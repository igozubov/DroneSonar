#include <Arduino.h>
#include <common/mavlink.h>

#define PIN_TRIG 9
#define PIN_ECHO 8

long duration, cm;

void setup() {
  // Инициализируем взаимодействие по последовательному порту
  Serial.begin (9600);
  //Определяем вводы и выводы
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
}

void loop() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_TRIG, HIGH);
  // Выставив высокий уровень сигнала, ждем около 10 микросекунд. В этот момент датчик будет посылать сигналы с частотой 40 КГц.
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  //  Время задержки акустического сигнала на эхолокаторе.
  duration = pulseIn(PIN_ECHO, HIGH);
  // Теперь осталось преобразовать время в расстояние
  cm = (duration / 2) / 29.1;
  Serial.print("Расстояние до объекта: ");
  Serial.print(cm);
  Serial.println(" см.");
  // Задержка между измерениями для корректной работы скеча
  delay(50);

  MavLink_receive();
}

void MavLink_receive() {
  mavlink_message_t msg;
  mavlink_status_t status;

}