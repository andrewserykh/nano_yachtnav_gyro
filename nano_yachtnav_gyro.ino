/* v2.1

  !!!Внимание!!! Если bootloader не optiboot, то wdt надо закомментировать, он не работает на стандартной прошивке
  При прошивке:
  Плата: Arduino Nano
  Инструменты->Процессор Atmega328P (Optiboot)

  Watchdog таймер
  Управление актуатором с джойстика ЛЕВО,ПРАВО

*/
#include <avr/wdt.h>
#include "gy521.h"
#include <Wire.h>

#define WDT_OFF         1     //1=Без WatchDog таймера (не optiboot)

#define BAUD            9600  //Скорость последовательного порта
#define LED             13    //Светоиод на Arduino
//D2  Прерывание GY-521 Гироскоп
#define OUT_RELAY1      3     //D3  Выходы на управление актуаторами через релейный модуль
#define OUT_RELAY2      4     //D4  на 4 реле
#define OUT_RELAY3      5     //D5
#define OUT_RELAY4      6     //D6
#define IN_BTN1         7     //D7  Входы на подключение джойстика
#define IN_BTN2         8     //D8
#define IN_BTN3         9     //D9
#define IN_BTN4         10    //D10

#define T_MS_AP         500  //интервал обработки автопилота
#define T_MS_RUDDER     2000 //интервал обновления управления румпелем
#define T_MS_OUT        1000 //длительность одного импульса на управление румпелем

bool Button1, Button2, Button3, Button4;
MPU6050 mpu6050(Wire);

bool AP; //состояние автопилота (включен/выключен) (-->LED13)
float Course; //курс по гиродатчику, долюен быть = 0 (ось z)

//- параметри ПИД-регулирования
long ms_ap; //таймер на автопилот
float P, I, D; //ПИД-регулятор
float Kp, Ki, Kd; //коэффициенты ПИД-регулирования
float OUT, OUT_H, OUT_L; //диапазон значений на выходе
float CYCLE, P1; //
float DEADZONE; //зона нечувствительности
bool MINUS; //при уходе от курса в отрицательную сторону
//---

//- параметры автоматического управления рулем
long ms_rudder; //таймер на автоматическое управление румпелем
long ms_out; //таймер на длительность выходного импульса
bool AP_OUT_ACTIVE; //происходит управление
float OUT_LIMIT; //при каком пороге OUT выдавать импульс на движение руля
//---

void setup() {
  wdt_disable();

  Serial.begin(9600);

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(false, 100, 100);

  pinMode(LED, OUTPUT); //D13 LED
  pinMode(LED, LOW);

  pinMode(OUT_RELAY1, OUTPUT);
  digitalWrite(OUT_RELAY1, LOW);
  pinMode(OUT_RELAY2, OUTPUT);
  digitalWrite(OUT_RELAY2, LOW);
  pinMode(OUT_RELAY3, OUTPUT);
  digitalWrite(OUT_RELAY3, LOW);
  pinMode(OUT_RELAY4, OUTPUT);
  digitalWrite(OUT_RELAY4, LOW);

  pinMode(IN_BTN1, INPUT_PULLUP);
  digitalWrite(IN_BTN1, HIGH);
  pinMode(IN_BTN2, INPUT_PULLUP);
  digitalWrite(IN_BTN2, HIGH);
  pinMode(IN_BTN3, INPUT_PULLUP);
  digitalWrite(IN_BTN3, HIGH);
  pinMode(IN_BTN4, INPUT_PULLUP);
  digitalWrite(IN_BTN4, HIGH);

  //настройки ПИД-регулирования
  Kp = 2;
  Ki = 1;
  Kd = 0.1;
  CYCLE = 0.1;
  DEADZONE = 20;
  OUT_H = 100;
  OUT_L = 0;
  OUT_LIMIT = 100; //выше какого значения OUT будет подан импульс на управление
  //---
  ms_rudder = ms_ap = ms_out = millis();

  if (!WDT_OFF) wdt_enable(WDTO_1S);
  wdt_reset();
}

void loop() {

  Button1 = ButtonPressed(IN_BTN1); //TURN LEFT SIDE
  Button2 = ButtonPressed(IN_BTN2); //TURN RIGHT SIDE
  Button3 = ButtonPressed(IN_BTN3); //JOY UP
  Button4 = ButtonPressed(IN_BTN4); //JOY DOWN

  digitalWrite( LED, AP ); // led 13 sign autopilot state

  if (!AP_OUT_ACTIVE) {
    digitalWrite( OUT_RELAY1, LOW );
    digitalWrite( OUT_RELAY2, LOW );
  }

  if (Button1) { //TURN LEFT
    digitalWrite( OUT_RELAY1, HIGH );
    digitalWrite( OUT_RELAY2, LOW );
  } //Button1

  if (Button2) { // TURN RIGHT
    digitalWrite( OUT_RELAY1, LOW );
    digitalWrite( OUT_RELAY2, HIGH );
  } //Button2

  mpu6050.update();
  Course = mpu6050.getGyroAngleZ();

  if (Button3) { //ДЖОЙСТИК ВВЕРХ - ВКЛЮЧЕНИЕ АВТОПИЛОТА
    digitalWrite( OUT_RELAY1, LOW ); //сброс текущего управления
    digitalWrite( OUT_RELAY2, LOW );
    wdt_disable();
    mpu6050.calcGyroOffsets(false, 100, 100);
    mpu6050.resetAngleZ();
    ms_ap = millis();
    AP = true;
    wdt_enable(WDTO_1S);
    wdt_reset();
  } //Button3

  if (Button4) { //ДЖОЙСТИК ВНИЗ - ВЫКЛЮЧЕНИЕ АВТОПИЛОТА
    digitalWrite( OUT_RELAY1, LOW ); //сброс текущего управления
    digitalWrite( OUT_RELAY2, LOW );
    AP_OUT_ACTIVE = false;
    AP = false;
  }

  if (AP) { //АВТОПИЛОТ ВКЛЮЧЕН

    //---ПИД-регулятор
    if (millis() - ms_ap > T_MS_AP) {
      if ( abs(Course) > DEADZONE ) {
        P = abs(Course) ;
      } //DEADZONE
      else {
        P = 0;
      } //DEADZONE
      MINUS = false;
      if (Course < 0) MINUS = true;
      I = (I + P * CYCLE);
      if (P == 0) {
        I = I * 0.98;
      }
      if (I > OUT_H) {
        I = OUT_H;
      }
      else if (-I > OUT_H) {
        I = - OUT_H;
      }
      D = ((P - P1) / CYCLE);
      P1 = P;
      OUT = (Kp * P) + (Ki * I) + (Kd * D);
      if (OUT > OUT_H) {
        OUT = OUT_H;
      }
      if (OUT < OUT_L) {
        OUT = OUT_L;
      }
      if (MINUS) OUT = -OUT;
      Serial.print(Course);
      Serial.print(" --> ");
      Serial.println(OUT);

      ms_ap = millis();
    } //ms_ap
    //---

    //-управление румпелем
    if (millis() - ms_rudder > T_MS_RUDDER) {
      
      if (AP_OUT_ACTIVE){ //есть сигнал на управление (дописать сброс сигнала по истечении времени)
        
        if (millis() - ms_out > T_MS_OUT) { //сброс выходного импульса по истечении его времени
            digitalWrite( OUT_RELAY1, LOW );
            digitalWrite( OUT_RELAY2, LOW );
            ms_out = millis();
        } //ms_out
        AP_OUT_ACTIVE = false;
        
      } else { //!AP_OUT_ACTIVE - в настоящий момент нет сигнала на управление
        if (abs(OUT) >= OUT_LIMIT) {
          Serial.print(OUT);
          Serial.println(" OUT ACTIVE");
          if (MINUS) {
            digitalWrite( OUT_RELAY1, HIGH );
            digitalWrite( OUT_RELAY2, LOW );
          } else {
            digitalWrite( OUT_RELAY1, LOW );
            digitalWrite( OUT_RELAY2, HIGH );
          } //MINUS
          AP_OUT_ACTIVE = true;
          ms_out = millis();
        } //OUT>=LIMIT
      }// AP_OUT_ACTIVE

      ms_rudder = millis();
    } //ms_rudder

  } //AP


  wdt_reset();

} //loop

bool ButtonPressed (uint8_t pin) {

  bool ButtonState = false;
  if (digitalRead(pin) == LOW) {
    delay(10);
    if (digitalRead(pin) == LOW) ButtonState = true;
  }

  return (ButtonState);
}
