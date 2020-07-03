/* v3.1  ATMEGA328P

  !!!Внимание!!! Если bootloader не optiboot, то wdt надо закомментировать, он не работает на стандартной прошивке
  При прошивке:
  Плата: Arduino Nano
  Инструменты->Процессор Atmega328P (Optiboot)

  Watchdog таймер
  Управление актуатором с джойстика ЛЕВО,ПРАВО, включение, выключение автопилота ВВЕРХ,ВНИЗ

	NEXTION описание страницы AP:
 
	* AP1 *
	tHDG.txt - Текущий курс
  tkp.txt; + = id8  ; - = id11 ;  65 03 08+ ; 65 03 0b-
  tki.txt; + = id9  ; - = id12 ;  65 03 09+ ; 65 03 0c-
  tkd.txt; + = id10 ; - = id13 ;  65 03 0a+ ; 65 03 0d-

  * AP2 *
  tdz.txt - DEADZONE  ; 65 04 09+ ; 65 04 0c-
  tol.txt - OUT_LIMIT ; 65 04 0a+ ; 65 04 0d-
  tcy.txt - CYCLE     ; 65 04 0b+ ; 65 04 0e-

  * RUDDER *
  LEFT =  65 05 04 00 FF FF FF
  RIGHT = 65 05 05
 

*/
#include <avr/wdt.h>
#include "gy521.h"
#include <Wire.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

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
#define T_MS_HMI        3000 //интервал посылки сообщения на HMI

unsigned long ms_hmi;

SoftwareSerial SSerial(11, 12); // RX, TX
char SSerialIn[256]; //буфер приема по serial портам
byte SSerialInLen; //заполнение буфера
long SSerialMillisRcv; //прием по 485 порту (отсрочка на прием всего пакета)

bool ThisMyData; //означает что данные предназначались этому ПЛК

bool Button1, Button2, Button3, Button4;
MPU6050 mpu6050(Wire);

bool MAN; //ручное управление
bool AP; //состояние автопилота (включен/выключен) (-->LED13)
float Course; //курс по гиродатчику, долюен быть = 0 (ось z)

//- параметри ПИД-регулирования
long ms_ap; //таймер на автопилот
long T_MS_OUT; //длительность одного импульса на управление румпелем
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
char SerialIn[64]; //буфер приема по serial портам
byte SerialInLen; //заполнение буфера
long SerialMillisRcv; //прием по 485 порту (отсрочка на прием всего пакета)
//---

void setup() {
  wdt_disable();

  Serial.begin(9600);
  SSerial.begin(9600);

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

  OUT_H = 100;
  OUT_L = 0;

  Kp = EEPROM_float_read(0);
  Ki = EEPROM_float_read(4);
  Kd = EEPROM_float_read(8);
  CYCLE = EEPROM_float_read(12);
  DEADZONE = EEPROM_float_read(16);
  OUT_LIMIT = EEPROM_float_read(20);
  T_MS_OUT = EEPROM_float_read(24);  

  if( isnan(Kp) ){
    //начальные настройки ПИД-регулирования, закомментить если уже записаны в EEPROM
    Kp = 2;
    Ki = 1;
    Kd = 0.1;
    CYCLE = 0.1;
    DEADZONE = 5;
    OUT_LIMIT = 80; //выше какого значения OUT будет подан импульс на управление
    T_MS_OUT = 1000; //длительность импульса управления на румпель
    EEPROM_float_write(0, Kp);
    EEPROM_float_write(4, Ki);
    EEPROM_float_write(8, Kd);
    EEPROM_float_write(12, CYCLE);
    EEPROM_float_write(16, DEADZONE);
    EEPROM_float_write(20, OUT_LIMIT);
    EEPROM_float_write(24, T_MS_OUT);
    delay(200);
  } //isnan
  
  Serial.println("Started.");
  Serial.print("Kp = ");
  Serial.println(Kp);
  Serial.print("Ki = ");
  Serial.println(Ki);
  Serial.print("Kd = ");
  Serial.println(Kd);
  
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
    digitalWrite( OUT_RELAY1, HIGH ); //сброс текущего управления
    digitalWrite( OUT_RELAY2, HIGH );
    wdt_disable();
    mpu6050.calcGyroOffsets(false, 100, 100);
    mpu6050.resetAngleZ();
    ms_ap = millis();
    AP = true;
    wdt_enable(WDTO_1S);
    wdt_reset();
    digitalWrite( OUT_RELAY1, LOW ); //сброс текущего управления
    digitalWrite( OUT_RELAY2, LOW );    
  } //Button3

  if (Button4) { //ДЖОЙСТИК ВНИЗ - ВЫКЛЮЧЕНИЕ АВТОПИЛОТА
    digitalWrite( OUT_RELAY1, LOW ); //сброс текущего управления
    digitalWrite( OUT_RELAY2, LOW );
    AP_OUT_ACTIVE = false;
    AP = false;
  }

  if (MAN) {
      if (millis() - ms_out > T_MS_OUT) {
        digitalWrite( OUT_RELAY1, LOW );
        digitalWrite( OUT_RELAY2, LOW );
        MAN = false;
      }
  }

  if (AP) { //АВТОПИЛОТ ВКЛЮЧЕН
    digitalWrite( LED, HIGH ); // led 13 sign autopilot state

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

      ms_ap = millis();
    } //ms_ap
    //---

    //-управление румпелем
    if (millis() - ms_rudder > T_MS_RUDDER) {
      Serial.println(Course);
      if (AP_OUT_ACTIVE){ //есть сигнал на управление (дописать сброс сигнала по истечении времени)
        if (millis() - ms_out > T_MS_OUT) { //сброс выходного импульса по истечении его времени
            digitalWrite( OUT_RELAY1, LOW );
            digitalWrite( OUT_RELAY2, LOW );
            ms_out = millis();
        } //ms_out
        AP_OUT_ACTIVE = false;
        
      } else { //!AP_OUT_ACTIVE - в настоящий момент нет сигнала на управление
        if (abs(OUT) >= OUT_LIMIT) {
          //Serial.print(OUT);
          //Serial.println(" OUT ACTIVE");
          if (MINUS) {
            digitalWrite( OUT_RELAY1, LOW );
            digitalWrite( OUT_RELAY2, HIGH );
          } else {
            digitalWrite( OUT_RELAY1, HIGH );
            digitalWrite( OUT_RELAY2, LOW );
          } //MINUS
          AP_OUT_ACTIVE = true;
          ms_out = millis();
        } //OUT>=LIMIT
      }// AP_OUT_ACTIVE

      ms_rudder = millis();
    } //ms_rudder

  } //AP

//-serial recieve
  while (SSerial.available()) {
    digitalWrite(LED,HIGH);
    char SSerialChar = (char)SSerial.read();
    SSerialIn[SSerialInLen] = SSerialChar;
    SSerialInLen++;
    SSerialMillisRcv = millis(); //для отсрочки обработки
    digitalWrite(LED,LOW);
  } //while available

//---Обработка команд с HMI панели
  if (SSerialInLen > 0 && (millis() - SSerialMillisRcv > 200)) {
    ThisMyData = false;
    //_______________id элемента_____________id страницы____
    if (SSerialIn[2]==0x08 && SSerialIn[1]==0x03) { Kp=Kp+0.1; ThisMyData = true; } //ADJ+
    if (SSerialIn[2]==0x0b && SSerialIn[1]==0x03) { Kp=Kp-0.1; ThisMyData = true; } //ADJ-

    if (SSerialIn[2]==0x09 && SSerialIn[1]==0x03) { Ki=Ki+0.1; ThisMyData = true; } //ADJ+
    if (SSerialIn[2]==0x0c && SSerialIn[1]==0x03) { Ki=Ki-0.1; ThisMyData = true; }//ADJ-

    if (SSerialIn[2]==0x0a && SSerialIn[1]==0x03) { Kd=Kd+0.1; ThisMyData = true; } //ADJ+
    if (SSerialIn[2]==0x0d && SSerialIn[1]==0x03) { Kd=Kd-0.1; ThisMyData = true; } //ADJ-

    if (SSerialIn[2]==0x09 && SSerialIn[1]==0x04) { DEADZONE=DEADZONE+1; ThisMyData = true; } //ADJ+
    if (SSerialIn[2]==0x0c && SSerialIn[1]==0x04) { DEADZONE=DEADZONE-1; ThisMyData = true; } //ADJ-

    if (SSerialIn[2]==0x0a && SSerialIn[1]==0x04) { OUT_LIMIT=OUT_LIMIT+1; ThisMyData = true; } //ADJ+
    if (SSerialIn[2]==0x0d && SSerialIn[1]==0x04) { OUT_LIMIT=OUT_LIMIT-1; ThisMyData = true; } //ADJ-

    if (SSerialIn[2]==0x0b && SSerialIn[1]==0x04) { CYCLE=CYCLE+1; ThisMyData = true; } //ADJ+
    if (SSerialIn[2]==0x0e && SSerialIn[1]==0x04) { CYCLE=CYCLE-1; ThisMyData = true; } //ADJ-

    if (SSerialIn[2]==0x04 && SSerialIn[1]==0x05) { //RUDDER LEFT
      ThisMyData = true;
      MAN = true;
      digitalWrite( OUT_RELAY1, LOW );
      digitalWrite( OUT_RELAY2, HIGH );     
      ms_out = millis();
    }
    if (SSerialIn[2]==0x05 && SSerialIn[1]==0x05) { //RUDDER RIGHT
      ThisMyData = true;
      MAN = true;
      digitalWrite( OUT_RELAY1, HIGH );
      digitalWrite( OUT_RELAY2, LOW );
      ms_out = millis();
    }

    SSerialInLen = 0;    

    if (ThisMyData) {
      digitalWrite(LED,HIGH);
      HmiPutFloat("tkp.txt=",Kp);
      HmiPutFloat("tki.txt=",Ki);
      HmiPutFloat("tkd.txt=",Kd);
      HmiPutFloat("tdz.txt=",DEADZONE);
      HmiPutFloat("tol.txt=",OUT_LIMIT);
      HmiPutFloat("tcy.txt=",CYCLE);
      
      EEPROM_float_write(0, Kp);
      EEPROM_float_write(4, Ki);
      EEPROM_float_write(8, Kd);
      EEPROM_float_write(12, CYCLE);
      EEPROM_float_write(16, DEADZONE);
      EEPROM_float_write(20, OUT_LIMIT);
      EEPROM_float_write(24, T_MS_OUT);      
      digitalWrite(LED,LOW);
      ThisMyData = false;
      ms_hmi = millis();
    }

  } //SerialInLen>0

  if (millis() - ms_hmi > T_MS_HMI) {
    HmiPutFloat("tHDG.txt=",Course); //вывод текущего курса на hmi панель
    HmiPutFloat("tkp.txt=",Kp);
    HmiPutFloat("tki.txt=",Ki);
    HmiPutFloat("tkd.txt=",Kd);
    HmiPutFloat("tdz.txt=",DEADZONE);
    HmiPutFloat("tol.txt=",OUT_LIMIT);
    HmiPutFloat("tcy.txt=",CYCLE);
    ms_hmi = millis();
  }

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

void EEPROM_float_write(int addr, float val) // запись в ЕЕПРОМ
{ 
  byte *x = (byte *)&val;
  for(byte i = 0; i < 4; i++) EEPROM.write(i+addr, x[i]);
}
 
float EEPROM_float_read(int addr) // чтение из ЕЕПРОМ
{   
  byte x[4];
  for(byte i = 0; i < 4; i++) x[i] = EEPROM.read(i+addr);
  float *y = (float *)&x;
  return y[0];
}

//отправка произвольной команды на HMI
void HmiCmd(String Object)
{
  SSerial.print(Object);
  SSerial.write(0xff);
  SSerial.write(0xff);
  SSerial.write(0xff);
}
//отправка строки в элемент (передает "")
void HmiPutFloat(String Object,float Value)
{
  SSerial.print(Object);
  SSerial.write(0x22);
  SSerial.print(Value);
  SSerial.write(0x22);
  SSerial.write(0xff);
  SSerial.write(0xff);
  SSerial.write(0xff);
}
