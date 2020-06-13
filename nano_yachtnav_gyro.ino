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

bool Button1,Button2,Button3,Button4;
MPU6050 mpu6050(Wire);

bool AP; //состояние автопилота (включен/выключен) (-->LED13)
float Course; //курс по гиродатчику, долюен быть = 0 (ось z)

void setup() {
  wdt_disable();

  Serial.begin(9600);

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(false,100,100);

  pinMode(LED, OUTPUT); //D13 LED
  pinMode(LED, LOW);
    
  pinMode(OUT_RELAY1,OUTPUT);
  digitalWrite(OUT_RELAY1, LOW);
  pinMode(OUT_RELAY2,OUTPUT);
  digitalWrite(OUT_RELAY2, LOW);
  pinMode(OUT_RELAY3,OUTPUT);
  digitalWrite(OUT_RELAY3, LOW);
  pinMode(OUT_RELAY4,OUTPUT);  
  digitalWrite(OUT_RELAY4, LOW);

  pinMode(IN_BTN1,INPUT_PULLUP);
  digitalWrite(IN_BTN1,HIGH);
  pinMode(IN_BTN2,INPUT_PULLUP);
  digitalWrite(IN_BTN2,HIGH);
  pinMode(IN_BTN3,INPUT_PULLUP);
  digitalWrite(IN_BTN3,HIGH);
  pinMode(IN_BTN4,INPUT_PULLUP);
  digitalWrite(IN_BTN4,HIGH);

  if (!WDT_OFF) wdt_enable(WDTO_1S);
  wdt_reset();
}

void loop() {

  Button1 = ButtonPressed(IN_BTN1); //TURN LEFT SIDE
  Button2 = ButtonPressed(IN_BTN2); //TURN RIGHT SIDE
  Button3 = ButtonPressed(IN_BTN3); //JOY UP
  Button4 = ButtonPressed(IN_BTN4); //JOY DOWN

  digitalWrite( LED, AP ); // led 13 sign autopilot state

  digitalWrite( OUT_RELAY1, LOW );
  digitalWrite( OUT_RELAY2, LOW );

  if (Button1) { //TURN LEFT
    digitalWrite( OUT_RELAY1, HIGH );
    digitalWrite( OUT_RELAY2, LOW );
  } //Button1
  
  if (Button2) { // TURN RIGHT
      Serial.println("RIGHT");

    digitalWrite( OUT_RELAY1, LOW );
    digitalWrite( OUT_RELAY2, HIGH );
  } //Button2

  mpu6050.update();
  Course = mpu6050.getGyroAngleZ();

  if (Button3) {
    wdt_disable();
    mpu6050.calcGyroOffsets(false,100,100);
    mpu6050.resetAngleZ();
    AP = true;
    wdt_enable(WDTO_1S);
    wdt_reset();
  } //Button3

  if (Button4) {
    AP = false;
  }

  
  wdt_reset();

} //loop

bool ButtonPressed (uint8_t pin){

  bool ButtonState=false;
  if (digitalRead(pin)==LOW) {
    delay(10);
    if (digitalRead(pin)==LOW) ButtonState = true;
  }

  return (ButtonState);
}
