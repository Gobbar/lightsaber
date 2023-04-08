/*
 Регистратор данных с использованием SD карт

 Пример сохранения данных с аналоговых портов на SD карте.
 Данные будут сохраняться в файле в виде набора строк с разделителем полей в виде символа ","

 Схема подключения:
 * Аналоговые сенсоры подключаются к аналоговым пинам
 * Модуль SD карты подключен в SPI по стандартной схеме:
 ** MOSI - пин 11
 ** MISO - пин12
 ** CLK - пин 13
 ** CS - pin 8 #можно любой

#include <SPI.h>
#include <SD.h>

const int PIN_CHIP_SELECT = 4;
*/
#include <SD.h>
#include <TMRpcm.h>
#include <toneAC.h>

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#include <FastLED.h>

#include <EEPROM.h>

#define NUM_LEDS 52
#define LED_PIN 3
#define INIT_ADDR 1023
#define INIT_KEY 0
#define BLADE_COLOR 0

#define BLADE_DELAY 10

uint32_t blade_time, mpuTimer, sound_disable_timer, hum_timer, strike_timer;
int strike_delay = 0;
int strike_time[8] = {779, 563, 687, 702, 673, 661, 666, 635};
int strike_s_time[8] = {270, 167, 186, 250, 252, 255, 250, 238};
bool isAction = false;
enum { On, Off } status;
int blade_idx = 0;
CRGB leds[NUM_LEDS];
byte colors_num;
byte current_color;
CRGB *colors;
bool sound_disable;

TMRpcm tmrpcm;
MPU6050 accelgyro;

bool flag = false;
int button_counter = 0;
uint32_t btnTimer = 0;

void setup() {
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(255);
  
  blade_time = millis();
  status = Off;

  sound_disable = false;
  sound_disable_timer = millis();

  //TODO: loading colors from SD
  colors_num = 5;
  colors = new CRGB[colors_num];
  colors[0] = CRGB::Blue;
  colors[1] = CRGB::Red;
  colors[2] = CRGB::Green;
  colors[3] = 0xFF6700; //yellow
  colors[4] = CRGB::White;
  colors[5] = 0x4f0070; //purple

  mpuTimer = millis();

  pinMode(2, INPUT_PULLUP);

  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  digitalWrite(A0, 0);
  digitalWrite(A1, 0);

  if (EEPROM.read(INIT_ADDR) != INIT_KEY) {
    EEPROM.write(INIT_ADDR, INIT_KEY);
    EEPROM.write(BLADE_COLOR, 0);
  }
  current_color = EEPROM.read(BLADE_COLOR);

  // Serial.begin(9600);
  Wire.begin();
  accelgyro.initialize();
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  tmrpcm.speakerPin = 9;
  tmrpcm.volume(1);
  tmrpcm.setVolume(5);
  tmrpcm.quality(1);
  bool sdOK = SD.begin(8);
  // Serial.println(sdOK ? "SD OK" : "SD NOT OK");
}

void loop() {
  button_tick();

  if (!tmrpcm.isPlaying() && !isAction && status == On && millis() - hum_timer > 500) {
    tmrpcm.play("HUM.wav");
    hum_timer = millis();
  }
  if (sound_disable && millis() - sound_disable_timer > 500) {
    tmrpcm.disable();
    sound_disable = false;
  }

  bladeAnimate();
  mpuProcess();
}

void button_tick() {
  bool btnState = digitalRead(2);
  if (btnState && !flag) {
    if (millis() - btnTimer > 100) {
      flag = true;
      button_counter++;
      btnTimer = millis();
    }
  }

  if (!btnState && flag && millis() - btnTimer > 100) {
    flag = false;
    btnTimer = millis();
  }

  if (button_counter && !flag && millis() - btnTimer > 200) {
    if (button_counter == 2) {
      if (status == Off) {
        current_color = (current_color + 1) % colors_num;
        EEPROM.write(BLADE_COLOR, current_color);
      }
    }
    button_counter = 0;
  }

  if (button_counter && flag && millis() - btnTimer > 2000) {
    if (button_counter == 1) {
      changeStatus();
    }
    button_counter = 0;
  }

}

void bladeAnimate() {
  if (isAction && millis() - blade_time >= BLADE_DELAY) {
    CRGB blade_color = status == On ? colors[current_color] : CRGB::Black;
    int inc = status == On ? 1 : -1;
    
    blade_time = millis();
    leds[blade_idx] = blade_color;
    blade_idx += inc;
    if (blade_idx == NUM_LEDS) {
      isAction = false;
    }
    if (blade_idx == -1) {
      isAction = false;
      sound_disable_timer = millis();
      sound_disable = true;
    }
    FastLED.show();
  }
}

void changeStatus() {
  isAction = true;
  sound_disable = false;
  if (status == On) {
    status = Off;
    blade_idx = blade_idx == NUM_LEDS ? NUM_LEDS - 1 : blade_idx;
    tmrpcm.play("OFF.wav");
    return;
  }
  if (status == Off) {
    status = On;
    blade_idx = blade_idx == -1 ? 0 : blade_idx;
    tmrpcm.play("ON.wav");
    return;
  }
}



void mpuProcess() {
  if (!isAction && status == On) {
    if (millis() - mpuTimer > strike_delay) {
      unsigned long ACC;//, GYR, COMP;
      int16_t ax, ay, az;
      int16_t gx, gy, gz;
      // int gyroX, gyroY, gyroZ;
      int accelX, accelY, accelZ;
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);       // получить показания с IMU

      // найти абсолютное значение, разделить на 100
      // gyroX = abs(gx / 100);
      // gyroY = abs(gy / 100);
      // gyroZ = abs(gz / 100);
      accelX = abs(ax / 100);
      accelY = abs(ay / 100);
      accelZ = abs(az / 100);

      ACC = sq((long)accelX) + sq((long)accelY) + sq((long)accelZ);
      ACC = sqrt(ACC);
      // GYR = sq((long)gyroX) + sq((long)gyroY) + sq((long)gyroZ);
      // GYR = sqrt((long)GYR);
      //COMPL = ACC + GYR;

      strike_delay = 100;
      if (ACC >= 60) {
        tmrpcm.play("SK1.wav");
        strike_timer = millis();
        strike_delay = strike_time[0];
      }
      mpuTimer = millis();
    }
  }
}