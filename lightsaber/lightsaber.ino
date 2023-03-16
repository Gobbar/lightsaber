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
#include <EEPROM.h>

#include <FastLED.h>
#define NUM_LEDS 104
#define LED_PIN 3
#define INIT_ADDR 1023
#define INIT_KEY 0
#define BLADE_COLOR 0

class Blade {
private:
  uint32_t blade_time;
  const uint32_t blade_delay = 10;
  bool isAction = false;
  enum { On, Off } status;
  int blade_idx = 0;
  CRGB leds[NUM_LEDS];
#define COLORS_NUM 5
  CRGB colors[COLORS_NUM] = { CRGB::Blue, CRGB::Red, CRGB::Green, 0xFF6700, CRGB::White };
  TMRpcm tmrpcm;

  
  void bladeActivate(uint32_t loop_time);

  void bladeDeactivate(uint32_t loop_time);
public:
  int current_color;
  uint32_t sound_disable_timer;
  bool sound_disable;

  Blade();
  void action(uint32_t loop_time);
  void changeStatus();
  void changeColor();
  void setIsAction(bool value);
  int getStatus();
};

Blade::Blade()  {
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(255);
  blade_time = millis();
  status = Off;

  sound_disable = false;
  sound_disable_timer = millis();

  tmrpcm.speakerPin = 9;
  tmrpcm.volume(1);
  tmrpcm.setVolume(5);
  tmrpcm.quality(1);
  SD.begin(8);
}  
void Blade::action(uint32_t loop_time) {
  if (!tmrpcm.isPlaying() && !isAction && status == On) {
    //tmrpcm.loop(true);
    tmrpcm.play("HUM.wav");
  }
  if (sound_disable && loop_time - sound_disable_timer > 500) {
    tmrpcm.disable();
    sound_disable = false;
  }
  if (!isAction) return;
  if (status == On) bladeActivate(loop_time);
  if (status == Off) bladeDeactivate(loop_time);
}

void Blade::bladeActivate(uint32_t loop_time) {
    if (isAction && loop_time - blade_time >= blade_delay) {
      blade_time = loop_time;
      leds[blade_idx] = colors[current_color];
      leds[103 - blade_idx] = colors[current_color];
      blade_idx++;
      if (blade_idx == NUM_LEDS / 2) {
        setIsAction(false);
      }
      FastLED.show();
    }
}
void Blade::bladeDeactivate(uint32_t loop_time) {
    if (isAction && loop_time - blade_time >= blade_delay) {
      blade_time = loop_time;
      leds[blade_idx] = CRGB::Black;
      leds[103 - blade_idx] = CRGB::Black;
      blade_idx--;
      if (blade_idx == -1) {
        setIsAction(false);
        sound_disable_timer = millis();
        sound_disable = true;
      }
      FastLED.show();
    }
}

void Blade::changeStatus() {
  if (status == On) {
    status = Off;
    setIsAction(true);
    sound_disable = false;
    blade_idx = blade_idx == NUM_LEDS / 2 ? NUM_LEDS / 2 - 1 : blade_idx;
    tmrpcm.play("OFF.wav");
    return;
  }
  if (status == Off) {
    status = On;
    setIsAction(true);
    sound_disable = false;
    blade_idx = blade_idx == -1 ? 0 : blade_idx;
    tmrpcm.play("ON.wav");
    return;
  }
}

void Blade::changeColor() { 
  current_color = (current_color + 1) % COLORS_NUM;
  EEPROM.write(BLADE_COLOR, current_color);
}

void Blade::setIsAction(bool value) {
  isAction = value;
  // if (!value) {
  //   if (status == On) toneAC(5);
  //   else noToneAC();
  // }
}

int Blade::getStatus() { return status; }

Blade blade;

uint32_t start_time = millis();

bool flag = false;
int button_counter = 0;
uint32_t btnTimer = 0;

void setup() {
  Serial.begin(9600);
  pinMode(2, INPUT_PULLUP);

  if (EEPROM.read(INIT_ADDR) != INIT_KEY) {
    EEPROM.write(INIT_ADDR, INIT_KEY);
    EEPROM.write(BLADE_COLOR, 0);
  }
  blade.current_color = EEPROM.read(BLADE_COLOR);
}

void loop() {
  uint32_t loop_time = millis();
  
  blade.action(loop_time);

  bool btnState = digitalRead(2);
  if (btnState && !flag) {
    if (loop_time - btnTimer > 100) {
      flag = true;
      button_counter++;
      btnTimer = millis();
    }
  }

  if (!btnState && flag && loop_time - btnTimer > 100) {
    flag = false;
    btnTimer = millis();
  }

  if (button_counter && loop_time - btnTimer > 200) {
    if (button_counter == 1) {
      blade.changeStatus();
    }
    if (button_counter == 2) {
      blade.changeColor();
    }
    flag = false;
    button_counter = 0;
  }


}