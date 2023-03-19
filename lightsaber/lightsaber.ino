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

// #define USE_TIMER2
#define NUM_LEDS 104
#define LED_PIN 3
#define INIT_ADDR 1023
#define INIT_KEY 0
#define BLADE_COLOR 0

class Blade {
private:
  uint32_t blade_time, mpuTimer, hum_timer;
  const uint32_t blade_delay = 10;
  bool isAction = false;
  enum { On, Off } status;
  int blade_idx = 0;
  CRGB leds[NUM_LEDS];
  int colors_num;
  CRGB *colors;



  unsigned long ACC, GYR, COMP;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int gyroX, gyroY, gyroZ, accelX, accelY, accelZ;
  
  void bladeActivate(uint32_t loop_time);

  void bladeDeactivate(uint32_t loop_time);
public:
  // TMRpcm *tmrpcm;
  // MPU6050 accelgyro;

  int current_color;
  uint32_t sound_disable_timer;
  bool sound_disable;

  Blade();
  void action(uint32_t loop_time);//, TMRpcm &trmpcm);//, MPU6050 &accelgyro);
  void changeStatus();//TMRpcm &trmpcm);
  void changeColor();
  void setIsAction(bool value);
  int getStatus();
  void test();//TMRpcm &tmrpcm);//, MPU6050 &accelgyro);
};



Blade blade;


uint32_t start_time = millis();

bool flag = false;
int button_counter = 0;
uint32_t btnTimer = 0;

TMRpcm tmrpcm;
MPU6050 accelgyro;

void setup() {

  pinMode(2, INPUT_PULLUP);

  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  digitalWrite(A0, 0);
  digitalWrite(A1, 0);

  if (EEPROM.read(INIT_ADDR) != INIT_KEY) {
    EEPROM.write(INIT_ADDR, INIT_KEY);
    EEPROM.write(BLADE_COLOR, 0);
  }
  blade.current_color = EEPROM.read(BLADE_COLOR);


  //Serial.println("accel");

  
  

  //Wire.begin();
  Serial.begin(9600);
  //accelgyro.initialize();

  tmrpcm.speakerPin = 9;
  tmrpcm.volume(1);
  tmrpcm.setVolume(5);
  tmrpcm.quality(1);
  Serial.println(SD.begin(8));
  //blade.tmrpcm = &tmrpcm;

  tmrpcm.play("ON.wav");
  Wire.begin();
}

void loop() {
  uint32_t loop_time = millis();

  blade.action(loop_time);//, accelgyro);

  bool btnState = digitalRead(2);
  if (btnState && !flag) {
    if (loop_time - btnTimer > 100) {
      flag = true;
      button_counter++;
      btnTimer = millis();
      Serial.println("Button pressed");
    }
  }

  if (!btnState && flag && loop_time - btnTimer > 100) {
    flag = false;
    btnTimer = millis();
  }

  if (button_counter && !flag && loop_time - btnTimer > 200) {
    if (button_counter == 1) {
      //blade.changeStatus();
    }
    if (button_counter == 2) {
      blade.changeColor();
    }
    button_counter = 0;
  }


}







Blade::Blade()  {
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(255);
  blade_time = millis();
  status = Off;

  sound_disable = false;
  sound_disable_timer = millis();
  hum_timer = millis();

  //TODO: loading colors from SD
  colors_num = 6;
  colors = new CRGB[colors_num];
  colors[0] = CRGB::Blue;
  colors[1] = CRGB::Red;
  colors[2] = CRGB::Green;
  colors[3] = 0xFF6700; //yellow
  colors[4] = CRGB::White;
  colors[5] = 0x4f0070; //purple

  mpuTimer = millis();
}  
void Blade::action(uint32_t loop_time) {//, TMRpcm &tmrpcm){//, MPU6050 &accelgyro) {
  if (!tmrpcm.isPlaying() && loop_time - hum_timer > 2000 && !isAction && status == On) {
    //tmrpcm.loop(true);
    //tmrpcm->play("HUM.wav");
    hum_timer = millis();
    Serial.println("Hum");
  }
  if (sound_disable && loop_time - sound_disable_timer > 500) {
    tmrpcm.disable();
    sound_disable = false;
  }
  if (!isAction && status == On) {
    //test(tmrpcm, accelgyro);
  }
  if (isAction && status == On) bladeActivate(loop_time);
  if (isAction && status == Off) bladeDeactivate(loop_time);
}

void Blade::bladeActivate(uint32_t loop_time) {
  Serial.println("Blade");
    if (isAction && loop_time - blade_time >= blade_delay) {
      blade_time = loop_time;
      leds[blade_idx] = colors[current_color];
      leds[NUM_LEDS - 1 - blade_idx] = colors[current_color];
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
      leds[NUM_LEDS - 1 - blade_idx] = CRGB::Black;
      //blade_idx++;
      blade_idx--;
      if (blade_idx == -1) {
        setIsAction(false);
        //blade_idx = 0;
        sound_disable_timer = millis();
        sound_disable = true;
      }
      FastLED.show();
    }
}

void Blade::changeStatus() {//TMRpcm &tmrpcm) {
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
  current_color = (current_color + 1) % colors_num;
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

void Blade::test() {//TMRpcm &tmrpcm){//, MPU6050 &accelgyro) {
  //if (ls_state) {                                               // если меч включен
    if (millis() - mpuTimer > 500) {                            // каждые полмиллисекунды
      //accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);       // получить показания с IMU

      // найти абсолютное значение, разделить на 100
      gyroX = abs(gx / 100);
      gyroY = abs(gy / 100);
      gyroZ = abs(gz / 100);
      accelX = abs(ax / 100);
      accelY = abs(ay / 100);
      accelZ = abs(az / 100);

      // найти среднеквадратичное (сумма трёх векторов в общем)
      ACC = sq((long)accelX) + sq((long)accelY) + sq((long)accelZ);
      ACC = sqrt(ACC);
      GYR = sq((long)gyroX) + sq((long)gyroY) + sq((long)gyroZ);
      GYR = sqrt((long)GYR);
      //COMPL = ACC + GYR;

      if (ACC > 200) {      // если ускорение превысило порог
        // читаем название трека из PROGMEM
        tmrpcm.play("SK1.wav");               // воспроизвести звук удара
        Serial.println(ACC);
        //strike_flash();
        //if (!HUMmode)
        //  bzzTimer = millis() + strike_s_time[nowNumber] - FLASH_DELAY;
        //else
        //  humTimer = millis() - 9000 + strike_s_time[nowNumber] - FLASH_DELAY;
        //strike_flag = 1;
      }

      mpuTimer = micros();
    }
  //}

//Strike
  

/*
//swing
  if (GYR > 80 && (millis() - swing_timeout > 100) && HUMmode) {
    swing_timeout = millis();
    if (((millis() - swing_timer) > SWING_TIMEOUT) && swing_flag && !strike_flag) {
      if (GYR >= SWING_THR) {      // если ускорение превысило порог
        nowNumber = random(5);             // взять случайное число
        // читаем название трека из PROGMEM
        strcpy_P(BUFFER, (char*)pgm_read_word(&(swings[nowNumber])));
        tmrpcm.play(BUFFER);               // воспроизвести звук взмаха
        humTimer = millis() - 9000 + swing_time[nowNumber];
        swing_flag = 0;
        swing_timer = millis();
        swing_allow = 0;
      }
      if ((GYR > SWING_L_THR) && (GYR < SWING_THR)) {
        nowNumber = random(5);             // взять случайное число
        // читаем название трека из PROGMEM
        strcpy_P(BUFFER, (char*)pgm_read_word(&(swings_L[nowNumber])));
        tmrpcm.play(BUFFER);               // воспроизвести звук взмаха
        humTimer = millis() - 9000 + swing_time_L[nowNumber];
        swing_flag = 0;
        swing_timer = millis();
        swing_allow = 0;
      }
    }
  }
  */
}