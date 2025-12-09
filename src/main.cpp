#include <Arduino.h>
#include "pinout.h"
#include "credential.h"
#include <EEPROM.h>
#include <Ticker.h>

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// Variables (same as original)
int addr = 0;
volatile int lastPos;
volatile unsigned char isCalib = 0;
volatile int servoPwm;
volatile unsigned char sensor;
unsigned int sensorValue[8];
unsigned int sensorPID[8];
unsigned int black_value[8];
unsigned int white_value[8];
unsigned int compare_value[8];
int speed_run_forward;
int cnt = 0;
unsigned char pattern, start;
int line = 0;
int RememberLine = 0;
float kp;
int kd;
int mode = 0; // 0: manual, 1: auto
float obstacle_threshold = 5.0; // cm, for avoidance

// ESP32 timer
hw_timer_t *timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;

void speed_run(int speedDC_left, int speedDC_right)
{
  if (speedDC_left < 0) {
    ledcWrite(1, 255 + speedDC_left);
    digitalWrite(LEFT_MOTOR_IN1_PIN, HIGH);
  } else if (speedDC_left >= 0) {
    speedDC_left = speedDC_left;
    ledcWrite(1, speedDC_left);
    digitalWrite(LEFT_MOTOR_IN1_PIN, LOW);
  }
  if (speedDC_right < 0) {
    ledcWrite(2, 255 + speedDC_right);
    digitalWrite(RIGHT_MOTOR_IN1_PIN, HIGH);
  } else if (speedDC_right >= 0) {
    speedDC_right = speedDC_right;
    ledcWrite(2, speedDC_right);
    digitalWrite(RIGHT_MOTOR_IN1_PIN, LOW);
  }
}

void read_sensor()
{
  unsigned char temp = 0;
  unsigned int sum = 0;
  unsigned long avg = 0;
  int i, iP, iD;
  int iRet;
  sensorValue[0] = 4095 - analogRead(LINE_SENSOR_IN8);
  sensorValue[1] = 4095 - analogRead(LINE_SENSOR_IN7);
  sensorValue[2] = 4095 - analogRead(LINE_SENSOR_IN6);
  sensorValue[3] = 4095 - analogRead(LINE_SENSOR_IN5);
  sensorValue[4] = 4095 - analogRead(LINE_SENSOR_IN4);
  sensorValue[5] = 4095 - analogRead(LINE_SENSOR_IN3);
  sensorValue[6] = 4095 - analogRead(LINE_SENSOR_IN2);
  sensorValue[7] = 4095 - analogRead(LINE_SENSOR_IN1);
  for (int j = 0; j < 8; j++) {
    if (isCalib == 0) {
      if (sensorValue[j] < black_value[j])
        sensorValue[j] = black_value[j];
      if (sensorValue[j] > white_value[j])
        sensorValue[j] = white_value[j];
      sensorPID[j] = map(sensorValue[j], black_value[j], white_value[j], 0, 1000);
    }
    temp = temp << 1;
    if (sensorValue[j] > compare_value[j]) {
      temp |= 0x01;
    } else {
      temp &= 0xfe;
    }
    sensor = temp;
  }
  for (int j = 0; j < 8; j++) {
    avg += (long)(sensorPID[j]) * ((j)*1000);
    sum += sensorPID[j];
  }
  i = (int)((avg / sum) - 3500);
  kp = 1;
  kd = 3;
  iP = kp * i;
  iD = kd * (lastPos - i);
  iRet = (iP - iD);
  if ((iRet < -4000)) {
    iRet = 0;
  }
  servoPwm = iRet / 20; //30 68mm
 // servoPwm = iRet / //60 48mm
  lastPos = i;
}

void readEeprom() {
  EEPROM.begin(512);
  for (int i = 0; i < 8; i++) {
    compare_value[i] = EEPROM.read(i) * 4;
  }
}

void handleAndSpeed(int angle, int speed1) {
  int speedLeft;
  int speedRight;
  if ((speed1 + angle) > 255) {
    speed1 = 255 - angle;
  }
  if ((speed1 - angle) > 255) {
    speed1 = 255 + angle;
  }
  speedLeft = speed1 + angle;
  speedRight = speed1 - angle;
  speed_run(speedLeft, speedRight);
}

// Rest of the functions same as previous code
void runforwardline(int speed)
{
  switch (sensor) {
    case 0b00000000:
      handleAndSpeed(servoPwm, speed);
      break;
    case 0b00011000:
    case 0b00001000:
    case 0b00010000:
    case 0b00111000:
    case 0b00011100:
      line = 0;
      handleAndSpeed(servoPwm, speed);
      break;
    case 0b00111100:
    case 0b00111110:
    case 0b01111110:
      line = 0;
      handleAndSpeed(servoPwm, speed);
      break;
      ///////////////////////////////////////////////////////////////////////
    case 0b00001100:
    case 0b00000100:
    case 0b00001110:
    case 0b00011110:
 
      line = 1;
      handleAndSpeed(servoPwm, speed);
      break;
    case 0b00000110:
    case 0b00000010:
    case 0b00000111:
  
      line = 2;
      handleAndSpeed(servoPwm, speed);
      break;
    case 0b00000011:
    case 0b00000001:
    case 0b00001111:
    case 0b00011111:
    case 0b00111111:
 
      line = 3;
      handleAndSpeed(servoPwm, speed);
      break;

    //////
    case 0b00110000:
    case 0b00100000:
    case 0b01110000:
    case 0b01111000:
 
      line = -1;
      handleAndSpeed(servoPwm, speed);
      break;
    //////
    case 0b01100000:
    case 0b01000000:
    case 0b11100000:
      line = -2;
      handleAndSpeed(servoPwm, speed);
      break;
    //////
    case 0b11000000:
    case 0b10000000:
    case 0b11110000:
    case 0b11111000:
    case 0b11111100:
      line = -3;
      handleAndSpeed(servoPwm, speed);
      break;
    default:
      handleAndSpeed(servoPwm, speed);
      break;
  }
}

void updateLine() {
  for (int i = 0; i < 8; i++) {
      Serial.print(sensorValue[i]);
      Serial.print("  ");
    if (black_value[i] == 0) black_value[i] = 1100;
    if (sensorValue[i] < black_value[i]) black_value[i] = sensorValue[i];
    if (sensorValue[i] > white_value[i]) white_value[i] = sensorValue[i];
    compare_value[i] = (black_value[i] + white_value[i]) / 2;
  }
  Serial.println();
}

float getDistance() {
  digitalWrite(ULTRA_SONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRA_SONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRA_SONIC_TRIG_PIN, LOW);
  float duration = pulseIn(ULTRA_SONIC_ECHO_PIN, HIGH);
  return (duration * 0.0343) / 2;
}

//-----------------------------------------------------------//

unsigned char sensorMask(unsigned char mask) {
  return (sensor & mask);
}

void IRAM_ATTR onTimer() {
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  read_sensor();
  cnt++;
}

// Blynk virtual pin handlers
BLYNK_WRITE(V0) { // Switch for mode: 0 manual, 1 auto
  mode = param.asInt();
}

BLYNK_WRITE(V1) { // Joystick for manual control
  if (mode == 0) {
    int x = param[0].asInt(); // -127 to 127
    int y = param[1].asInt(); // -127 to 127
    int speed = (y * 255) / 127;
    int turn = (x * 255) / 127;
    int left = speed + turn;
    int right = speed - turn;
    if (left > 255) left = 255;
    if (left < -255) left = -255;
    if (right > 255) right = 255;
    if (right < -255) right = -255;
    speed_run(left, right);
  }
}

// Blynk virtual pin handlers
BLYNK_WRITE(V2) { // Switch for mode: 0 not Calibrate, 1 Calibrate
  isCalib = param.asInt();
}

void setup() {
  pinMode(LEFT_MOTOR_IN1_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1_PIN, OUTPUT);
  pinMode(ULTRA_SONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRA_SONIC_ECHO_PIN, INPUT);
  digitalWrite(LEFT_MOTOR_IN1_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_IN1_PIN, LOW);

  speed_run(0, 0);
  pattern = 10;
  start = 0;
  readEeprom();
  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, SSID_WIFI, PASS_WIFI);

  // Timer setup for ESP32 (1ms interrupt)
  timerSemaphore = xSemaphoreCreateBinary();
  timer = timerBegin(0, 80, true); // 80MHz / 80 = 1MHz, for 1us tick
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000, true); // 1ms
  timerAlarmEnable(timer);

  isCalib = 1;

  // PWM setup
  ledcSetup(1, 5000, 8); // Channel 1 for LPWM
  ledcAttachPin(LEFT_MOTOR_IN2_PIN, 1);
  ledcSetup(2, 5000, 8); // Channel 2 for RPWM
  ledcAttachPin(RIGHT_MOTOR_IN2_PIN, 2);
}

void loop() {
  Blynk.run();
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE) {
    // Processed in ISR
  }

  if (mode == 0) { // Manual mode via Blynk
    // Control handled in BLYNK_WRITE(V1)
    // In calibration, if needed
    if (isCalib) updateLine();
  } else { // Auto mode
    // Learning/avoidance: In auto, check obstacle
    float dist = getDistance();
    if (dist < obstacle_threshold && pattern == 11) {
      // Simple avoidance: stop, reverse briefly, turn right
      speed_run(-80, 80); // turn right
      delay(500);
      speed_run(80, -80); // turn left
      delay(500);
      speed_run(80, -80); // turn left
      delay(500);
      // Then continue
    }

    switch (pattern) {  
      case 10:
        if (cnt >= 50) {
          pattern = 11;
          cnt = 0;
          break;
        }
        runforwardline(speed_run_forward);
        break;
      case 11:
        if (sensorMask(0x01) == 0x01) {
          RememberLine = 1;
          cnt = 0;
        } else if (sensorMask(0x80) == 0x80) {
          RememberLine = -1;
          cnt = 0;
        }
        if (cnt > 500) RememberLine = 0;
        if (sensor == 0b00000000) {
          if (RememberLine != 0) {
            if (RememberLine == 1) {
              handleAndSpeed(40, speed_run_forward);
            } else if (RememberLine == -1) {
              handleAndSpeed(-40, speed_run_forward);
            }
          } else {
            speed_run(0, 0);
          }
          break;
        } else runforwardline(speed_run_forward);  
        if (sensorMask(0b00111100) != 0b00000000) {
          RememberLine = 0;
        }
        break;
      // Other cases as in original
      case 12:
        if (RememberLine == 1) {
          speed_run(100, -40);
          pattern = 21;
          break;
        } else if (RememberLine == -1) {
          speed_run(-40, 100);
          pattern = 31;
          break;
        } else {
          pattern = 11;
          break;
        }
      case 21:
        speed_run(100, -40);
        if (sensorMask(0xff) != 0) {
          speed_run(60, 60 / 2);
          pattern = 22;
        }
        break;
      case 22:
        speed_run(60, 60 / 2);
        if (sensorMask(0xfc) != 0) {
          pattern = 11;
        }
        break;
      case 31:
        speed_run(-40, 60);
        if (sensorMask(0xff) != 0) {
          speed_run(60 / 2, 60);
          pattern = 32;
        }
        break;
      case 32:
        speed_run(60 / 2, 60);
        if (sensorMask(0x3f) != 0) {
          pattern = 11;
        }
        break;
      case 100:
        speed_run(0, 0);
        break;
      default:
        pattern = 11;
        break;
    }
  }
}