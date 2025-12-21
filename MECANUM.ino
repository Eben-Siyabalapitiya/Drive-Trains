// Lord of Robots - LoR Core V3 - AUG 3 2025
// Sample MiniBot Control Program with Bluetooth GamePad Interface

#include <Bluepad32.h>     // Gamepad Core
#include "esp_task_wdt.h"  // System stability
#include <ESP32Servo.h>    // Servo PWM Core
#include <FastLED.h>       // Addressable LED Core

////////////////////////////////////////////////////////////////////////////////////////////
//                            AUX Port Config                                             //
////////////////////////////////////////////////////////////////////////////////////////////
const uint8_t AUX_PINS[9] = { 0, 5, 18, 23, 19, 22, 21, 1, 3 };

////////////////////////////////////////////////////////////////////////////////////////////
//                            IO Port Config                                              //
////////////////////////////////////////////////////////////////////////////////////////////
const uint8_t IO_PINS[13] = { 0, 32, 25, 26, 27, 14, 12, 13, 15, 2, 4, 22, 21 };

////////////////////////////////////////////////////////////////////////////////////////////
//                         User Button and Switch Config                                  //
////////////////////////////////////////////////////////////////////////////////////////////
#define User_BTN_A 35
#define User_BTN_B 39
#define User_BTN_C 38
#define User_BTN_D 37
#define User_SW 36

////////////////////////////////////////////////////////////////////////////////////////////
//                      Input Voltage Monitor Config                                      //
////////////////////////////////////////////////////////////////////////////////////////////
#define VIN_SENSE 34
#define VOLT_SLOPE 0.0063492
#define VOLT_OFFSET 1.079

////////////////////////////////////////////////////////////////////////////////////////////
//                            LED Config                                                  //
////////////////////////////////////////////////////////////////////////////////////////////
#define LED_PIN 33
#define LED_COUNT 4
#define BRIGHTNESS 255
#define COLOR_ORDER GRB
#define CHIPSET WS2812B
CRGB leds[LED_COUNT];
uint8_t rainbowHue = 0;

////////////////////////////////////////////////////////////////////////////////////////////
//                            Internal features                                           //
////////////////////////////////////////////////////////////////////////////////////////////
#define WDT_TIMEOUT 3
Servo MotorOutput[13];  // only for mecanum drive now

void INIT_InternalFeatures() {
  Serial.begin(115200);
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);

  pinMode(User_BTN_A, INPUT);
  pinMode(User_BTN_B, INPUT);
  pinMode(User_BTN_C, INPUT);
  pinMode(User_BTN_D, INPUT);
  pinMode(User_SW, INPUT);
  pinMode(VIN_SENSE, INPUT);

  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, LED_COUNT);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();
  delay(100);
}

////////////////////////////////////////////////////////////////////////////////////////////
//                            Bluepad32 Config                                           //
////////////////////////////////////////////////////////////////////////////////////////////
ControllerPtr myController = nullptr;

void onConnectedController(ControllerPtr ctl) {
  if (!myController) {
    Serial.println("! GamePad connected !");
    myController = ctl;
    ctl->playDualRumble(0x00, 0xc0, 0xc0, 0xc0);
    ctl->setColorLED(0, 255, 0);
    BP32.enableNewBluetoothConnections(false);
    fill_solid(leds, LED_COUNT, CRGB(0, 255, 0));
    FastLED.show();
    delay(500);
  } else {
    Serial.println("Another controller tried to connect but is rejected");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (myController == ctl) {
    Serial.println("! GamePad disconnected !");
    myController = nullptr;
    fill_solid(leds, LED_COUNT, CRGB(255, 0, 0));
    FastLED.show();
    delay(1000);
  }
}

static unsigned long lastBatteryUpdate = 0;
void GamePad_BatteryMonitor() {
  if (millis() - lastBatteryUpdate > 1000) {
    int battery = myController->battery();
    if (battery == 0) myController->setColorLED(255, 0, 0);
    else if (battery <= 64) {
      myController->setColorLED(255, 0, 0);
      Serial.println("! GamePad Low Battery !");
      myController->playDualRumble(0x00, 0xc0, 0xc0, 0xc0);
      fill_solid(leds, LED_COUNT, CRGB(255, 0, 0));
      FastLED.show();
      delay(100);
    }
    else if (battery <= 128) myController->setColorLED(255, 255, 0);
    else myController->setColorLED(0, 255, 0);

    lastBatteryUpdate = millis();
  }
}

void INIT_BluetoothGamepad_PairMode() {
  if (!digitalRead(User_BTN_A) && !digitalRead(User_BTN_D)) {
    BP32.forgetBluetoothKeys();
    Serial.println("Gamepad Unpaired!");
    BP32.enableNewBluetoothConnections(true);
    BP32.setup(&onConnectedController, &onDisconnectedController);
    while (!(myController && myController->isConnected())) {
      esp_task_wdt_reset();
      fill_solid(leds, LED_COUNT, CRGB(0, 0, 255));
      FastLED.show();
      delay(100);
      fill_solid(leds, LED_COUNT, CRGB(255, 255, 255));
      FastLED.show();
      delay(100);
      BP32.update();
    }
    BP32.enableNewBluetoothConnections(false);
  } else BP32.setup(&onConnectedController, &onDisconnectedController);

  BP32.enableVirtualDevice(false);
}

////////////////////////////////////////////////////////////////////////////////////////////
//                            Input voltage / Battery monitor                             //
////////////////////////////////////////////////////////////////////////////////////////////
float Low_Batt_Scaler = 0.25; //Lower the number faster the drive train
unsigned long TriggerTime = 0;
bool Scaler_StepState = 0;
unsigned long Check_Period_TriggerTime = 0;

float LoRcore_BatteryMonitor(uint8_t cellCount, float perCellLowV = 3.0, bool DEBUG = true) {
  int vin_raw = analogRead(VIN_SENSE);
  float vin_voltage = (vin_raw * VOLT_SLOPE) + VOLT_OFFSET;
  float lowVoltageThreshold = cellCount * perCellLowV;

  if (millis() > Check_Period_TriggerTime && DEBUG) {
    Check_Period_TriggerTime = millis() + 500;
    Serial.printf("VIN: %.2f V, Threshold: %.2f V\n", vin_voltage, lowVoltageThreshold);
  }

  if (vin_voltage < lowVoltageThreshold) {
    Serial.printf("LOW Battery: %.2f V\n", vin_voltage);
    if (millis() > TriggerTime) {
      Low_Batt_Scaler = Scaler_StepState ? 0 : 0.25;
      if (!Scaler_StepState) {
        fill_solid(leds, LED_COUNT, CRGB(255, 0, 0));
        FastLED.show();
        delay(100);
      }
      Scaler_StepState = !Scaler_StepState;
      TriggerTime = millis() + 100;
    }
  } else {
    Low_Batt_Scaler = 1.0;
  }
  return vin_voltage;
}

////////////////////////////////////////////////////////////////////////////////////////////
//                            Power up Diagnostics                                        //
////////////////////////////////////////////////////////////////////////////////////////////
void Powerup_Diagnostics_LED() {
  Serial.print("BOOT Condition: ");
  if (esp_reset_reason() == ESP_RST_TASK_WDT) fill_solid(leds, LED_COUNT, CRGB(255, 255, 255));
  else if (esp_reset_reason() == ESP_RST_BROWNOUT) fill_solid(leds, LED_COUNT, CRGB(255, 255, 0));
  else if (esp_reset_reason() == ESP_RST_POWERON) fill_solid(leds, LED_COUNT, CRGB(0, 255, 0));
  else if (esp_reset_reason() == ESP_RST_SW) fill_solid(leds, LED_COUNT, CRGB(0, 0, 255));
  else if (esp_reset_reason() == ESP_RST_PANIC) fill_solid(leds, LED_COUNT, CRGB(255, 0, 0));
  else fill_solid(leds, LED_COUNT, CRGB(255, 0, 255));
  FastLED.show();
  delay(500);
  fill_solid(leds, LED_COUNT, CRGB(0, 0, 0));
  FastLED.show();
  delay(100);
}

////////////////////////////////////////////////////////////////////////////////////////////
//                               Motor Config                                            //
////////////////////////////////////////////////////////////////////////////////////////////
enum MotorType { N20Plus, CUSTOM };
struct MotorTypeConfig { MotorType type; float pwmFreq; int minPulseUs; int maxPulseUs; };
MotorTypeConfig motorTypeConfigs[] = { {N20Plus, 50, 800, 2200} };

void ConfigureMotorOutput(uint8_t slot, MotorType motorType) {
  float pwmFreq = 50;
  int minPulseUs = 1000;
  int maxPulseUs = 2000;
  for (auto &cfg : motorTypeConfigs) {
    if (cfg.type == motorType) { pwmFreq = cfg.pwmFreq; minPulseUs = cfg.minPulseUs; maxPulseUs = cfg.maxPulseUs; break; }
  }
  uint8_t pin = IO_PINS[slot];
  pinMode(pin, OUTPUT);
  MotorOutput[slot].setPeriodHertz(pwmFreq);
  MotorOutput[slot].attach(pin, minPulseUs, maxPulseUs);
  MotorOutput[slot].writeMicroseconds(1500);
}

////////////////////////////////////////////////////////////////////////////////////////////
//                            Initialize LoR Core                                         //
////////////////////////////////////////////////////////////////////////////////////////////
void INIT_LoRcore() {
  INIT_InternalFeatures();
  Powerup_Diagnostics_LED();
  INIT_BluetoothGamepad_PairMode();
}

////////////////////////////////////////////////////////////////////////////////////////////
//                            Setup LOOP                                                  //
////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  INIT_LoRcore();
  Serial.println("Motors Startup");
  for (int i = 1; i <= 12; i++) ConfigureMotorOutput(i, N20Plus);
  Serial.println("LoRcore V3 System Ready!");
}

////////////////////////////////////////////////////////////////////////////////////////////
//                            Main LOOP                                                   //
////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  esp_task_wdt_reset();
  BP32.update();

  if (myController && myController->isConnected()) {
    GamePad_BatteryMonitor();

    float rawForward = -myController->axisY();
    float rawStrafe  = -myController->axisX();
    float rawRotate  = -myController->axisRX();

    if (digitalRead(User_SW) == LOW) {
      rawForward = -rawForward;
      rawStrafe  = -rawStrafe;
      rawRotate  = -rawRotate;
    }

    float speedFactor = 2;
    rawForward *= Low_Batt_Scaler * speedFactor;
    rawStrafe  *= Low_Batt_Scaler * speedFactor;
    rawRotate  *= Low_Batt_Scaler * speedFactor;

    if (abs(rawForward) < 50) rawForward = 0;
    if (abs(rawStrafe)  < 50) rawStrafe  = 0;
    if (abs(rawRotate)  < 50) rawRotate  = 0;

    // Mecanum wheel mixing
    float fl = rawForward + rawStrafe + rawRotate;
    float fr = rawForward - rawStrafe - rawRotate;
    float rl = rawForward - rawStrafe + rawRotate;
    float rr = rawForward + rawStrafe - rawRotate;

    fl = constrain(fl, -512, 512);
    fr = constrain(fr, -512, 512);
    rl = constrain(rl, -512, 512);
    rr = constrain(rr, -512, 512);

    // Map to motor output
    MotorOutput[1].write(map(fl, -512, 512, 0, 180));
    MotorOutput[2].write(map(fr, -512, 512, 0, 180));
    MotorOutput[3].write(map(rl, -512, 512, 0, 180));
    MotorOutput[4].write(map(rr, -512, 512, 0, 180));

    fill_rainbow(leds, LED_COUNT, rainbowHue++, 20);
    FastLED.show();
    delay(50);
  } else {
    for (int i = 1; i <= 12; i++) MotorOutput[i].write(90);
    fill_solid(leds, LED_COUNT, CRGB(0, 80, 255));
    FastLED.show();
  }
}
