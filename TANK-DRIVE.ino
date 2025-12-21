// ====================== LoR Core V3 - MiniBot Control ====================== //
// Sample Bluetooth GamePad Interface
// Pins and motor slots labeled for easy changes (Front, Back, Left, Right, etc.)

////////////////////////////////////////////////////////////////////////////////
// -------------------- User Config / Pin Mapping ----------------------------
////////////////////////////////////////////////////////////////////////////////

// ---------------- AUX Ports ----------------
const uint8_t AUX_PINS[9] = { 0, 5, 18, 23, 19, 22, 21, 1, 3 }; // slot 0 imaginary

// ---------------- IO Ports -----------------
const uint8_t IO_PINS[13] = { 0, 32, 25, 26, 27, 14, 12, 13, 15, 2, 4, 22, 21 }; // slot 0 imaginary

// ---------------- User Buttons ----------------
#define User_BTN_A 35
#define User_BTN_B 39
#define User_BTN_C 38
#define User_BTN_D 37
#define User_SW    36

// ---------------- Voltage Monitor ----------------
#define VIN_SENSE   34
#define VOLT_SLOPE  0.0063492
#define VOLT_OFFSET 1.079

// ---------------- LEDs ----------------
#define LED_PIN     33
#define LED_COUNT   4
#define BRIGHTNESS  255
#define COLOR_ORDER GRB
#define CHIPSET     WS2812B

CRGB leds[LED_COUNT];
uint8_t rainbowHue = 0;

// ---------------- Servo / Motors ----------------
Servo MotorOutput[13]; // Motor slots 1-12 usable
#define WDT_TIMEOUT 3

// Predefined motor types
enum MotorType { MG90_CR, MG90_Degree, N20Plus, STD_SERVO, Victor_SPX, Talon_SRX, SPARK_MAX, CUSTOM };
struct MotorTypeConfig {
    MotorType type;
    float pwmFreq;
    int minPulseUs;
    int maxPulseUs;
    float inputMin;
    float inputMax;
};

MotorTypeConfig motorTypeConfigs[] = {
    { MG90_CR, 50, 500, 2500, -1, 1 },
    { MG90_Degree, 50, 500, 2500, 1, 180 },
    { N20Plus, 50, 800, 2200, -1, 1 },
    { Victor_SPX, 50, 1000, 2000, -1, 1 },
    { Talon_SRX, 50, 1000, 2000-1, 1 },
    { STD_SERVO, 50, 1000, 2000-1, 1 },
    { SPARK_MAX, 50, 1000, 2000-1, 1 }
};

// ====================== Internal Features ====================== //
void INIT_InternalFeatures() {
    Serial.begin(115200);
    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(NULL);

    // Inputs
    pinMode(User_BTN_A, INPUT);
    pinMode(User_BTN_B, INPUT);
    pinMode(User_BTN_C, INPUT);
    pinMode(User_BTN_D, INPUT);
    pinMode(User_SW, INPUT);
    pinMode(VIN_SENSE, INPUT);

    // LEDs
    FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, LED_COUNT);
    FastLED.setBrightness(BRIGHTNESS);
    FastLED.clear();
    FastLED.show();
    delay(100);
}

// ====================== GamePad Setup ====================== //
ControllerPtr myController = nullptr;

void onConnectedController(ControllerPtr ctl) {
    if (!myController) {
        myController = ctl;
        ctl->playDualRumble(0x00, 0xc0, 0xc0, 0xc0);
        ctl->setColorLED(0, 255, 0);
        BP32.enableNewBluetoothConnections(false);
        fill_solid(leds, LED_COUNT, CRGB(0, 255, 0));
        FastLED.show();
        delay(500);
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    if (myController == ctl) {
        myController = nullptr;
        fill_solid(leds, LED_COUNT, CRGB(255, 0, 0));
        FastLED.show();
        delay(1000);
    }
}

// ====================== Motor Config ====================== //
void ConfigureMotorOutput(uint8_t slot, MotorType motorType, int startupPositionDeg = 90) {
    float pwmFreq = 50;
    int minPulseUs = 1000;
    int maxPulseUs = 2000;

    for (auto &cfg : motorTypeConfigs) {
        if (cfg.type == motorType) {
            pwmFreq = cfg.pwmFreq;
            minPulseUs = cfg.minPulseUs;
            maxPulseUs = cfg.maxPulseUs;
            break;
        }
    }

    uint8_t pin = IO_PINS[slot];
    pinMode(pin, OUTPUT);
    MotorOutput[slot].setPeriodHertz(pwmFreq);
    MotorOutput[slot].attach(pin, minPulseUs, maxPulseUs);
    MotorOutput[slot].writeMicroseconds(1500);
    Serial.printf("Motor slot %d configured on pin %d type=%d start=%d°\n",
                  slot, pin, motorType, startupPositionDeg);
}

// ====================== Battery Monitor ====================== //
float Low_Batt_Scaler = 0.25;
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
        if (millis() > TriggerTime) {
            Low_Batt_Scaler = Scaler_StepState ? 0 : 0.25;
            fill_solid(leds, LED_COUNT, CRGB(255, 0, 0));
            FastLED.show();
            delay(100);
            Scaler_StepState = !Scaler_StepState;
            TriggerTime = millis() + 100;
        }
    } else {
        Low_Batt_Scaler = 1.0;
    }
    return vin_voltage;
}

// ====================== Powerup Diagnostics ====================== //
void Powerup_Diagnostics_LED() {
    if (esp_reset_reason() == ESP_RST_TASK_WDT)
        fill_solid(leds, LED_COUNT, CRGB(255, 255, 255)); // White = Watchdog
    else if (esp_reset_reason() == ESP_RST_BROWNOUT)
        fill_solid(leds, LED_COUNT, CRGB(255, 255, 0)); // Yellow = Brownout
    else if (esp_reset_reason() == ESP_RST_POWERON)
        fill_solid(leds, LED_COUNT, CRGB(0, 255, 0));   // Green = Normal Start
    else if (esp_reset_reason() == ESP_RST_SW)
        fill_solid(leds, LED_COUNT, CRGB(0, 0, 255));   // Blue = Software Reset
    else if (esp_reset_reason() == ESP_RST_PANIC)
        fill_solid(leds, LED_COUNT, CRGB(255, 0, 0));   // Red = Panic
    else
        fill_solid(leds, LED_COUNT, CRGB(255, 0, 255)); // Purple = Unknown

    FastLED.show();
    delay(500);
    fill_solid(leds, LED_COUNT, CRGB(0, 0, 0));
    FastLED.show();
}

// ====================== LoR Core Init ====================== //
void INIT_LoRcore() {
    INIT_InternalFeatures();
    Powerup_Diagnostics_LED();
    // INIT_BluetoothGamepad_PairMode(); // optional
}

// ====================== Setup ====================== //
void setup() {
    INIT_LoRcore();
    Serial.println("Motors Startup");

    // Drivetrain Motors (example mapping, change as needed)
    ConfigureMotorOutput(1, N20Plus, 90); // Front Left
    ConfigureMotorOutput(2, N20Plus, 90); // Back Left
    ConfigureMotorOutput(3, N20Plus, 90); // Front Right
    ConfigureMotorOutput(4, N20Plus, 90); // Back Right

    // Other servos
    ConfigureMotorOutput(5, N20Plus, 90);
    ConfigureMotorOutput(6, MG90_Degree, 90); // Example manipulator
    ConfigureMotorOutput(7, MG90_Degree, 90); // Example manipulator
    ConfigureMotorOutput(8, N20Plus, 90);
    ConfigureMotorOutput(9, N20Plus, 90);
    ConfigureMotorOutput(10, N20Plus, 90);
    ConfigureMotorOutput(11, N20Plus, 90);
    ConfigureMotorOutput(12, N20Plus, 90);

    Serial.println("LoRcore V3 System Ready!");
}

// ====================== Loop ====================== //
void loop() {
    esp_task_wdt_reset(); // Watchdog
    BP32.update();        // Gamepad input

    // Example: Drivetrain Arcade Drive
    if (myController && myController->isConnected()) {
        int moveValue = -myController->axisRX();
        int turnValue = myController->axisY();

        int currentLeft  = moveValue + turnValue;
        int currentRight = moveValue - turnValue;

        if (abs(currentLeft) < 50) currentLeft = 0;
        if (abs(currentRight) < 50) currentRight = 0;

        int MappedLeft  = constrain(map(currentLeft, -512, 512, 0, 180), 0, 180);
        int MappedRight = constrain(map(currentRight, -512, 512, 0, 180), 0, 180);

        MotorOutput[1].write(MappedLeft);  // Front Left
        MotorOutput[2].write(MappedLeft);  // Back Left
        MotorOutput[3].write(MappedRight); // Front Right
        MotorOutput[4].write(MappedRight); // Back Right

        // LEDs
        fill_rainbow(leds, LED_COUNT, rainbowHue++, 20);
        FastLED.show();
        delay(50);
    } else {
        for (int i = 1; i <= 12; i++) MotorOutput[i].write(90);
        fill_solid(leds, LED_COUNT, CRGB(0, 80, 255));
        FastLED.show();
    }
}
