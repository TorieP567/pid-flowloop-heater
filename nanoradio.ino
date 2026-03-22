#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <max6675.h>

// ============================================================
// Remote Nano test
// 2x MAX6675 + 3 buttons + ST7789 LCD + nRF24 detect
// ============================================================

// ---------------- LCD ----------------
// ST7789 shares hardware SPI with the radio on D11/D13.
// LCD gets its own CS/DC/RST.
#define TFT_CS   A1
#define TFT_DC   A2
#define TFT_RST  A3

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// ---------------- Radio ----------------
RF24 radio(9, 10);   // CE, CSN
bool radioOk = false;

// ---------------- MAX6675 ----------------
const int thermoSCK      = 5;   // shared clock
const int thermoCS_main  = 6;
const int thermoSO_main  = 7;

const int thermoCS_res   = 8;
const int thermoSO_res   = A0;

MAX6675 thermocoupleMain(thermoSCK, thermoCS_main, thermoSO_main);
MAX6675 thermocoupleRes(thermoSCK, thermoCS_res, thermoSO_res);

// ---------------- Buttons ----------------
const int BTN_UP_PIN   = 2;
const int BTN_SET_PIN  = 3;
const int BTN_DOWN_PIN = 4;

// ---------------- Constants ----------------
const int TANK_COUNT = 2;
const char* TANK_LABELS[TANK_COUNT] = {"MAIN", "RES"};

const float EMA_ALPHA               = 0.30f;
const float TREND_THRESHOLD         = 0.20f;
const int   TREND_BUFFER_SIZE       = 5;
const float MIN_SETPOINT_C          = 20.0f;
const float MAX_SETPOINT_ALLOWED_C  = 39.0f;
const float BUTTON_SP_STEP_C        = 0.1f;

const unsigned long SENSOR_INTERVAL     = 250UL;
const unsigned long DISPLAY_INTERVAL    = 250UL;
const unsigned long BUTTON_DEBOUNCE_MS  = 40UL;
const unsigned long DOUBLE_CLICK_MS     = 350UL;
const unsigned long RADIO_CHECK_MS      = 2000UL;

// ---------------- State ----------------
struct TankState {
  float rawTemp;
  float filteredTemp;
  float setpoint;
  float tempHistory[TREND_BUFFER_SIZE];
  int historyIndex;
  int historyCount;
};

TankState tanks[TANK_COUNT];

struct ButtonState {
  int pin;
  bool stableState;
  bool lastReading;
  unsigned long lastChangeMs;
};

ButtonState btnUp   = {BTN_UP_PIN,   HIGH, HIGH, 0};
ButtonState btnSet  = {BTN_SET_PIN,  HIGH, HIGH, 0};
ButtonState btnDown = {BTN_DOWN_PIN, HIGH, HIGH, 0};

int selectedTank = 0;
bool setMode = false;
bool pendingSetSingleClick = false;
unsigned long lastSetClickMs = 0;

unsigned long lastSensorRead = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastRadioCheck = 0;

// ============================================================
// Helpers
// ============================================================

float clampFloat(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

bool isValidTemp(float t) {
  return (!isnan(t) && t > -50.0f && t < 500.0f);
}

float readThermocoupleC(int tank) {
  if (tank == 0) return thermocoupleMain.readCelsius();
  return thermocoupleRes.readCelsius();
}

void setTankSetpoint(int tank, float newSP) {
  if (tank < 0 || tank >= TANK_COUNT) return;
  tanks[tank].setpoint = clampFloat(newSP, MIN_SETPOINT_C, MAX_SETPOINT_ALLOWED_C);
}

char getTrendSymbol(int tank) {
  if (tanks[tank].historyCount < TREND_BUFFER_SIZE) return '-';

  int oldestIdx = tanks[tank].historyIndex;
  int newestIdx = (tanks[tank].historyIndex + TREND_BUFFER_SIZE - 1) % TREND_BUFFER_SIZE;
  float delta = tanks[tank].tempHistory[newestIdx] - tanks[tank].tempHistory[oldestIdx];

  if (delta > TREND_THRESHOLD) return '^';
  if (delta < -TREND_THRESHOLD) return 'v';
  return '-';
}

// ============================================================
// Init
// ============================================================

void initButtons() {
  pinMode(BTN_UP_PIN, INPUT_PULLUP);
  pinMode(BTN_SET_PIN, INPUT_PULLUP);
  pinMode(BTN_DOWN_PIN, INPUT_PULLUP);
}

void initDisplay() {
  tft.init(240, 320);      // portrait init
  tft.setRotation(1);      // landscape
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(false);
}

void initSensors() {
  delay(500);  // MAX6675 startup

  for (int i = 0; i < TANK_COUNT; i++) {
    tanks[i].setpoint = 37.0f;
    tanks[i].rawTemp = readThermocoupleC(i);

    if (isValidTemp(tanks[i].rawTemp)) {
      tanks[i].filteredTemp = tanks[i].rawTemp;
    } else {
      tanks[i].filteredTemp = tanks[i].setpoint;
    }

    tanks[i].historyIndex = 0;
    tanks[i].historyCount = 0;
    for (int j = 0; j < TREND_BUFFER_SIZE; j++) {
      tanks[i].tempHistory[j] = tanks[i].filteredTemp;
    }
  }
}

void initRadio() {
  // Make sure LCD CS is inactive before talking to radio
  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);

  radioOk = radio.begin();
  if (radioOk) {
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(108);
  }
}

// ============================================================
// Buttons
// ============================================================

bool updateButtonPressed(ButtonState &btn) {
  bool reading = digitalRead(btn.pin);
  unsigned long now = millis();

  if (reading != btn.lastReading) {
    btn.lastChangeMs = now;
    btn.lastReading = reading;
  }

  if ((now - btn.lastChangeMs) > BUTTON_DEBOUNCE_MS) {
    if (reading != btn.stableState) {
      btn.stableState = reading;
      if (btn.stableState == LOW) return true;
    }
  }
  return false;
}

void handleButtons() {
  unsigned long now = millis();

  if (updateButtonPressed(btnSet)) {
    if (pendingSetSingleClick && (now - lastSetClickMs <= DOUBLE_CLICK_MS)) {
      pendingSetSingleClick = false;
      selectedTank = (selectedTank + 1) % TANK_COUNT;
    } else {
      pendingSetSingleClick = true;
      lastSetClickMs = now;
    }
  }

  if (pendingSetSingleClick && (now - lastSetClickMs > DOUBLE_CLICK_MS)) {
    pendingSetSingleClick = false;
    setMode = !setMode;
  }

  if (setMode) {
    if (updateButtonPressed(btnUp)) {
      setTankSetpoint(selectedTank, tanks[selectedTank].setpoint + BUTTON_SP_STEP_C);
    }
    if (updateButtonPressed(btnDown)) {
      setTankSetpoint(selectedTank, tanks[selectedTank].setpoint - BUTTON_SP_STEP_C);
    }
  }
}

// ============================================================
// Sensors / radio / display
// ============================================================

void readSensors() {
  for (int i = 0; i < TANK_COUNT; i++) {
    float raw = readThermocoupleC(i);
    tanks[i].rawTemp = raw;

    if (isValidTemp(raw)) {
      tanks[i].filteredTemp = EMA_ALPHA * raw + (1.0f - EMA_ALPHA) * tanks[i].filteredTemp;

      tanks[i].tempHistory[tanks[i].historyIndex] = tanks[i].filteredTemp;
      tanks[i].historyIndex = (tanks[i].historyIndex + 1) % TREND_BUFFER_SIZE;
      if (tanks[i].historyCount < TREND_BUFFER_SIZE) tanks[i].historyCount++;
    }
  }
}

void refreshRadioStatus() {
  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  radioOk = radio.begin();
  if (radioOk) {
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(108);
  }
}

void drawTankBlock(int tank, int y) {
  tft.setTextSize(2);
  tft.setCursor(8, y);
  tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);

  if (selectedTank == tank) {
    tft.print(setMode ? "* " : "> ");
  } else {
    tft.print("  ");
  }

  tft.print(TANK_LABELS[tank]);

  tft.setCursor(8, y + 28);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.print("T: ");

  if (!isValidTemp(tanks[tank].rawTemp)) {
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
    tft.print("ERR");
  } else {
    tft.print(tanks[tank].filteredTemp, 1);
    tft.print(" C ");
    tft.print(getTrendSymbol(tank));
  }

  tft.setCursor(8, y + 56);
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.print("SP: ");
  tft.print(tanks[tank].setpoint, 1);
  tft.print(" C");

  tft.setCursor(8, y + 84);
  float err = tanks[tank].setpoint - tanks[tank].filteredTemp;
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  tft.print("ERR: ");
  if (err >= 0) tft.print("+");
  tft.print(err, 1);
}

void updateDisplay() {
  // Make radio CS inactive before display SPI
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);

  tft.fillScreen(ST77XX_BLACK);

  tft.setTextSize(2);
  tft.setCursor(8, 8);
  tft.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
  tft.print("REMOTE TEST");

  drawTankBlock(0, 40);
  drawTankBlock(1, 160);

  tft.drawFastHLine(0, 148, 320, ST77XX_BLUE);

  tft.setTextSize(2);
  tft.setCursor(8, 286);
  if (radioOk) {
    tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    tft.print("RADIO: OK");
  } else {
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
    tft.print("RADIO: FAIL");
  }
}

void printSerialDebug() {
  Serial.print("MAIN T=");
  Serial.print(tanks[0].filteredTemp, 2);
  Serial.print(" SP=");
  Serial.print(tanks[0].setpoint, 2);

  Serial.print(" | RES T=");
  Serial.print(tanks[1].filteredTemp, 2);
  Serial.print(" SP=");
  Serial.print(tanks[1].setpoint, 2);

  Serial.print(" | RADIO=");
  Serial.println(radioOk ? "OK" : "FAIL");
}

// ============================================================
// Setup / loop
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(300);

  initButtons();
  initDisplay();
  initSensors();
  initRadio();

  updateDisplay();
  Serial.println("Remote LCD/MAX6675/button/radio test ready");
}

void loop() {
  unsigned long now = millis();

  handleButtons();

  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;
    readSensors();
    printSerialDebug();
  }

  if (now - lastRadioCheck >= RADIO_CHECK_MS) {
    lastRadioCheck = now;
    refreshRadioStatus();
  }

  if (now - lastDisplayUpdate >= DISPLAY_INTERVAL) {
    lastDisplayUpdate = now;
    updateDisplay();
  }
}