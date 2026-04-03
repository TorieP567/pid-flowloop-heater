// Legacy reference sketch.
// Active two-MCU main-box firmware now lives in firmware/main_box_uno/ and firmware/main_box_nano_bridge/.

#include <math.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <QuickPID.h>
#include "radio_payload.h"

// Main heater controller:
// - no local buttons
// - no OLED
// - no local MAX6675
// - receives raw temperature + setpoint from the remote box over nRF24L01
// - runs QuickPID with adaptive thresholding for two SSR outputs

const uint8_t HEAT_PIN_MAIN = 7;
const uint8_t HEAT_PIN_RES  = 6;

const uint8_t RADIO_CE_PIN  = 9;
const uint8_t RADIO_CSN_PIN = 10;

const float EMA_ALPHA               = 0.30f;
const float D_FILTER_ALPHA          = 0.25f;
const float MIN_SETPOINT_C          = 20.0f;
const float MAX_SETPOINT_ALLOWED_C  = 39.0f;
const float MAX_TEMP_C              = 41.0f;
const float WARN_TEMP_C             = 39.0f;

const unsigned long CONTROL_INTERVAL_MS = 500UL;
const unsigned long LOG_INTERVAL_MS     = 500UL;
const unsigned long WINDOW_SIZE_MS      = 1000UL;
const unsigned long RADIO_TIMEOUT_MS    = 1500UL;

struct TankControl {
  const char* label;
  uint8_t heatPin;

  float rawTemp;
  float filteredTemp;
  float setpoint;

  float pidInput;
  float pidSetpoint;
  float pidOutput;
  float outputPct;

  float prevTemp;
  float dTempFilt;

  float kpBase;
  float kiBase;
  float kdBase;

  float kpEff;
  float kiEff;
  float kdEff;

  bool sensorValid;
  bool heaterOn;
  unsigned long windowStart;
};

TankControl tanks[2];

QuickPID mainPid(&tanks[0].pidInput, &tanks[0].pidOutput, &tanks[0].pidSetpoint);
QuickPID resPid(&tanks[1].pidInput, &tanks[1].pidOutput, &tanks[1].pidSetpoint);
QuickPID* const pids[2] = {&mainPid, &resPid};

RF24 radioHw(RADIO_CE_PIN, RADIO_CSN_PIN);
const byte RADIO_ADDRESS[6] = "00001";

bool radioInitOk = false;
bool havePacket = false;
uint8_t lastPacketCounter = 0;
unsigned long bootMs = 0;
unsigned long lastPacketMs = 0;
unsigned long lastControlRunMs = 0;
unsigned long lastLogMs = 0;

float clampFloat(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

bool isValidTemp(float t) {
  return (!isnan(t) && t > -50.0f && t < 500.0f);
}

bool isRadioStale(unsigned long nowMs) {
  return (!havePacket) || ((nowMs - lastPacketMs) > RADIO_TIMEOUT_MS);
}

void initTank(int tank, const char* label, uint8_t heatPin, float defaultSetpoint,
              float kp, float ki, float kd) {
  TankControl& t = tanks[tank];
  t.label = label;
  t.heatPin = heatPin;
  t.rawTemp = NAN;
  t.filteredTemp = defaultSetpoint;
  t.setpoint = defaultSetpoint;
  t.pidInput = defaultSetpoint;
  t.pidSetpoint = defaultSetpoint;
  t.pidOutput = 0.0f;
  t.outputPct = 0.0f;
  t.prevTemp = defaultSetpoint;
  t.dTempFilt = 0.0f;
  t.kpBase = kp;
  t.kiBase = ki;
  t.kdBase = kd;
  t.kpEff = kp;
  t.kiEff = ki;
  t.kdEff = kd;
  t.sensorValid = false;
  t.heaterOn = false;
  t.windowStart = 0;
}

void initPid(int tank) {
  QuickPID& pid = *pids[tank];
  TankControl& t = tanks[tank];

  pid.SetControllerDirection(QuickPID::Action::direct);
  pid.SetProportionalMode(QuickPID::pMode::pOnError);
  pid.SetDerivativeMode(QuickPID::dMode::dOnMeas);
  pid.SetAntiWindupMode(QuickPID::iAwMode::iAwCondition);
  pid.SetTunings(t.kpBase, t.kiBase, t.kdBase);
  pid.SetOutputLimits(0.0f, 100.0f);
  pid.SetSampleTimeUs(CONTROL_INTERVAL_MS * 1000UL);
  pid.SetMode(QuickPID::Control::automatic);
  pid.Reset();
}

void initRadio() {
  radioInitOk = radioHw.begin();
  if (!radioInitOk) {
    Serial.println("Radio init failed");
    return;
  }

  radioHw.setPALevel(RF24_PA_LOW);
  radioHw.setDataRate(RF24_250KBPS);
  radioHw.setChannel(108);
  radioHw.setAutoAck(false);
  radioHw.openReadingPipe(1, RADIO_ADDRESS);
  radioHw.startListening();
}

void applyRemoteTank(int tank, const RadioTankPayload& remote, bool valid) {
  TankControl& t = tanks[tank];
  bool hadValidSensor = t.sensorValid;

  t.setpoint = clampFloat(remote.setpointC, MIN_SETPOINT_C, MAX_SETPOINT_ALLOWED_C);

  if (!valid || !isValidTemp(remote.rawTempC)) {
    t.rawTemp = NAN;
    t.sensorValid = false;
    return;
  }

  t.rawTemp = remote.rawTempC;
  t.sensorValid = true;

  if (!hadValidSensor || !isValidTemp(t.filteredTemp)) {
    t.filteredTemp = t.rawTemp;
    t.prevTemp = t.filteredTemp;
  } else {
    t.filteredTemp = EMA_ALPHA * t.rawTemp + (1.0f - EMA_ALPHA) * t.filteredTemp;
  }
}

void readRadio() {
  if (!radioInitOk) return;

  bool gotPacket = false;
  RadioPayload payload;

  while (radioHw.available()) {
    radioHw.read(&payload, sizeof(payload));
    gotPacket = true;
  }

  if (!gotPacket) return;

  unsigned long now = millis();
  havePacket = true;
  lastPacketMs = now;
  lastPacketCounter = payload.counter;

  applyRemoteTank(0, payload.main, (payload.validMask & RADIO_VALID_MAIN) != 0);
  applyRemoteTank(1, payload.res,  (payload.validMask & RADIO_VALID_RES)  != 0);
}

bool tankHasFault(int tank, unsigned long nowMs) {
  const TankControl& t = tanks[tank];
  if (isRadioStale(nowMs)) return true;
  if (!t.sensorValid) return true;
  if (!isValidTemp(t.rawTemp)) return true;
  if (t.filteredTemp >= MAX_TEMP_C) return true;
  return false;
}

void applyAdaptiveThresholding(int tank, float dtSec) {
  TankControl& t = tanks[tank];

  float dTemp = (t.filteredTemp - t.prevTemp) / dtSec;
  t.dTempFilt = (1.0f - D_FILTER_ALPHA) * t.dTempFilt + D_FILTER_ALPHA * dTemp;

  float error = t.setpoint - t.filteredTemp;

  float kpEff = t.kpBase;
  float kiEff = t.kiBase;
  float kdEff = t.kdBase;

  if (error > 3.0f) {
    kpEff *= 1.20f;
    kiEff *= 0.35f;
    kdEff *= 0.45f;
  } else if (error > 1.0f) {
    kpEff *= 1.05f;
    kiEff *= 0.80f;
    kdEff *= 0.75f;
  } else if (error >= -0.10f) {
    kpEff *= 0.85f;
    kiEff *= 1.05f;
    kdEff *= 1.25f;
  } else {
    kpEff *= 0.55f;
    kiEff  = 0.0f;
    kdEff *= 1.80f;
  }

  if (t.filteredTemp > (t.setpoint - 0.20f) && t.dTempFilt > 0.04f) {
    kpEff *= 0.75f;
    kiEff *= 0.50f;
    kdEff *= 1.40f;
  }

  t.kpEff = kpEff;
  t.kiEff = kiEff;
  t.kdEff = kdEff;
  t.prevTemp = t.filteredTemp;

  pids[tank]->SetTunings(kpEff, kiEff, kdEff);
}

void resetTankOutput(int tank) {
  TankControl& t = tanks[tank];
  t.pidOutput = 0.0f;
  t.outputPct = 0.0f;
  t.dTempFilt = 0.0f;
  t.prevTemp = t.filteredTemp;
  pids[tank]->Reset();
  pids[tank]->SetOutputSum(0.0f);
}

void runControl() {
  unsigned long now = millis();
  if (now - lastControlRunMs < CONTROL_INTERVAL_MS) return;

  float dtSec;
  if (lastControlRunMs == 0) dtSec = CONTROL_INTERVAL_MS / 1000.0f;
  else dtSec = (now - lastControlRunMs) / 1000.0f;
  if (dtSec <= 0.0f) dtSec = CONTROL_INTERVAL_MS / 1000.0f;
  lastControlRunMs = now;

  for (int i = 0; i < 2; ++i) {
    TankControl& t = tanks[i];
    QuickPID& pid = *pids[i];

    t.pidInput = t.filteredTemp;
    t.pidSetpoint = t.setpoint;

    if (tankHasFault(i, now)) {
      resetTankOutput(i);
      continue;
    }

    applyAdaptiveThresholding(i, dtSec);
    pid.Compute();

    float error = t.setpoint - t.filteredTemp;
    if (error < -0.30f) {
      float outputSum = pid.GetOutputSum() * 0.96f;
      if (outputSum < 0.0f) outputSum = 0.0f;
      pid.SetOutputSum(outputSum);
    }

    t.outputPct = clampFloat(t.pidOutput, 0.0f, 100.0f);
  }
}

void driveHeaters() {
  unsigned long now = millis();

  for (int i = 0; i < 2; ++i) {
    TankControl& t = tanks[i];

    while (now - t.windowStart >= WINDOW_SIZE_MS) {
      t.windowStart += WINDOW_SIZE_MS;
    }

    unsigned long onTime = (unsigned long)(WINDOW_SIZE_MS * (t.outputPct / 100.0f));
    bool interlock = tankHasFault(i, now);
    bool shouldBeOn = (!interlock) && ((now - t.windowStart) < onTime);

    if (shouldBeOn != t.heaterOn) {
      t.heaterOn = shouldBeOn;
      digitalWrite(t.heatPin, t.heaterOn ? HIGH : LOW);
    }
  }
}

const char* getStatusText(int tank, unsigned long nowMs) {
  const TankControl& t = tanks[tank];
  if (isRadioStale(nowMs)) return "RADIO";
  if (!t.sensorValid || !isValidTemp(t.rawTemp)) return "SENS";
  if (t.filteredTemp >= MAX_TEMP_C) return "CUT";
  if (t.filteredTemp >= WARN_TEMP_C) return "WRN";
  return "OK";
}

void logData() {
  unsigned long now = millis();
  if (now - lastLogMs < LOG_INTERVAL_MS) return;
  lastLogMs = now;

  Serial.print("PKT:");
  Serial.print(lastPacketCounter);
  Serial.print(",AGE:");
  Serial.print(havePacket ? (now - lastPacketMs) : 9999UL);

  for (int i = 0; i < 2; ++i) {
    TankControl& t = tanks[i];
    Serial.print(",");
    Serial.print(t.label);
    Serial.print("Raw:");
    Serial.print(t.rawTemp, 2);
    Serial.print(",");
    Serial.print(t.label);
    Serial.print("Temp:");
    Serial.print(t.filteredTemp, 2);
    Serial.print(",");
    Serial.print(t.label);
    Serial.print("SP:");
    Serial.print(t.setpoint, 2);
    Serial.print(",");
    Serial.print(t.label);
    Serial.print("Out:");
    Serial.print(t.outputPct, 1);
    Serial.print(",");
    Serial.print(t.label);
    Serial.print("Kp:");
    Serial.print(t.kpEff, 2);
    Serial.print(",");
    Serial.print(t.label);
    Serial.print("Ki:");
    Serial.print(t.kiEff, 3);
    Serial.print(",");
    Serial.print(t.label);
    Serial.print("Kd:");
    Serial.print(t.kdEff, 2);
    Serial.print(",");
    Serial.print(t.label);
    Serial.print("St:");
    Serial.print(getStatusText(i, now));
  }

  Serial.println();
}

void setup() {
  Serial.begin(115200);

  pinMode(HEAT_PIN_MAIN, OUTPUT);
  pinMode(HEAT_PIN_RES, OUTPUT);
  digitalWrite(HEAT_PIN_MAIN, LOW);
  digitalWrite(HEAT_PIN_RES, LOW);

  initTank(0, "MAIN", HEAT_PIN_MAIN, 37.0f, 10.0f, 0.04f, 16.0f);
  initTank(1, "RES",  HEAT_PIN_RES,  37.0f, 12.0f, 0.05f, 18.0f);

  SPI.begin();
  initRadio();
  initPid(0);
  initPid(1);

  bootMs = millis();
  lastControlRunMs = bootMs;
  lastLogMs = bootMs;

  for (int i = 0; i < 2; ++i) {
    tanks[i].windowStart = bootMs;
  }

  Serial.println("V03_QuickPID receiver ready");
}

void loop() {
  readRadio();
  runControl();
  driveHeaters();
  logData();
}
