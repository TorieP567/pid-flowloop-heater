// =============================================================================
// MAIN BOX UNO R4 MINIMA
// Dual-tank heater controller with adaptive/self-tuning logic.
//
// Architecture:
//   remote box --nRF24--> main-box Nano bridge --I2C--> this Uno R4
//
// Responsibilities on this board:
//   - final heater control authority for both SSR outputs
//   - EMA filtering, trend tracking, online model identification, periodic retune
//   - adaptive control law derived from the legacy dual-tank controller
//   - heater interlocks for radio loss, local bridge loss, invalid sensors, overtemp
//
// Pin map:
//   D7  -> SSR outlet box #1 (MAIN tank heater)
//   D6  -> SSR outlet box #2 (RES tank heater)
//   SDA -> I2C link from main-box Nano A4
//   SCL -> I2C link from main-box Nano A5
// =============================================================================

#include <math.h>
#include <string.h>
#include <Wire.h>

#include "../shared/system_packets.h"

namespace {

constexpr uint8_t TANK_COUNT = 2;

constexpr uint8_t HEAT_PIN_MAIN = 7;
constexpr uint8_t HEAT_PIN_RES = 6;
const uint8_t HEAT_PINS[TANK_COUNT] = {HEAT_PIN_MAIN, HEAT_PIN_RES};
const char* const TANK_LABELS[TANK_COUNT] = {"MAIN", "RES"};

constexpr double DEFAULT_SETPOINT_C = 37.0;
constexpr double MIN_SETPOINT_C = 20.0;
constexpr double MAX_SETPOINT_ALLOWED_C = 39.0;
constexpr double WARN_TEMP_C = 39.0;
constexpr double MAX_TEMP_C = 41.0;
constexpr double EMA_ALPHA = 0.30;
constexpr double D_FILTER_ALPHA = 0.25;
constexpr double RLS_FORGET = 0.995;
constexpr double INTEGRAL_MIN = -25.0;
constexpr double INTEGRAL_MAX = 100.0;
constexpr double TREND_THRESHOLD_C = 0.20;
constexpr double AT_SETPOINT_BAND_C = 1.0;

constexpr double MIN_REASONABLE_TEMP_C = -20.0;
constexpr double MAX_REASONABLE_TEMP_C = 120.0;

constexpr uint8_t TREND_BUFFER_SIZE = 5;

constexpr unsigned long WINDOW_SIZE_MS = 1000UL;
constexpr unsigned long CONTROL_INTERVAL_MS = 500UL;
constexpr unsigned long LOG_INTERVAL_MS = 500UL;
constexpr unsigned long RETUNE_INTERVAL_MS = 5000UL;
constexpr unsigned long WARMUP_TIME_MS = 30000UL;
constexpr unsigned long BRIDGE_TIMEOUT_MS = 1200UL;
constexpr unsigned long REMOTE_TIMEOUT_MS = 1500UL;
constexpr unsigned long STATUS_PUBLISH_INTERVAL_MS = 100UL;

struct TankState {
  double rawTemp;
  double filteredTemp;
  double setpoint;
  double outputPct;
  bool heaterOn;
  bool sensorValid;
  bool setpointClamped;
  unsigned long windowStart;

  float tempHistory[TREND_BUFFER_SIZE];
  uint8_t historyIndex;
  uint8_t historyCount;

  double kp;
  double ki;
  double kd;

  double kpDefault;
  double kiDefault;
  double kdDefault;

  double kpMin;
  double kpMax;
  double kiMin;
  double kiMax;
  double kdMin;
  double kdMax;

  double integralTerm;
  double prevTemp;
  double dTempFilt;
  unsigned long lastControlMs;

  double theta[3];
  double P[3][3];
  double modelPrevTemp;
  double prevModelOutputNorm;
  bool modelInitialized;
  bool modelValid;
  unsigned long lastRetuneMs;
  unsigned long warmupUntilMs;

  double estTauSec;
  double estKproc;
};

TankState tanks[TANK_COUNT];

NanoToUnoCommandPacket latestCommand = {};
bool haveCommand = false;
uint16_t lastRemoteSequenceApplied = 0;
unsigned long lastBridgeRxMs = 0;
unsigned long lastControlRunMs = 0;
unsigned long lastLogMs = 0;
unsigned long lastStatusPublishMs = 0;
unsigned long lastAtSetpointCheckMs = 0;
unsigned long bootMs = 0;
unsigned long timeAtSetpointSec = 0;
bool atSetpoint = false;

uint16_t statusSequence = 0;

unsigned long bridgeRxCount = 0;
unsigned long bridgeBadLengthCount = 0;
unsigned long bridgeBadPacketCount = 0;
unsigned long bridgeStatusRequestCount = 0;

volatile bool rxPacketPending = false;
volatile uint8_t rxPacketBuffer[sizeof(NanoToUnoCommandPacket)] = {};
volatile uint8_t txStatusBuffer[sizeof(UnoToNanoStatusPacket)] = {};

double clampDouble(double value, double low, double high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

bool isReasonableTemp(double tempC) {
  return !isnan(tempC) && tempC > MIN_REASONABLE_TEMP_C && tempC < MAX_REASONABLE_TEMP_C;
}

void zeroMatrix3(double matrix[3][3]) {
  for (uint8_t row = 0; row < 3; ++row) {
    for (uint8_t col = 0; col < 3; ++col) {
      matrix[row][col] = 0.0;
    }
  }
}

void initTankState(uint8_t tank, double setpointC, double kp0, double ki0, double kd0) {
  TankState& state = tanks[tank];

  state.rawTemp = NAN;
  state.filteredTemp = setpointC;
  state.setpoint = setpointC;
  state.outputPct = 0.0;
  state.heaterOn = false;
  state.sensorValid = false;
  state.setpointClamped = false;
  state.windowStart = 0;

  state.historyIndex = 0;
  state.historyCount = 0;
  for (uint8_t index = 0; index < TREND_BUFFER_SIZE; ++index) {
    state.tempHistory[index] = static_cast<float>(setpointC);
  }

  state.kp = kp0;
  state.ki = ki0;
  state.kd = kd0;
  state.kpDefault = kp0;
  state.kiDefault = ki0;
  state.kdDefault = kd0;

  state.kpMin = 2.0;
  state.kpMax = 45.0;
  state.kiMin = 0.0;
  state.kiMax = 0.60;
  state.kdMin = 0.0;
  state.kdMax = 80.0;

  state.integralTerm = 0.0;
  state.prevTemp = setpointC;
  state.dTempFilt = 0.0;
  state.lastControlMs = 0;

  state.theta[0] = 0.995;
  state.theta[1] = 0.050;
  state.theta[2] = 0.0;

  zeroMatrix3(state.P);
  state.P[0][0] = 1000.0;
  state.P[1][1] = 1000.0;
  state.P[2][2] = 1000.0;

  state.modelPrevTemp = setpointC;
  state.prevModelOutputNorm = 0.0;
  state.modelInitialized = false;
  state.modelValid = false;
  state.lastRetuneMs = 0;
  state.warmupUntilMs = 0;
  state.estTauSec = NAN;
  state.estKproc = NAN;
}

void resetTankControlState(uint8_t tank) {
  TankState& state = tanks[tank];
  state.outputPct = 0.0;
  state.integralTerm = 0.0;
  state.dTempFilt = 0.0;
  state.prevTemp = state.filteredTemp;
  state.prevModelOutputNorm = 0.0;
  state.modelPrevTemp = state.filteredTemp;
}

void pushHistorySample(TankState& state, double tempC) {
  state.tempHistory[state.historyIndex] = static_cast<float>(tempC);
  state.historyIndex = static_cast<uint8_t>((state.historyIndex + 1U) % TREND_BUFFER_SIZE);
  if (state.historyCount < TREND_BUFFER_SIZE) ++state.historyCount;
}

char getTrendChar(uint8_t tank) {
  const TankState& state = tanks[tank];
  if (state.historyCount < TREND_BUFFER_SIZE) return '-';

  uint8_t oldestIndex = state.historyIndex;
  uint8_t newestIndex = static_cast<uint8_t>((state.historyIndex + TREND_BUFFER_SIZE - 1U) % TREND_BUFFER_SIZE);
  float delta = state.tempHistory[newestIndex] - state.tempHistory[oldestIndex];

  if (delta > TREND_THRESHOLD_C) return 'U';
  if (delta < -TREND_THRESHOLD_C) return 'D';
  return '=';
}

bool localBridgeFault(unsigned long nowMs) {
  return (!haveCommand) || ((nowMs - lastBridgeRxMs) > BRIDGE_TIMEOUT_MS);
}

bool remoteCommFault(const NanoToUnoCommandPacket& packet) {
  if ((packet.bridgeFlags & BRIDGE_FLAG_REMOTE_PRESENT) == 0U) return true;
  if ((packet.bridgeFlags & BRIDGE_FLAG_REMOTE_FRESH) == 0U) return true;
  if ((packet.bridgeFlags & BRIDGE_FLAG_REMOTE_TIMEOUT) != 0U) return true;
  if (packet.remoteAgeMs > REMOTE_TIMEOUT_MS) return true;
  return false;
}

bool tankSensorFault(uint8_t tank) {
  const TankState& state = tanks[tank];
  return (!state.sensorValid) || !isReasonableTemp(state.rawTemp);
}

bool tankOverTempFault(uint8_t tank) {
  return isReasonableTemp(tanks[tank].filteredTemp) && (tanks[tank].filteredTemp >= MAX_TEMP_C);
}

bool tankForcedOff(uint8_t tank, unsigned long nowMs) {
  if (localBridgeFault(nowMs)) return true;
  if (haveCommand && remoteCommFault(latestCommand)) return true;
  if (tankSensorFault(tank)) return true;
  if (tankOverTempFault(tank)) return true;
  return false;
}

void updateRLSModel(uint8_t tank) {
  TankState& state = tanks[tank];

  if (!state.modelInitialized) {
    state.modelPrevTemp = state.filteredTemp;
    state.modelInitialized = true;
    return;
  }

  double phi[3];
  phi[0] = state.modelPrevTemp;
  phi[1] = state.prevModelOutputNorm;
  phi[2] = 1.0;

  double Pphi[3] = {0.0, 0.0, 0.0};
  for (uint8_t row = 0; row < 3; ++row) {
    for (uint8_t col = 0; col < 3; ++col) {
      Pphi[row] += state.P[row][col] * phi[col];
    }
  }

  double denom = RLS_FORGET;
  for (uint8_t index = 0; index < 3; ++index) {
    denom += phi[index] * Pphi[index];
  }
  if (denom < 1e-9) return;

  double gain[3];
  for (uint8_t index = 0; index < 3; ++index) {
    gain[index] = Pphi[index] / denom;
  }

  double yHat = 0.0;
  for (uint8_t index = 0; index < 3; ++index) {
    yHat += state.theta[index] * phi[index];
  }

  double error = state.filteredTemp - yHat;
  for (uint8_t index = 0; index < 3; ++index) {
    state.theta[index] += gain[index] * error;
  }

  double phiTP[3] = {0.0, 0.0, 0.0};
  for (uint8_t col = 0; col < 3; ++col) {
    for (uint8_t row = 0; row < 3; ++row) {
      phiTP[col] += phi[row] * state.P[row][col];
    }
  }

  double newP[3][3];
  for (uint8_t row = 0; row < 3; ++row) {
    for (uint8_t col = 0; col < 3; ++col) {
      newP[row][col] = (state.P[row][col] - gain[row] * phiTP[col]) / RLS_FORGET;
    }
  }

  for (uint8_t row = 0; row < 3; ++row) {
    for (uint8_t col = 0; col < 3; ++col) {
      state.P[row][col] = newP[row][col];
    }
  }

  for (uint8_t row = 0; row < 3; ++row) {
    for (uint8_t col = static_cast<uint8_t>(row + 1U); col < 3; ++col) {
      double symmetric = 0.5 * (state.P[row][col] + state.P[col][row]);
      state.P[row][col] = symmetric;
      state.P[col][row] = symmetric;
    }
  }

  state.modelPrevTemp = state.filteredTemp;
}

void retuneAdaptiveGains(uint8_t tank, unsigned long nowMs) {
  TankState& state = tanks[tank];

  if (!state.modelInitialized) return;
  if (nowMs < state.warmupUntilMs) return;
  if ((nowMs - state.lastRetuneMs) < RETUNE_INTERVAL_MS) return;

  state.lastRetuneMs = nowMs;

  double a = state.theta[0];
  double b = state.theta[1];
  double sampleTimeSec = CONTROL_INTERVAL_MS / 1000.0;

  if (!(a > 0.80 && a < 0.99995 && b > 0.0005)) {
    state.modelValid = false;
    return;
  }

  double tauSec = -sampleTimeSec / log(a);
  double processGain = b / (1.0 - a);
  if (!(tauSec > 2.0 && tauSec < 3600.0 && processGain > 0.05 && processGain < 500.0)) {
    state.modelValid = false;
    return;
  }

  double deadTimeSec = 1.0;
  if (deadTimeSec < sampleTimeSec) deadTimeSec = sampleTimeSec;

  double lambdaSec = 0.40 * tauSec;
  if (lambdaSec < 8.0) lambdaSec = 8.0;

  double kc = tauSec / (processGain * (lambdaSec + deadTimeSec));
  double ti = tauSec + 0.5 * deadTimeSec;
  double td = (tauSec * deadTimeSec) / (2.0 * tauSec + deadTimeSec);

  double kpNew = clampDouble(100.0 * kc, state.kpMin, state.kpMax);
  double kiNew = clampDouble((100.0 * kc) / ti, state.kiMin, state.kiMax);
  double kdNew = clampDouble((100.0 * kc) * td, state.kdMin, state.kdMax);

  constexpr double BETA = 0.15;
  state.kp = (1.0 - BETA) * state.kp + BETA * kpNew;
  state.ki = (1.0 - BETA) * state.ki + BETA * kiNew;
  state.kd = (1.0 - BETA) * state.kd + BETA * kdNew;

  state.estTauSec = tauSec;
  state.estKproc = processGain;
  state.modelValid = true;
}

void computeAdaptiveOutput(uint8_t tank, double dtSec) {
  TankState& state = tanks[tank];
  if (dtSec <= 0.0) dtSec = CONTROL_INTERVAL_MS / 1000.0;

  double error = state.setpoint - state.filteredTemp;
  double dTemp = (state.filteredTemp - state.prevTemp) / dtSec;
  state.dTempFilt = (1.0 - D_FILTER_ALPHA) * state.dTempFilt + D_FILTER_ALPHA * dTemp;

  double kpEff = state.kp;
  double kiEff = state.ki;
  double kdEff = state.kd;

  if (error > 3.0) {
    kpEff *= 1.20;
    kiEff *= 0.35;
    kdEff *= 0.45;
  } else if (error > 1.0) {
    kpEff *= 1.05;
    kiEff *= 0.80;
    kdEff *= 0.75;
  } else if (error >= -0.10) {
    kpEff *= 0.85;
    kiEff *= 1.05;
    kdEff *= 1.25;
  } else {
    kpEff *= 0.55;
    kiEff = 0.0;
    kdEff *= 1.80;
  }

  if (state.filteredTemp > (state.setpoint - 0.20) && state.dTempFilt > 0.04) {
    kpEff *= 0.75;
    kiEff *= 0.50;
    kdEff *= 1.40;
  }

  double proportional = kpEff * error;
  double derivative = -kdEff * state.dTempFilt;

  if (fabs(error) < 6.0) {
    state.integralTerm += kiEff * error * dtSec;
  }

  state.integralTerm = clampDouble(state.integralTerm, INTEGRAL_MIN, INTEGRAL_MAX);

  double unsaturated = proportional + state.integralTerm + derivative;
  double output = clampDouble(unsaturated, 0.0, 100.0);

  if (output != unsaturated) {
    state.integralTerm += 0.25 * (output - unsaturated);
    state.integralTerm = clampDouble(state.integralTerm, INTEGRAL_MIN, INTEGRAL_MAX);
    output = clampDouble(proportional + state.integralTerm + derivative, 0.0, 100.0);
  }

  if (error < -0.30) {
    state.integralTerm *= 0.96;
  }

  state.outputPct = output;
  state.prevTemp = state.filteredTemp;
}

void applyMeasurement(uint8_t tank, double tempC, bool valid) {
  TankState& state = tanks[tank];

  if (!valid || !isReasonableTemp(tempC)) {
    state.rawTemp = NAN;
    state.sensorValid = false;
    return;
  }

  state.rawTemp = tempC;
  if (!state.sensorValid || !isReasonableTemp(state.filteredTemp)) {
    state.filteredTemp = tempC;
    state.prevTemp = tempC;
    state.modelPrevTemp = tempC;
  } else {
    state.filteredTemp = EMA_ALPHA * tempC + (1.0 - EMA_ALPHA) * state.filteredTemp;
  }

  state.sensorValid = true;
  pushHistorySample(state, state.filteredTemp);
}

void applyCommandPacket(const NanoToUnoCommandPacket& packet) {
  unsigned long nowMs = millis();
  bool hadCommand = haveCommand;
  latestCommand = packet;
  haveCommand = true;
  lastBridgeRxMs = nowMs;
  ++bridgeRxCount;

  double requestedMainSetpoint = decodeTempCx100(packet.mainSetpointCx100);
  double requestedResSetpoint = decodeTempCx100(packet.resSetpointCx100);
  bool mainSetpointInvalid = isnan(requestedMainSetpoint);
  bool resSetpointInvalid = isnan(requestedResSetpoint);
  if (mainSetpointInvalid) requestedMainSetpoint = DEFAULT_SETPOINT_C;
  if (resSetpointInvalid) requestedResSetpoint = DEFAULT_SETPOINT_C;

  double mainSetpoint = clampDouble(requestedMainSetpoint, MIN_SETPOINT_C, MAX_SETPOINT_ALLOWED_C);
  double resSetpoint = clampDouble(requestedResSetpoint, MIN_SETPOINT_C, MAX_SETPOINT_ALLOWED_C);

  tanks[0].setpointClamped = mainSetpointInvalid || fabs(mainSetpoint - requestedMainSetpoint) > 0.001;
  tanks[1].setpointClamped = resSetpointInvalid || fabs(resSetpoint - requestedResSetpoint) > 0.001;

  tanks[0].setpoint = mainSetpoint;
  tanks[1].setpoint = resSetpoint;

  bool newRemoteSample = (!hadCommand) || (packet.remoteSequence != lastRemoteSequenceApplied);
  if (!newRemoteSample) {
    return;
  }

  lastRemoteSequenceApplied = packet.remoteSequence;

  bool mainValid = (packet.validFlags & REMOTE_VALID_MAIN_TEMP) != 0U;
  bool resValid = (packet.validFlags & REMOTE_VALID_RES_TEMP) != 0U;

  applyMeasurement(0, decodeTempCx100(packet.mainTempCx100), mainValid);
  applyMeasurement(1, decodeTempCx100(packet.resTempCx100), resValid);
}

bool consumeReceivedBridgePacket() {
  if (!rxPacketPending) return false;

  uint8_t localBuffer[sizeof(NanoToUnoCommandPacket)];
  noInterrupts();
  for (size_t index = 0; index < sizeof(NanoToUnoCommandPacket); ++index) {
    localBuffer[index] = rxPacketBuffer[index];
  }
  rxPacketPending = false;
  interrupts();

  NanoToUnoCommandPacket packet;
  memcpy(&packet, localBuffer, sizeof(packet));
  applyCommandPacket(packet);
  return true;
}

void updateAtSetpointTimer(unsigned long nowMs) {
  bool bothHealthy = true;
  for (uint8_t tank = 0; tank < TANK_COUNT; ++tank) {
    if (tankForcedOff(tank, nowMs)) {
      bothHealthy = false;
      break;
    }

    double error = fabs(tanks[tank].filteredTemp - tanks[tank].setpoint);
    if (error > AT_SETPOINT_BAND_C) {
      bothHealthy = false;
      break;
    }
  }

  if (!bothHealthy) {
    atSetpoint = false;
    timeAtSetpointSec = 0;
    lastAtSetpointCheckMs = nowMs;
    return;
  }

  if (lastAtSetpointCheckMs == 0) lastAtSetpointCheckMs = nowMs;
  unsigned long elapsed = nowMs - lastAtSetpointCheckMs;
  if (elapsed >= 1000UL) {
    unsigned long wholeSeconds = elapsed / 1000UL;
    timeAtSetpointSec += wholeSeconds;
    if (timeAtSetpointSec > 359999UL) timeAtSetpointSec = 359999UL;
    lastAtSetpointCheckMs = nowMs - (elapsed % 1000UL);
  }
  atSetpoint = true;
}

void runControl() {
  unsigned long nowMs = millis();
  if ((nowMs - lastControlRunMs) < CONTROL_INTERVAL_MS) return;
  lastControlRunMs = nowMs;

  for (uint8_t tank = 0; tank < TANK_COUNT; ++tank) {
    TankState& state = tanks[tank];

    double dtSec;
    if (state.lastControlMs == 0) {
      dtSec = CONTROL_INTERVAL_MS / 1000.0;
    } else {
      dtSec = (nowMs - state.lastControlMs) / 1000.0;
    }
    if (dtSec <= 0.0) dtSec = CONTROL_INTERVAL_MS / 1000.0;
    state.lastControlMs = nowMs;

    if (tankForcedOff(tank, nowMs)) {
      resetTankControlState(tank);
      continue;
    }

    updateRLSModel(tank);
    retuneAdaptiveGains(tank, nowMs);
    computeAdaptiveOutput(tank, dtSec);
    state.prevModelOutputNorm = state.outputPct / 100.0;
  }

  updateAtSetpointTimer(nowMs);
}

void driveHeaters() {
  unsigned long nowMs = millis();

  for (uint8_t tank = 0; tank < TANK_COUNT; ++tank) {
    TankState& state = tanks[tank];

    while ((nowMs - state.windowStart) >= WINDOW_SIZE_MS) {
      state.windowStart += WINDOW_SIZE_MS;
    }

    unsigned long onTimeMs = static_cast<unsigned long>(WINDOW_SIZE_MS * (state.outputPct / 100.0));
    bool shouldBeOn = !tankForcedOff(tank, nowMs) && ((nowMs - state.windowStart) < onTimeMs);

    if (shouldBeOn != state.heaterOn) {
      state.heaterOn = shouldBeOn;
      digitalWrite(HEAT_PINS[tank], state.heaterOn ? HIGH : LOW);
    }
  }
}

uint16_t buildFaultFlags(unsigned long nowMs) {
  uint16_t faultFlags = 0;

  if (localBridgeFault(nowMs)) faultFlags |= FAULT_LOCAL_BRIDGE;
  if (haveCommand && remoteCommFault(latestCommand)) faultFlags |= FAULT_REMOTE_COMM;
  if (haveCommand && (latestCommand.bridgeFlags & BRIDGE_FLAG_REMOTE_PACKET_DROPPED) != 0U) faultFlags |= FAULT_REMOTE_PACKET_BAD;

  if (tankSensorFault(0)) faultFlags |= FAULT_MAIN_SENSOR_INVALID;
  if (tankSensorFault(1)) faultFlags |= FAULT_RES_SENSOR_INVALID;
  if (tankOverTempFault(0)) faultFlags |= FAULT_MAIN_OVERTEMP;
  if (tankOverTempFault(1)) faultFlags |= FAULT_RES_OVERTEMP;
  if (tanks[0].setpointClamped) faultFlags |= FAULT_MAIN_SP_CLAMPED;
  if (tanks[1].setpointClamped) faultFlags |= FAULT_RES_SP_CLAMPED;
  if (tankForcedOff(0, nowMs)) faultFlags |= FAULT_MAIN_FORCED_OFF;
  if (tankForcedOff(1, nowMs)) faultFlags |= FAULT_RES_FORCED_OFF;
  if (isReasonableTemp(tanks[0].filteredTemp) && tanks[0].filteredTemp >= WARN_TEMP_C) faultFlags |= FAULT_MAIN_WARN;
  if (isReasonableTemp(tanks[1].filteredTemp) && tanks[1].filteredTemp >= WARN_TEMP_C) faultFlags |= FAULT_RES_WARN;

  return faultFlags;
}

void publishStatusPacket() {
  unsigned long nowMs = millis();
  if ((nowMs - lastStatusPublishMs) < STATUS_PUBLISH_INTERVAL_MS) return;
  lastStatusPublishMs = nowMs;

  UnoToNanoStatusPacket packet = {};
  float mainReportedOutput = tankForcedOff(0, nowMs) ? 0.0f : static_cast<float>(tanks[0].outputPct);
  float resReportedOutput = tankForcedOff(1, nowMs) ? 0.0f : static_cast<float>(tanks[1].outputPct);
  packet.statusSequence = ++statusSequence;
  packet.bridgeSequenceEcho = haveCommand ? latestCommand.bridgeSequence : 0U;
  packet.commandAgeMs = haveCommand ? saturateMillisToU16(nowMs - lastBridgeRxMs) : 65535U;
  packet.mainFilteredCx100 = encodeTempCx100(static_cast<float>(tanks[0].filteredTemp));
  packet.resFilteredCx100 = encodeTempCx100(static_cast<float>(tanks[1].filteredTemp));
  packet.mainSetpointCx100 = encodeTempCx100(static_cast<float>(tanks[0].setpoint));
  packet.resSetpointCx100 = encodeTempCx100(static_cast<float>(tanks[1].setpoint));
  packet.mainOutputPermille = encodeOutputPermille(mainReportedOutput);
  packet.resOutputPermille = encodeOutputPermille(resReportedOutput);
  packet.faultFlags = buildFaultFlags(nowMs);
  packet.heaterFlags = 0;
  if (tanks[0].heaterOn) packet.heaterFlags |= HEATER_FLAG_MAIN_ON;
  if (tanks[1].heaterOn) packet.heaterFlags |= HEATER_FLAG_RES_ON;
  packet.atSetpointSeconds = saturateSecondsToU16(timeAtSetpointSec);
  finalizePacket(packet);

  noInterrupts();
  const uint8_t* raw = reinterpret_cast<const uint8_t*>(&packet);
  for (size_t index = 0; index < sizeof(UnoToNanoStatusPacket); ++index) {
    txStatusBuffer[index] = raw[index];
  }
  interrupts();
}

const char* getTankStatusText(uint8_t tank, unsigned long nowMs) {
  if (localBridgeFault(nowMs)) return "BRDG";
  if (haveCommand && remoteCommFault(latestCommand)) return "COMM";
  if (tankSensorFault(tank)) return "SENS";
  if (tankOverTempFault(tank)) return "CUT";
  if (isReasonableTemp(tanks[tank].filteredTemp) && tanks[tank].filteredTemp >= WARN_TEMP_C) return "WARN";
  return "OK";
}

void printLogHeader() {
  Serial.println("# main_box_uno | Uno R4 Minima heater authority");
  Serial.println("# Bridge: I2C slave at 0x42 | SSR MAIN=D7 | SSR RES=D6");
  Serial.println("# Preserved legacy behavior: EMA, trend, RLS model ID, retune, adaptive control, 1 s SSR window");
}

void logData() {
  unsigned long nowMs = millis();
  if ((nowMs - lastLogMs) < LOG_INTERVAL_MS) return;
  lastLogMs = nowMs;

  Serial.print("BSEQ:");
  Serial.print(haveCommand ? latestCommand.bridgeSequence : 0U);
  Serial.print(",RSEQ:");
  Serial.print(haveCommand ? latestCommand.remoteSequence : 0U);
  Serial.print(",BAGE:");
  Serial.print(haveCommand ? (nowMs - lastBridgeRxMs) : 9999UL);
  Serial.print(",RAGE:");
  Serial.print(haveCommand ? latestCommand.remoteAgeMs : 65535U);
  Serial.print(",BridgeFault:");
  Serial.print(localBridgeFault(nowMs) ? 1 : 0);
  Serial.print(",CommFault:");
  Serial.print((haveCommand && remoteCommFault(latestCommand)) ? 1 : 0);
  Serial.print(",AtSP:");
  Serial.print(atSetpoint ? 1 : 0);
  Serial.print(",AtSPSec:");
  Serial.print(timeAtSetpointSec);
  Serial.print(",Faults:0x");
  Serial.print(buildFaultFlags(nowMs), HEX);

  for (uint8_t tank = 0; tank < TANK_COUNT; ++tank) {
    const TankState& state = tanks[tank];
    Serial.print(",");
    Serial.print(TANK_LABELS[tank]);
    Serial.print("Raw:");
    Serial.print(state.rawTemp, 2);
    Serial.print(",");
    Serial.print(TANK_LABELS[tank]);
    Serial.print("Filt:");
    Serial.print(state.filteredTemp, 2);
    Serial.print(",");
    Serial.print(TANK_LABELS[tank]);
    Serial.print("SP:");
    Serial.print(state.setpoint, 2);
    Serial.print(",");
    Serial.print(TANK_LABELS[tank]);
    Serial.print("Out:");
    Serial.print(state.outputPct, 1);
    Serial.print(",");
    Serial.print(TANK_LABELS[tank]);
    Serial.print("Heat:");
    Serial.print(state.heaterOn ? 1 : 0);
    Serial.print(",");
    Serial.print(TANK_LABELS[tank]);
    Serial.print("Trend:");
    Serial.print(getTrendChar(tank));
    Serial.print(",");
    Serial.print(TANK_LABELS[tank]);
    Serial.print("Kp:");
    Serial.print(state.kp, 3);
    Serial.print(",");
    Serial.print(TANK_LABELS[tank]);
    Serial.print("Ki:");
    Serial.print(state.ki, 3);
    Serial.print(",");
    Serial.print(TANK_LABELS[tank]);
    Serial.print("Kd:");
    Serial.print(state.kd, 3);
    Serial.print(",");
    Serial.print(TANK_LABELS[tank]);
    Serial.print("Tau:");
    Serial.print(state.estTauSec, 2);
    Serial.print(",");
    Serial.print(TANK_LABELS[tank]);
    Serial.print("Status:");
    Serial.print(getTankStatusText(tank, nowMs));
  }

  Serial.print(",RxOK:");
  Serial.print(bridgeRxCount);
  Serial.print(",RxBadLen:");
  Serial.print(bridgeBadLengthCount);
  Serial.print(",RxBadPkt:");
  Serial.print(bridgeBadPacketCount);
  Serial.print(",StatusReq:");
  Serial.println(bridgeStatusRequestCount);
}

void onI2CReceive(int byteCount) {
  if (byteCount != static_cast<int>(sizeof(NanoToUnoCommandPacket))) {
    while (Wire.available() > 0) {
      (void)Wire.read();
    }
    ++bridgeBadLengthCount;
    return;
  }

  NanoToUnoCommandPacket packet = {};
  uint8_t* raw = reinterpret_cast<uint8_t*>(&packet);
  int index = 0;
  while (Wire.available() > 0 && index < byteCount) {
    raw[index++] = static_cast<uint8_t>(Wire.read());
  }

  if (index != byteCount || !validatePacket(packet)) {
    ++bridgeBadPacketCount;
    return;
  }

  for (size_t offset = 0; offset < sizeof(NanoToUnoCommandPacket); ++offset) {
    rxPacketBuffer[offset] = raw[offset];
  }
  rxPacketPending = true;
}

void onI2CRequest() {
  ++bridgeStatusRequestCount;
  uint8_t localBuffer[sizeof(UnoToNanoStatusPacket)];
  for (size_t index = 0; index < sizeof(UnoToNanoStatusPacket); ++index) {
    localBuffer[index] = txStatusBuffer[index];
  }
  Wire.write(localBuffer, sizeof(localBuffer));
}

}  // namespace

void setup() {
  Serial.begin(115200);

  pinMode(HEAT_PIN_MAIN, OUTPUT);
  pinMode(HEAT_PIN_RES, OUTPUT);
  digitalWrite(HEAT_PIN_MAIN, LOW);
  digitalWrite(HEAT_PIN_RES, LOW);

  initTankState(0, DEFAULT_SETPOINT_C, 10.0, 0.04, 16.0);
  initTankState(1, DEFAULT_SETPOINT_C, 12.0, 0.05, 18.0);

  bootMs = millis();
  lastControlRunMs = bootMs;
  lastLogMs = bootMs;
  lastStatusPublishMs = bootMs;
  lastAtSetpointCheckMs = bootMs;

  for (uint8_t tank = 0; tank < TANK_COUNT; ++tank) {
    tanks[tank].windowStart = bootMs;
    tanks[tank].lastControlMs = bootMs;
    tanks[tank].lastRetuneMs = bootMs;
    tanks[tank].warmupUntilMs = bootMs + WARMUP_TIME_MS;
  }

  UnoToNanoStatusPacket startupStatus = {};
  startupStatus.statusSequence = statusSequence;
  startupStatus.bridgeSequenceEcho = 0;
  startupStatus.commandAgeMs = 65535U;
  startupStatus.mainFilteredCx100 = encodeTempCx100(static_cast<float>(DEFAULT_SETPOINT_C));
  startupStatus.resFilteredCx100 = encodeTempCx100(static_cast<float>(DEFAULT_SETPOINT_C));
  startupStatus.mainSetpointCx100 = encodeTempCx100(static_cast<float>(DEFAULT_SETPOINT_C));
  startupStatus.resSetpointCx100 = encodeTempCx100(static_cast<float>(DEFAULT_SETPOINT_C));
  startupStatus.mainOutputPermille = 0;
  startupStatus.resOutputPermille = 0;
  startupStatus.faultFlags = FAULT_LOCAL_BRIDGE | FAULT_MAIN_FORCED_OFF | FAULT_RES_FORCED_OFF | FAULT_STATUS_STALE;
  startupStatus.heaterFlags = 0;
  startupStatus.atSetpointSeconds = 0;
  finalizePacket(startupStatus);
  const uint8_t* startupRaw = reinterpret_cast<const uint8_t*>(&startupStatus);
  for (size_t index = 0; index < sizeof(UnoToNanoStatusPacket); ++index) {
    txStatusBuffer[index] = startupRaw[index];
  }

  Wire.begin(SYSTEM_I2C_UNO_ADDRESS);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);

  printLogHeader();
}

void loop() {
  consumeReceivedBridgePacket();
  runControl();
  driveHeaters();
  publishStatusPacket();
  logData();
}
