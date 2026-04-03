// =============================================================================
// MAIN BOX NANO BRIDGE
//
// Architecture:
//   remote box --nRF24--> this Nano --I2C--> main-box Uno R4 heater controller
//
// Responsibilities on this board:
//   - own the main-box nRF24L01 radio
//   - validate remote packets, track freshness, and forward compact commands
//   - poll controller status from the Uno over I2C
//   - relay Uno status back to the remote box over nRF24
//
// This board is a transport bridge only. Heater control authority stays on the
// main-box Uno.
//
// Pin map:
//   D9   -> nRF24 CE
//   D10  -> nRF24 CSN
//   D11  -> nRF24 MOSI
//   D12  -> nRF24 MISO
//   D13  -> nRF24 SCK
//   A4   -> I2C SDA to main-box Uno SDA
//   A5   -> I2C SCL to main-box Uno SCL
//
// Hardware note:
//   Place a 10 uF capacitor across the nRF24L01 VCC and GND pins close to the
//   radio module to reduce brownout and packet-loss issues.
// =============================================================================

#include <SPI.h>
#include <string.h>
#include <Wire.h>
#include <nRF24L01.h>
#include <RF24.h>

#include "system_packets.h"

namespace {

constexpr uint8_t RADIO_CE_PIN = 9;
constexpr uint8_t RADIO_CSN_PIN = 10;

constexpr float DEFAULT_SETPOINT_C = 37.0f;

constexpr unsigned long REMOTE_TIMEOUT_MS = 1500UL;
constexpr unsigned long LOCAL_BRIDGE_TIMEOUT_MS = 1200UL;
constexpr unsigned long I2C_COMMAND_INTERVAL_MS = 100UL;
constexpr unsigned long I2C_STATUS_INTERVAL_MS = 100UL;
constexpr unsigned long RADIO_STATUS_INTERVAL_MS = 150UL;
constexpr unsigned long LOG_INTERVAL_MS = 500UL;

RF24 radioHw(RADIO_CE_PIN, RADIO_CSN_PIN);

bool radioInitOk = false;
bool lastRadioTxOk = false;

RemoteToMainPacket latestRemotePacket = {};
bool haveRemotePacket = false;
bool remotePacketDroppedSinceGood = false;
unsigned long lastRemoteRxMs = 0;

NanoToUnoCommandPacket lastCommandPacket = {};
unsigned long lastCommandTxAttemptMs = 0;
unsigned long lastCommandTxOkMs = 0;
uint8_t lastCommandTxError = 0;
unsigned long i2cCommandOkCount = 0;
unsigned long i2cCommandFailCount = 0;

UnoToNanoStatusPacket latestUnoStatus = {};
bool haveUnoStatus = false;
unsigned long lastUnoStatusMs = 0;
uint8_t lastUnoStatusError = 0;
unsigned long i2cStatusOkCount = 0;
unsigned long i2cStatusFailCount = 0;

MainToRemotePacket lastDownlinkPacket = {};
bool haveDownlinkPacket = false;

unsigned long lastCommandSendMs = 0;
unsigned long lastStatusPollMs = 0;
unsigned long lastRadioStatusMs = 0;
unsigned long lastLogMs = 0;

uint16_t bridgeSequence = 0;
uint16_t downlinkSequence = 0;

unsigned long radioRxOkCount = 0;
unsigned long radioRxBadCount = 0;
unsigned long radioTxCount = 0;

bool remoteFresh(unsigned long nowMs) {
  return haveRemotePacket && ((nowMs - lastRemoteRxMs) <= REMOTE_TIMEOUT_MS);
}

bool commandBridgeFresh(unsigned long nowMs) {
  return lastCommandTxOkMs != 0 && ((nowMs - lastCommandTxOkMs) <= LOCAL_BRIDGE_TIMEOUT_MS);
}

bool unoStatusFresh(unsigned long nowMs) {
  return haveUnoStatus && ((nowMs - lastUnoStatusMs) <= LOCAL_BRIDGE_TIMEOUT_MS);
}

bool localBridgeFresh(unsigned long nowMs) {
  return commandBridgeFresh(nowMs) && unoStatusFresh(nowMs);
}

uint8_t buildBridgeFlags(unsigned long nowMs) {
  uint8_t flags = 0;

  if (haveRemotePacket) flags |= BRIDGE_FLAG_REMOTE_PRESENT;
  if (remoteFresh(nowMs)) flags |= BRIDGE_FLAG_REMOTE_FRESH;
  if (haveRemotePacket) flags |= BRIDGE_FLAG_REMOTE_PACKET_VALID;
  if (remotePacketDroppedSinceGood) flags |= BRIDGE_FLAG_REMOTE_PACKET_DROPPED;
  if (!remoteFresh(nowMs)) flags |= BRIDGE_FLAG_REMOTE_TIMEOUT;

  return flags;
}

void initRadio() {
  radioInitOk = radioHw.begin();
  if (!radioInitOk) {
    Serial.println("Radio init failed");
    return;
  }

  radioHw.setPALevel(RF24_PA_LOW);
  radioHw.setDataRate(RF24_250KBPS);
  radioHw.setChannel(SYSTEM_RADIO_CHANNEL);
  radioHw.setAutoAck(false);
  radioHw.openReadingPipe(1, SYSTEM_PIPE_REMOTE_TO_MAIN);
  radioHw.openWritingPipe(SYSTEM_PIPE_MAIN_TO_REMOTE);
  radioHw.startListening();
}

void pollRadio() {
  if (!radioInitOk) return;

  RemoteToMainPacket packet = {};
  bool sawPacket = false;

  while (radioHw.available()) {
    radioHw.read(&packet, sizeof(packet));
    sawPacket = true;
  }

  if (!sawPacket) return;

  if (!validatePacket(packet)) {
    ++radioRxBadCount;
    remotePacketDroppedSinceGood = true;
    return;
  }

  latestRemotePacket = packet;
  haveRemotePacket = true;
  remotePacketDroppedSinceGood = false;
  lastRemoteRxMs = millis();
  ++radioRxOkCount;
}

void buildCommandPacket(NanoToUnoCommandPacket& packet, unsigned long nowMs) {
  packet = {};
  packet.bridgeSequence = ++bridgeSequence;
  packet.remoteSequence = haveRemotePacket ? latestRemotePacket.sequence : 0U;
  packet.remoteAgeMs = haveRemotePacket ? saturateMillisToU16(nowMs - lastRemoteRxMs) : 65535U;

  if (haveRemotePacket) {
    packet.mainTempCx100 = latestRemotePacket.mainTempCx100;
    packet.resTempCx100 = latestRemotePacket.resTempCx100;
    packet.mainSetpointCx100 = latestRemotePacket.mainSetpointCx100;
    packet.resSetpointCx100 = latestRemotePacket.resSetpointCx100;
    packet.validFlags = latestRemotePacket.validFlags;
    packet.uiFlags = latestRemotePacket.uiFlags;
    packet.buttonFlags = latestRemotePacket.buttonFlags;
  } else {
    packet.mainTempCx100 = TEMP_CX100_INVALID;
    packet.resTempCx100 = TEMP_CX100_INVALID;
    packet.mainSetpointCx100 = encodeTempCx100(DEFAULT_SETPOINT_C);
    packet.resSetpointCx100 = encodeTempCx100(DEFAULT_SETPOINT_C);
    packet.validFlags = 0;
    packet.uiFlags = 0;
    packet.buttonFlags = 0;
  }

  packet.bridgeFlags = buildBridgeFlags(nowMs);
  finalizePacket(packet);
}

void sendCommandToUno() {
  unsigned long nowMs = millis();
  if ((nowMs - lastCommandSendMs) < I2C_COMMAND_INTERVAL_MS) return;
  lastCommandSendMs = nowMs;

  NanoToUnoCommandPacket packet;
  buildCommandPacket(packet, nowMs);

  lastCommandPacket = packet;
  lastCommandTxAttemptMs = nowMs;

  Wire.beginTransmission(SYSTEM_I2C_UNO_ADDRESS);
  Wire.write(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
  uint8_t error = Wire.endTransmission();

  if (error == 0U) {
    lastCommandTxError = 0U;
    lastCommandTxOkMs = nowMs;
    ++i2cCommandOkCount;
  } else {
    lastCommandTxError = error;
    ++i2cCommandFailCount;
  }
}

void pollUnoStatus() {
  unsigned long nowMs = millis();
  if ((nowMs - lastStatusPollMs) < I2C_STATUS_INTERVAL_MS) return;
  lastStatusPollMs = nowMs;

  UnoToNanoStatusPacket packet = {};
  uint8_t raw[sizeof(UnoToNanoStatusPacket)];
  uint8_t received = Wire.requestFrom(static_cast<uint8_t>(SYSTEM_I2C_UNO_ADDRESS),
                                      static_cast<uint8_t>(sizeof(UnoToNanoStatusPacket)));

  if (received != sizeof(UnoToNanoStatusPacket)) {
    while (Wire.available() > 0) {
      (void)Wire.read();
    }
    lastUnoStatusError = 1U;
    ++i2cStatusFailCount;
    return;
  }

  uint8_t index = 0;
  while (Wire.available() > 0 && index < sizeof(UnoToNanoStatusPacket)) {
    raw[index++] = static_cast<uint8_t>(Wire.read());
  }

  if (index != sizeof(UnoToNanoStatusPacket)) {
    lastUnoStatusError = 2U;
    ++i2cStatusFailCount;
    return;
  }

  memcpy(&packet, raw, sizeof(packet));
  if (!validatePacket(packet)) {
    lastUnoStatusError = 3U;
    ++i2cStatusFailCount;
    return;
  }

  latestUnoStatus = packet;
  haveUnoStatus = true;
  lastUnoStatusMs = nowMs;
  lastUnoStatusError = 0U;
  ++i2cStatusOkCount;
}

void buildDownlinkPacket(MainToRemotePacket& packet, unsigned long nowMs) {
  packet = {};
  packet.statusSequence = ++downlinkSequence;
  packet.remoteSequenceEcho = haveRemotePacket ? latestRemotePacket.sequence : 0U;
  packet.controllerAgeMs = haveUnoStatus ? latestUnoStatus.commandAgeMs : 65535U;

  if (haveUnoStatus) {
    packet.mainFilteredCx100 = latestUnoStatus.mainFilteredCx100;
    packet.resFilteredCx100 = latestUnoStatus.resFilteredCx100;
    packet.mainSetpointCx100 = latestUnoStatus.mainSetpointCx100;
    packet.resSetpointCx100 = latestUnoStatus.resSetpointCx100;
    packet.mainOutputPermille = latestUnoStatus.mainOutputPermille;
    packet.resOutputPermille = latestUnoStatus.resOutputPermille;
    packet.faultFlags = latestUnoStatus.faultFlags;
    packet.heaterFlags = latestUnoStatus.heaterFlags;
    packet.atSetpointSeconds = latestUnoStatus.atSetpointSeconds;
  } else {
    packet.mainFilteredCx100 = TEMP_CX100_INVALID;
    packet.resFilteredCx100 = TEMP_CX100_INVALID;
    packet.mainSetpointCx100 = encodeTempCx100(DEFAULT_SETPOINT_C);
    packet.resSetpointCx100 = encodeTempCx100(DEFAULT_SETPOINT_C);
    packet.mainOutputPermille = 0;
    packet.resOutputPermille = 0;
    packet.faultFlags = 0;
    packet.heaterFlags = 0;
    packet.atSetpointSeconds = 0;
  }

  packet.linkFlags = 0;
  if (remoteFresh(nowMs)) packet.linkFlags |= LINK_FLAG_REMOTE_PACKET_FRESH;
  if (localBridgeFresh(nowMs)) packet.linkFlags |= LINK_FLAG_LOCAL_BRIDGE_FRESH;
  if (unoStatusFresh(nowMs)) packet.linkFlags |= LINK_FLAG_UNO_STATUS_FRESH;
  if (haveRemotePacket) packet.linkFlags |= LINK_FLAG_RADIO_RX_VALID;
  if (lastRadioTxOk) packet.linkFlags |= LINK_FLAG_RADIO_TX_OK;

  if (!remoteFresh(nowMs)) {
    packet.faultFlags |= FAULT_REMOTE_COMM;
  }
  if (remotePacketDroppedSinceGood) {
    packet.faultFlags |= FAULT_REMOTE_PACKET_BAD;
  }
  if (!localBridgeFresh(nowMs)) {
    packet.faultFlags |= FAULT_LOCAL_BRIDGE;
  }

  if (!unoStatusFresh(nowMs)) {
    packet.controllerAgeMs = haveUnoStatus ? saturateMillisToU16(nowMs - lastUnoStatusMs) : 65535U;
    packet.linkFlags |= LINK_FLAG_STATUS_SYNTHETIC;
    packet.faultFlags |= FAULT_STATUS_STALE | FAULT_MAIN_FORCED_OFF | FAULT_RES_FORCED_OFF;
    packet.mainOutputPermille = 0;
    packet.resOutputPermille = 0;
    packet.heaterFlags = 0;
  }

  finalizePacket(packet);
}

void sendStatusToRemote() {
  unsigned long nowMs = millis();
  if (!radioInitOk) return;
  if ((nowMs - lastRadioStatusMs) < RADIO_STATUS_INTERVAL_MS) return;
  lastRadioStatusMs = nowMs;

  MainToRemotePacket packet;
  buildDownlinkPacket(packet, nowMs);

  radioHw.stopListening();
  bool ok = radioHw.write(&packet, sizeof(packet));
  radioHw.startListening();

  lastRadioTxOk = ok;
  lastDownlinkPacket = packet;
  haveDownlinkPacket = true;
  ++radioTxCount;
}

void printLogHeader() {
  Serial.println("# main_box_nano_bridge | Nano radio/I2C bridge");
  Serial.println("# Radio CE=D9 CSN=D10 | I2C master to Uno 0x42");
  Serial.println("# nRF24 note: add 10 uF decoupling capacitor close to VCC/GND");
}

void logData() {
  unsigned long nowMs = millis();
  if ((nowMs - lastLogMs) < LOG_INTERVAL_MS) return;
  lastLogMs = nowMs;

  Serial.print("RadioInit:");
  Serial.print(radioInitOk ? 1 : 0);
  Serial.print(",RxOK:");
  Serial.print(radioRxOkCount);
  Serial.print(",RxBad:");
  Serial.print(radioRxBadCount);
  Serial.print(",Tx:");
  Serial.print(radioTxCount);
  Serial.print(",RemoteSeq:");
  Serial.print(haveRemotePacket ? latestRemotePacket.sequence : 0U);
  Serial.print(",RemoteAge:");
  Serial.print(haveRemotePacket ? (nowMs - lastRemoteRxMs) : 65535UL);
  Serial.print(",RemoteFresh:");
  Serial.print(remoteFresh(nowMs) ? 1 : 0);
  Serial.print(",BridgeSeq:");
  Serial.print(lastCommandPacket.bridgeSequence);
  Serial.print(",I2CTxOK:");
  Serial.print(i2cCommandOkCount);
  Serial.print(",I2CTxFail:");
  Serial.print(i2cCommandFailCount);
  Serial.print(",I2CTxErr:");
  Serial.print(lastCommandTxError);
  Serial.print(",I2CStatusOK:");
  Serial.print(i2cStatusOkCount);
  Serial.print(",I2CStatusFail:");
  Serial.print(i2cStatusFailCount);
  Serial.print(",I2CStatusErr:");
  Serial.print(lastUnoStatusError);
  Serial.print(",LocalBridgeFresh:");
  Serial.print(localBridgeFresh(nowMs) ? 1 : 0);
  Serial.print(",UnoStatusAge:");
  Serial.print(haveUnoStatus ? (nowMs - lastUnoStatusMs) : 65535UL);
  Serial.print(",RadioTxOK:");
  Serial.print(lastRadioTxOk ? 1 : 0);

  if (haveDownlinkPacket) {
    Serial.print(",DownFaults:0x");
    Serial.print(lastDownlinkPacket.faultFlags, HEX);
    Serial.print(",DownLink:0x");
    Serial.print(lastDownlinkPacket.linkFlags, HEX);
    Serial.print(",MainOut:");
    Serial.print(decodeOutputPermille(lastDownlinkPacket.mainOutputPermille), 1);
    Serial.print(",ResOut:");
    Serial.print(decodeOutputPermille(lastDownlinkPacket.resOutputPermille), 1);
  }

  Serial.println();
}

}  // namespace

void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(100000UL);

  SPI.begin();
  initRadio();
  printLogHeader();
}

void loop() {
  pollRadio();
  sendCommandToUno();
  pollUnoStatus();
  sendStatusToRemote();
  logData();
}
