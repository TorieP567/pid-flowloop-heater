#line 1 "C:\\Users\\18tor\\Downloads\\MechResearch\\pid-flowloop-heater\\firmware\\remote_box\\system_packets.h"
#ifndef SYSTEM_PACKETS_H
#define SYSTEM_PACKETS_H

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

constexpr uint8_t SYSTEM_PACKET_VERSION = 1;
constexpr uint8_t SYSTEM_I2C_UNO_ADDRESS = 0x42;

constexpr uint8_t SYSTEM_RADIO_CHANNEL = 108;
constexpr uint8_t SYSTEM_RADIO_PA_LEVEL = 1;  // RF24_PA_LOW

static const uint8_t SYSTEM_PIPE_REMOTE_TO_MAIN[6] = "R2M01";
static const uint8_t SYSTEM_PIPE_MAIN_TO_REMOTE[6] = "M2R01";

constexpr int16_t TEMP_CX100_INVALID = INT16_MIN;
constexpr uint16_t OUTPUT_PERMILLE_MAX = 1000;

enum RemoteValidFlags : uint8_t {
  REMOTE_VALID_MAIN_TEMP = 0x01,
  REMOTE_VALID_RES_TEMP  = 0x02
};

enum RemoteUiFlags : uint8_t {
  REMOTE_UI_SET_MODE          = 0x01,
  REMOTE_UI_SELECTED_TANK_RES = 0x02
};

enum RemoteButtonFlags : uint8_t {
  REMOTE_BUTTON_UP           = 0x01,
  REMOTE_BUTTON_SET          = 0x02,
  REMOTE_BUTTON_DOWN         = 0x04,
  REMOTE_BUTTON_DOUBLE_CLICK = 0x08
};

enum BridgeFlags : uint8_t {
  BRIDGE_FLAG_REMOTE_PRESENT        = 0x01,
  BRIDGE_FLAG_REMOTE_FRESH          = 0x02,
  BRIDGE_FLAG_REMOTE_PACKET_VALID   = 0x04,
  BRIDGE_FLAG_REMOTE_PACKET_DROPPED = 0x08,
  BRIDGE_FLAG_REMOTE_TIMEOUT        = 0x10
};

enum HeaterFlags : uint8_t {
  HEATER_FLAG_MAIN_ON = 0x01,
  HEATER_FLAG_RES_ON  = 0x02
};

enum FaultFlags : uint16_t {
  FAULT_REMOTE_COMM         = 0x0001,
  FAULT_LOCAL_BRIDGE        = 0x0002,
  FAULT_MAIN_SENSOR_INVALID = 0x0004,
  FAULT_RES_SENSOR_INVALID  = 0x0008,
  FAULT_MAIN_OVERTEMP       = 0x0010,
  FAULT_RES_OVERTEMP        = 0x0020,
  FAULT_MAIN_SP_CLAMPED     = 0x0040,
  FAULT_RES_SP_CLAMPED      = 0x0080,
  FAULT_MAIN_FORCED_OFF     = 0x0100,
  FAULT_RES_FORCED_OFF      = 0x0200,
  FAULT_MAIN_WARN           = 0x0400,
  FAULT_RES_WARN            = 0x0800,
  FAULT_REMOTE_PACKET_BAD   = 0x1000,
  FAULT_STATUS_STALE        = 0x2000
};

enum LinkFlags : uint8_t {
  LINK_FLAG_REMOTE_PACKET_FRESH = 0x01,
  LINK_FLAG_LOCAL_BRIDGE_FRESH  = 0x02,
  LINK_FLAG_UNO_STATUS_FRESH    = 0x04,
  LINK_FLAG_STATUS_SYNTHETIC    = 0x08,
  LINK_FLAG_RADIO_RX_VALID      = 0x10,
  LINK_FLAG_RADIO_TX_OK         = 0x20
};

inline int16_t encodeTempCx100(float valueC) {
  if (isnan(valueC)) return TEMP_CX100_INVALID;
  long scaled = lroundf(valueC * 100.0f);
  if (scaled < -30000L) scaled = -30000L;
  if (scaled > 30000L) scaled = 30000L;
  return static_cast<int16_t>(scaled);
}

inline float decodeTempCx100(int16_t valueCx100) {
  if (valueCx100 == TEMP_CX100_INVALID) return NAN;
  return static_cast<float>(valueCx100) / 100.0f;
}

inline uint16_t encodeOutputPermille(float outputPct) {
  if (isnan(outputPct)) return 0;
  long scaled = lroundf(outputPct * 10.0f);
  if (scaled < 0L) scaled = 0L;
  if (scaled > static_cast<long>(OUTPUT_PERMILLE_MAX)) scaled = OUTPUT_PERMILLE_MAX;
  return static_cast<uint16_t>(scaled);
}

inline float decodeOutputPermille(uint16_t permille) {
  if (permille > OUTPUT_PERMILLE_MAX) permille = OUTPUT_PERMILLE_MAX;
  return static_cast<float>(permille) / 10.0f;
}

inline uint16_t saturateMillisToU16(unsigned long elapsedMs) {
  if (elapsedMs > 65535UL) return 65535U;
  return static_cast<uint16_t>(elapsedMs);
}

inline uint16_t saturateSecondsToU16(unsigned long elapsedSec) {
  if (elapsedSec > 65535UL) return 65535U;
  return static_cast<uint16_t>(elapsedSec);
}

inline uint16_t crc16Ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000U) != 0U) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021U);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

#pragma pack(push, 1)
struct RemoteToMainPacket {
  uint8_t version;
  uint16_t sequence;
  uint32_t remoteMillis;
  int16_t mainTempCx100;
  int16_t resTempCx100;
  int16_t mainSetpointCx100;
  int16_t resSetpointCx100;
  uint8_t validFlags;
  uint8_t uiFlags;
  uint8_t buttonFlags;
  uint16_t checksum;
};

struct NanoToUnoCommandPacket {
  uint8_t version;
  uint16_t bridgeSequence;
  uint16_t remoteSequence;
  uint16_t remoteAgeMs;
  int16_t mainTempCx100;
  int16_t resTempCx100;
  int16_t mainSetpointCx100;
  int16_t resSetpointCx100;
  uint8_t validFlags;
  uint8_t uiFlags;
  uint8_t buttonFlags;
  uint8_t bridgeFlags;
  uint16_t checksum;
};

struct UnoToNanoStatusPacket {
  uint8_t version;
  uint16_t statusSequence;
  uint16_t bridgeSequenceEcho;
  uint16_t commandAgeMs;
  int16_t mainFilteredCx100;
  int16_t resFilteredCx100;
  int16_t mainSetpointCx100;
  int16_t resSetpointCx100;
  uint16_t mainOutputPermille;
  uint16_t resOutputPermille;
  uint16_t faultFlags;
  uint8_t heaterFlags;
  uint16_t atSetpointSeconds;
  uint16_t checksum;
};

struct MainToRemotePacket {
  uint8_t version;
  uint16_t statusSequence;
  uint16_t remoteSequenceEcho;
  uint16_t controllerAgeMs;
  int16_t mainFilteredCx100;
  int16_t resFilteredCx100;
  int16_t mainSetpointCx100;
  int16_t resSetpointCx100;
  uint16_t mainOutputPermille;
  uint16_t resOutputPermille;
  uint16_t faultFlags;
  uint8_t heaterFlags;
  uint8_t linkFlags;
  uint16_t atSetpointSeconds;
  uint16_t checksum;
};
#pragma pack(pop)

static_assert(sizeof(RemoteToMainPacket) <= 32, "RemoteToMainPacket must fit in an nRF24 payload");
static_assert(sizeof(NanoToUnoCommandPacket) <= 32, "NanoToUnoCommandPacket must fit in an I2C transaction");
static_assert(sizeof(UnoToNanoStatusPacket) <= 32, "UnoToNanoStatusPacket must fit in an I2C transaction");
static_assert(sizeof(MainToRemotePacket) <= 32, "MainToRemotePacket must fit in an nRF24 payload");

template <typename PacketT>
inline void finalizePacket(PacketT& packet) {
  packet.version = SYSTEM_PACKET_VERSION;
  packet.checksum = 0;
  packet.checksum = crc16Ccitt(reinterpret_cast<const uint8_t*>(&packet), sizeof(PacketT));
}

template <typename PacketT>
inline bool validatePacket(const PacketT& packet) {
  if (packet.version != SYSTEM_PACKET_VERSION) return false;
  PacketT copy = packet;
  uint16_t expected = copy.checksum;
  copy.checksum = 0;
  return expected == crc16Ccitt(reinterpret_cast<const uint8_t*>(&copy), sizeof(PacketT));
}

#endif  // SYSTEM_PACKETS_H
