// Legacy payload definition retained for the previous radio-only architecture.
// Active packet definitions now live in firmware/shared/system_packets.h.

#ifndef RADIO_PAYLOAD_H
#define RADIO_PAYLOAD_H

#include <stdint.h>

constexpr uint8_t RADIO_VALID_MAIN = 0x01;
constexpr uint8_t RADIO_VALID_RES  = 0x02;

#pragma pack(push, 1)
struct RadioTankPayload {
  float rawTempC;
  float setpointC;
};

struct RadioPayload {
  RadioTankPayload main;
  RadioTankPayload res;
  uint8_t validMask;
  uint8_t counter;
};
#pragma pack(pop)

static_assert(sizeof(RadioPayload) == 18, "Unexpected radio payload size");

#endif // RADIO_PAYLOAD_H
