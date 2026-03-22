#include "serial_log.h"
#include "config.h"
#include <stdio.h>
#include <stdlib.h>  // for dtostrf

namespace serial_log {

static char buf[128];
static uint8_t len = 0;
static uint8_t cursor = 0;
static unsigned long lastBuild = 0;

void init(DashboardState& state) {
  (void)state;
}

void update(DashboardState& state) {
  unsigned long now = millis();

  // Build new message when previous is fully sent and interval elapsed
  if (cursor >= len && (now - lastBuild >= SERIAL_INTERVAL)) {
    lastBuild = now;

    char t1[7], t2[7], s1[7], s2[7];
    dtostrf(state.main.filteredTemp, 5, 2, t1);
    dtostrf(state.res.filteredTemp,  5, 2, t2);
    dtostrf(state.main.setpoint,     5, 2, s1);
    dtostrf(state.res.setpoint,      5, 2, s2);

    len = snprintf(buf, sizeof(buf),
                   "MAIN T=%s SP=%s | RES T=%s SP=%s | TX=%s\n",
                   t1, s1, t2, s2,
                   state.lastTxOk ? "OK" : "FAIL");
    if (len >= sizeof(buf)) len = sizeof(buf) - 1;
    cursor = 0;
  }

  // Drip-feed: write only what the HW buffer can accept
  if (cursor < len) {
    int avail = Serial.availableForWrite();
    if (avail > 0) {
      uint8_t chunk = min((uint8_t)avail, (uint8_t)(len - cursor));
      Serial.write((const uint8_t*)(buf + cursor), chunk);
      cursor += chunk;
    }
  }
}

} // namespace serial_log
