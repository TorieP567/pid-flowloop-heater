#include "serial_log.h"
#include "config.h"

namespace serial_log {

static char buf[96];
static uint8_t len = 0;
static uint8_t cursor = 0;
static unsigned long lastBuild = 0;

// Append float as "XX.X" to buf at pos, return new pos
static uint8_t appendFloat1(char* b, uint8_t pos, float v) {
  if (v < 0) { b[pos++] = '-'; v = -v; }
  int whole = (int)v;
  int frac = (int)((v - whole) * 10 + 0.5f);
  if (frac >= 10) { whole++; frac = 0; }
  if (whole >= 100) { b[pos++] = '0' + whole / 100; whole %= 100; }
  if (whole >= 10) b[pos++] = '0' + whole / 10;
  b[pos++] = '0' + whole % 10;
  b[pos++] = '.';
  b[pos++] = '0' + frac;
  return pos;
}

static uint8_t appendStr(char* b, uint8_t pos, const char* s) {
  while (*s) b[pos++] = *s++;
  return pos;
}

void init(DashboardState& state) {
  (void)state;
}

void update(DashboardState& state) {
  unsigned long now = millis();

  if (cursor >= len && (now - lastBuild >= SERIAL_INTERVAL)) {
    lastBuild = now;
    uint8_t p = 0;
    p = appendStr(buf, p, "M=");
    p = appendFloat1(buf, p, state.main.filteredTemp);
    p = appendStr(buf, p, "/");
    p = appendFloat1(buf, p, state.main.setpoint);
    p = appendStr(buf, p, " R=");
    p = appendFloat1(buf, p, state.res.filteredTemp);
    p = appendStr(buf, p, "/");
    p = appendFloat1(buf, p, state.res.setpoint);
    p = appendStr(buf, p, state.lastTxOk ? " OK\n" : " FL\n");
    buf[p] = '\0';
    len = p;
    cursor = 0;
  }

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
