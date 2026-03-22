#include "buttons.h"
#include "config.h"

// ---------------- File-scope statics ----------------
static ButtonState btnUp   = {BTN_UP_PIN,   HIGH, HIGH, 0};
static ButtonState btnSet  = {BTN_SET_PIN,  HIGH, HIGH, 0};
static ButtonState btnDown = {BTN_DOWN_PIN, HIGH, HIGH, 0};

static bool pendingSetSingleClick = false;
static unsigned long lastSetClickMs = 0;

// ---------------- Static helpers ----------------
static bool updateButtonPressed(ButtonState &btn) {
  bool reading = digitalRead(btn.pin);
  unsigned long now = millis();

  if (reading != btn.lastReading) {
    btn.lastChangeMs = now;
    btn.lastReading = reading;
  }

  if ((now - btn.lastChangeMs) > BUTTON_DEBOUNCE_MS) {
    if (reading != btn.stableState) {
      btn.stableState = reading;
      if (btn.stableState == LOW) {
        return true;
      }
    }
  }
  return false;
}

static void setButtonEvent(DashboardState& state, const char* evt) {
  state.lastButtonEvent = evt;
  state.lastButtonEventAt = millis();
  state.displayDirty = true;
}

static void setTankSetpoint(DashboardState& state, int tank, float newSP) {
  newSP = clampFloat(newSP, MIN_SETPOINT_C, MAX_SETPOINT_ALLOWED_C);
  if (tank == 0) state.main.setpoint = newSP;
  else           state.res.setpoint  = newSP;
  state.displayDirty = true;
}

// ---------------- Public API ----------------
namespace buttons {

void init(DashboardState& state) {
  pinMode(BTN_UP_PIN, INPUT_PULLUP);
  pinMode(BTN_SET_PIN, INPUT_PULLUP);
  pinMode(BTN_DOWN_PIN, INPUT_PULLUP);
}

void update(DashboardState& state) {
  unsigned long now = millis();

  // SET button: single click vs double click
  if (updateButtonPressed(btnSet)) {
    if (pendingSetSingleClick && (now - lastSetClickMs <= DOUBLE_CLICK_MS)) {
      pendingSetSingleClick = false;
      state.selectedTank = (state.selectedTank + 1) % 2;
      setButtonEvent(state, "SET DBL");
    } else {
      pendingSetSingleClick = true;
      lastSetClickMs = now;
    }
  }

  // single click = toggle SET/VIEW
  if (pendingSetSingleClick && (now - lastSetClickMs > DOUBLE_CLICK_MS)) {
    pendingSetSingleClick = false;
    state.setMode = !state.setMode;
    setButtonEvent(state, "SET SGL");
  }

  if (updateButtonPressed(btnUp)) {
    if (state.setMode) {
      if (state.selectedTank == 0) setTankSetpoint(state, 0, state.main.setpoint + BUTTON_SP_STEP_C);
      else                         setTankSetpoint(state, 1, state.res.setpoint  + BUTTON_SP_STEP_C);
    }
    setButtonEvent(state, "UP");
  }

  if (updateButtonPressed(btnDown)) {
    if (state.setMode) {
      if (state.selectedTank == 0) setTankSetpoint(state, 0, state.main.setpoint - BUTTON_SP_STEP_C);
      else                         setTankSetpoint(state, 1, state.res.setpoint  - BUTTON_SP_STEP_C);
    }
    setButtonEvent(state, "DOWN");
  }

  if ((strcmp(state.lastButtonEvent, "NONE") != 0) &&
      (millis() - state.lastButtonEventAt > BTN_MSG_HOLD_MS)) {
    state.lastButtonEvent = "NONE";
    state.displayDirty = true;
  }
}

} // namespace buttons
