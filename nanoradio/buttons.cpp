#include "buttons.h"

#include "config.h"

namespace {

enum ButtonIndex : uint8_t {
  BUTTON_INDEX_UP = 0,
  BUTTON_INDEX_SET = 1,
  BUTTON_INDEX_DOWN = 2
};

void initButtonTracker(ButtonTracker& button, uint8_t pin) {
  button.pin = pin;
  button.stableState = HIGH;
  button.lastReading = HIGH;
  button.lastChangeMs = 0;
  button.pressedAtMs = 0;
  button.longHandled = false;
}

bool updateButtonState(ButtonTracker& button, bool& pressedEvent, bool& releasedEvent) {
  pressedEvent = false;
  releasedEvent = false;

  const bool reading = digitalRead(button.pin);
  const unsigned long nowMs = millis();

  if (reading != button.lastReading) {
    button.lastReading = reading;
    button.lastChangeMs = nowMs;
  }

  if ((nowMs - button.lastChangeMs) > config::timing::BUTTON_DEBOUNCE_MS &&
      reading != button.stableState) {
    button.stableState = reading;
    if (button.stableState == LOW) {
      button.pressedAtMs = nowMs;
      button.longHandled = false;
      pressedEvent = true;
    } else {
      releasedEvent = true;
    }
    return true;
  }

  return false;
}

void toggleScreenMode(DashboardState& state) {
  state.screenMode = (state.screenMode == SCREEN_MODE_MAIN) ? SCREEN_MODE_DEBUG : SCREEN_MODE_MAIN;
  state.displayNeedsFullRedraw = true;
}

void adjustSelectedSetpoint(DashboardState& state, float deltaC) {
  TankLocalState& tank = state.localTanks[state.selectedTank];
  tank.requestedSetpointC = config::clampFloat(
      tank.requestedSetpointC + deltaC,
      config::setpoint::MIN_SETPOINT_C,
      config::setpoint::MAX_SETPOINT_ALLOWED_C);
}

}  // namespace

namespace buttons {

void init(DashboardState& state) {
  initButtonTracker(state.buttons[BUTTON_INDEX_UP], config::pins::BTN_UP_PIN);
  initButtonTracker(state.buttons[BUTTON_INDEX_SET], config::pins::BTN_SET_PIN);
  initButtonTracker(state.buttons[BUTTON_INDEX_DOWN], config::pins::BTN_DOWN_PIN);

  pinMode(config::pins::BTN_UP_PIN, INPUT_PULLUP);
  pinMode(config::pins::BTN_SET_PIN, INPUT_PULLUP);
  pinMode(config::pins::BTN_DOWN_PIN, INPUT_PULLUP);
}

void update(DashboardState& state) {
  bool upPressed = false;
  bool upReleased = false;
  bool setPressed = false;
  bool setReleased = false;
  bool downPressed = false;
  bool downReleased = false;

  updateButtonState(state.buttons[BUTTON_INDEX_UP], upPressed, upReleased);
  updateButtonState(state.buttons[BUTTON_INDEX_SET], setPressed, setReleased);
  updateButtonState(state.buttons[BUTTON_INDEX_DOWN], downPressed, downReleased);

  const unsigned long nowMs = millis();
  ButtonTracker& upButton = state.buttons[BUTTON_INDEX_UP];
  ButtonTracker& setButton = state.buttons[BUTTON_INDEX_SET];
  ButtonTracker& downButton = state.buttons[BUTTON_INDEX_DOWN];

  const bool comboActive = (upButton.stableState == LOW) && (downButton.stableState == LOW);
  if (comboActive) {
    if (state.screenToggleComboStartMs == 0) {
      state.screenToggleComboStartMs = nowMs;
    }
    if (!state.screenToggleComboHandled &&
        (nowMs - state.screenToggleComboStartMs) >= config::timing::SCREEN_TOGGLE_HOLD_MS) {
      state.screenToggleComboHandled = true;
      toggleScreenMode(state);
    }
  } else {
    state.screenToggleComboStartMs = 0;
    state.screenToggleComboHandled = false;
  }

  if (setButton.stableState == LOW &&
      !setButton.longHandled &&
      (nowMs - setButton.pressedAtMs) >= config::timing::SET_LONG_PRESS_MS) {
    setButton.longHandled = true;
    state.editMode = !state.editMode;
    state.pendingButtonFlags |= REMOTE_BUTTON_SET;
  }

  if (setReleased && !setButton.longHandled) {
    state.selectedTank = (state.selectedTank == config::TANK_MAIN) ? config::TANK_RES : config::TANK_MAIN;
    state.pendingButtonFlags |= REMOTE_BUTTON_SET;
  }

  if (!comboActive && state.editMode) {
    if (upPressed && downButton.stableState != LOW) {
      adjustSelectedSetpoint(state, config::setpoint::SETPOINT_STEP_C);
      state.pendingButtonFlags |= REMOTE_BUTTON_UP;
    }
    if (downPressed && upButton.stableState != LOW) {
      adjustSelectedSetpoint(state, -config::setpoint::SETPOINT_STEP_C);
      state.pendingButtonFlags |= REMOTE_BUTTON_DOWN;
    }
  }

  (void)setPressed;
  (void)upReleased;
  (void)downReleased;
}

}  // namespace buttons
