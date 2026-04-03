#ifndef BUTTONS_H
#define BUTTONS_H

#include "types.h"

// Debounced button handling for tank selection, edit-mode toggling, setpoint
// adjustment, and the local main/debug screen toggle combo.
namespace buttons {

// Configures the three INPUT_PULLUP buttons and seeds the ButtonTracker state.
void init(DashboardState& state);

// Polls the buttons, updates UI state, and accumulates button flags for the
// next RemoteToMainPacket.
void update(DashboardState& state);

}  // namespace buttons

#endif  // BUTTONS_H
