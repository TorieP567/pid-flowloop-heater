#ifndef DISPLAY_H
#define DISPLAY_H

#include "types.h"

// TFT rendering for the production remote main and debug screens.
namespace display {

// Initializes the ST7789 and marks the screen for a full redraw.
void init(DashboardState& state);

// Refreshes the active screen on the configured display interval.
void update(DashboardState& state);

}  // namespace display

#endif  // DISPLAY_H
