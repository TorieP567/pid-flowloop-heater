#line 1 "C:\\Users\\18tor\\Downloads\\MechResearch\\pid-flowloop-heater\\nanoradio\\serial_log.h"
#ifndef SERIAL_LOG_H
#define SERIAL_LOG_H

#include "types.h"

// Optional serial diagnostics for link transitions and runtime summaries.
namespace serial_log {

// Seeds serial-log state and emits the startup banner when logging is enabled.
void init(DashboardState& state);

// Emits link-transition and summary logs when NANORADIO_ENABLE_SERIAL_LOG != 0.
void update(DashboardState& state);

}  // namespace serial_log

#endif  // SERIAL_LOG_H
