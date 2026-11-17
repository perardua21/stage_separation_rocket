#include <Arduino.h>

#include "app/flight_controller_app.h"

#if !defined(NURA_BENCH_SAFE)
#error "Flight outputs are unavailable until physical interlocks and hardware validation are complete."
#endif

namespace {
FlightControllerApp app;
}

void setup() { app.setup(millis()); }

void loop() { app.loop(millis()); }
