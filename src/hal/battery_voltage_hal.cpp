#include "hal/battery_voltage_hal.h"

#include <Arduino.h>

#include "board_pins.h"

void BatteryVoltageHal::begin() { pinMode(BoardPins::kBatteryAdc, INPUT); }

bool BatteryVoltageHal::read(float& voltageV) {
  voltageV = (analogReadMilliVolts(BoardPins::kBatteryAdc) / 1000.0f) *
             BoardPins::kBatteryDividerRatio;
  return true;
}
