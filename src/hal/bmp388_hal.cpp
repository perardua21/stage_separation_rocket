#include "hal/bmp388_hal.h"

#include <Wire.h>

#include "board_pins.h"

namespace { constexpr float kSeaLevelPressureHpa = 1013.25f; }

bool Bmp388Hal::begin() {
  Wire.begin(BoardPins::kBmpSda, BoardPins::kBmpScl, 400000);
  if (!device_.begin_I2C(0x77, &Wire)) return false;
  chipId_ = device_.chipID();
  // BMP388 reports chip ID 0x50. Reject a BMP390 (0x60) so an unintended
  // sensor substitution cannot silently change the flight configuration.
  if (chipId_ != BMP3_CHIP_ID) return false;
  device_.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  device_.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  device_.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  device_.setOutputDataRate(BMP3_ODR_25_HZ);
  return true;
}

bool Bmp388Hal::read(Bmp388Reading& reading, uint32_t nowMs) {
  if (!device_.performReading()) return false;
  reading.sampleMs = nowMs;
  reading.pressurePa = device_.pressure;
  reading.temperatureC = device_.temperature;
  reading.absoluteAltitudeM = device_.readAltitude(kSeaLevelPressureHpa);
  return true;
}
