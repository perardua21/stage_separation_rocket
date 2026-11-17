#pragma once

#include <Adafruit_BMP3XX.h>

struct Bmp388Reading {
  uint32_t sampleMs = 0;
  float pressurePa = 0.0f;
  float temperatureC = 0.0f;
  float absoluteAltitudeM = 0.0f;
};

class IBarometer {
 public:
  virtual ~IBarometer() = default;
  virtual bool begin() = 0;
  virtual bool read(Bmp388Reading& reading, uint32_t nowMs) = 0;
};

class Bmp388Hal final : public IBarometer {
 public:
  bool begin() override;
  bool read(Bmp388Reading& reading, uint32_t nowMs) override;
  uint8_t chipId() const { return chipId_; }

 private:
  Adafruit_BMP3XX device_;
  uint8_t chipId_ = 0;
};
