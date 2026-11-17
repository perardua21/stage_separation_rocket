#pragma once

#include <Adafruit_BNO08x.h>

struct Bno085Reading {
  uint32_t sampleMs = 0;
  float accelXMps2 = 0.0f;
  float accelYMps2 = 0.0f;
  float accelZMps2 = 0.0f;
  float linearAccelXMps2 = 0.0f;
  float linearAccelYMps2 = 0.0f;
  float linearAccelZMps2 = 0.0f;
  float quaternionW = 1.0f;
  float quaternionX = 0.0f;
  float quaternionY = 0.0f;
  float quaternionZ = 0.0f;
  bool orientationValid = false;
};

class ILowGImu {
 public:
  virtual ~ILowGImu() = default;
  virtual bool begin() = 0;
  virtual bool read(Bno085Reading& reading, uint32_t nowMs) = 0;
};

class Bno085Hal final : public ILowGImu {
 public:
  Bno085Hal();
  bool begin() override;
  bool read(Bno085Reading& reading, uint32_t nowMs) override;

 private:
  bool enableReports();

  Adafruit_BNO08x device_;
  Bno085Reading latest_;
};
