#pragma once

class IBatteryVoltage {
 public:
  virtual ~IBatteryVoltage() = default;
  virtual void begin() = 0;
  virtual bool read(float& voltageV) = 0;
};

class BatteryVoltageHal final : public IBatteryVoltage {
 public:
  void begin() override;
  bool read(float& voltageV) override;
};
