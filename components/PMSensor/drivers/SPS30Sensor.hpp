/*
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#ifndef SPS30_SENSOR_HPP
#define SPS30_SENSOR_HPP

#include "MeasuresTypes.h"
#include "PMSensor.hpp"

extern "C" {
#include "sps30.h"
}

#include <stdint.h>

class SPS30Sensor : public PMSensor {
public:
  explicit SPS30Sensor(i2c_master_bus_handle_t bus_handle);
  ~SPS30Sensor() override;

  bool init() override;
  bool read(PMData &out) override;

  void deinit() override;

  bool support_temp_hum() const override { return false; }
  TempHumData temp_hum_data() override;

private:
  const char *const TAG = "SPS30Sensor";
  i2c_master_bus_handle_t bus_handle_;
  sps30_handle_t handle_;
};

#endif // SPS30_SENSOR_HPP
