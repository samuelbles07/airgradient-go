/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#ifndef STCC4_SENSOR_HPP
#define STCC4_SENSOR_HPP

#include "CO2Sensor.hpp"

#include "driver/i2c_master.h"
#include <stdint.h>

extern "C" {
#include "stcc4.h"
}

class STCC4Sensor : public CO2Sensor {
public:
  explicit STCC4Sensor(i2c_master_bus_handle_t bus_handle,
                       uint8_t i2c_addr = STCC4_I2C_ADDR_DEFAULT);
  ~STCC4Sensor() override;

  bool init() override;
  bool read(CO2Data &out) override;

  bool support_temp_hum() const override { return true; }
  TempHumData temp_hum_data() override;

  bool support_force_calibration() const override { return true; }
  bool force_calibration(uint16_t target_ppm = 400) override;

private:
  const char *const TAG = "STCC4Sensor";

  i2c_master_bus_handle_t bus_handle_;
  uint8_t i2c_addr_;
  stcc4_dev_t dev_;

  TempHumData last_temp_hum_;
  bool have_temp_hum_;
};

#endif // STCC4_SENSOR_HPP
