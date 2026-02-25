#ifndef CO2_SENSOR_HPP
#define CO2_SENSOR_HPP

#include <stdint.h>

#include "MeasuresTypes.h"

class CO2Sensor {
public:
  virtual ~CO2Sensor() = default;

  virtual bool init() = 0;
  virtual bool read(CO2Data &out) = 0;

  virtual bool support_temp_hum() const { return false; }
  virtual TempHumData temp_hum_data() = 0;

  // Optional calibration support.
  // Implementations that don't support it can ignore.
  virtual bool support_force_calibration() const { return false; }
  virtual bool force_calibration(uint16_t target_ppm = 400) {
    (void)target_ppm;
    return false;
  }

private:
};

#endif // !CO2_SENSOR_HPP
