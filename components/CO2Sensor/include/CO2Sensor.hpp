#ifndef CO2_SENSOR_HPP
#define CO2_SENSOR_HPP

#include "MeasuresTypes.h"

class CO2Sensor {
public:
  virtual ~CO2Sensor() = default;

  virtual bool init() = 0;
  virtual bool read(CO2Data &out) = 0;

  virtual bool support_temp_hum() const { return false; }
  virtual TempHumData temp_hum_data() = 0;

private:
};

#endif // !CO2_SENSOR_HPP
