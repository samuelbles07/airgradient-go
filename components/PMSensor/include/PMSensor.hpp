#ifndef PM_SENSOR_HPP
#define PM_SENSOR_HPP

#include "MeasuresTypes.h"

class PMSensor {
public:
  virtual ~PMSensor() = default;

  virtual bool init() = 0;
  virtual bool read(PMData &out) = 0;

  // Optional lifecycle hook for drivers that hold resources.
  virtual void deinit() {}

  // Convenience helper.
  virtual bool reinit() {
    deinit();
    return init();
  }

  virtual bool support_temp_hum() const { return false; }
  virtual TempHumData temp_hum_data() = 0;

private:
};
#endif // !PM_SENSOR_HPP
