#ifndef TVOC_NOX_SENSOR_HPP
#define TVOC_NOX_SENSOR_HPP

#include "MeasuresTypes.h"

class TVOCNOxSensor {
public:
  virtual ~TVOCNOxSensor() = default;

  virtual bool init() = 0;
  virtual bool read(TVOCNOxData &out) = 0;

private:
};

#endif // !TVOC_NOX_SENSOR_HPP
