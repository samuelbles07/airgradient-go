# cap1203

Simple usage:

```cpp
#include "cap1203.h"

CAP1203 touch(i2c_bus);
ESP_ERROR_CHECK(touch.init());

uint8_t status = 0;
ESP_ERROR_CHECK(touch.readSensorInputStatus(&status));
```
