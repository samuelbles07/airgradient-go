#ifndef AIRGRADIENT_GO_MAIN_UTILS_HPP
#define AIRGRADIENT_GO_MAIN_UTILS_HPP

#include <limits.h>
#include <math.h>
#include <stdint.h>

#include "MeasuresTypes.h"
#include "cJSON.h"

namespace go_utils {

static inline int32_t deg_to_e7(double deg) {
  return (int32_t)llround(deg * 10000000.0);
}

static inline uint16_t pm_ugm3_to_x10(float ugm3) {
  if (!(ugm3 >= 0.0f)) {
    return 0xFFFF;
  }

  const int v = (int)lroundf(ugm3 * 10.0f);
  if (v < 0) {
    return 0;
  }
  if (v > 65534) {
    return 65534;
  }
  return (uint16_t)v;
}

static inline uint32_t count_to_x10(float v) {
  if (!(v >= 0.0f)) {
    return 0xFFFFFFFFu;
  }
  const double dv = (double)v * 10.0;
  if (!(dv >= 0.0)) {
    return 0xFFFFFFFFu;
  }
  const uint64_t iv = (uint64_t)llround(dv);
  if (iv >= 0xFFFFFFFFu) {
    return 0xFFFFFFFEu;
  }
  return (uint32_t)iv;
}

static inline int16_t temp_c_to_x100(float c) {
  if (!(c >= MeasuresRange::MIN_VALID_TEMP && c <= MeasuresRange::MAX_VALID_TEMP)) {
    return (int16_t)INT16_MIN;
  }
  const int v = (int)lroundf(c * 100.0f);
  if (v < (int)INT16_MIN) {
    return (int16_t)INT16_MIN;
  }
  if (v > (int)INT16_MAX) {
    return (int16_t)INT16_MAX;
  }
  return (int16_t)v;
}

static inline uint16_t hum_rh_to_x100(float rh) {
  if (!(rh >= MeasuresRange::MIN_VALID_HUM && rh <= MeasuresRange::MAX_VALID_HUM)) {
    return 0xFFFF;
  }
  const int v = (int)lroundf(rh * 100.0f);
  if (v < 0) {
    return 0;
  }
  if (v > 65534) {
    return 65534;
  }
  return (uint16_t)v;
}

static inline uint16_t u16_from_int_nonneg(int v) {
  if (v < 0) {
    return 0xFFFF;
  }
  if (v > 65534) {
    return 65534;
  }
  return (uint16_t)v;
}

static inline uint32_t pressure_pa_from_float(float pa) {
  if (!(pa >= 0.0f)) {
    return 0xFFFFFFFFu;
  }
  const double dv = (double)pa;
  const uint64_t iv = (uint64_t)llround(dv);
  if (iv >= 0xFFFFFFFFu) {
    return 0xFFFFFFFEu;
  }
  return (uint32_t)iv;
}

static inline void json_add_u16_x10_if_valid(cJSON *obj, const char *key, uint16_t x10) {
  if (x10 == 0xFFFF) {
    return;
  }
  cJSON_AddNumberToObject(obj, key, (double)x10 / 10.0);
}

static inline void json_add_u32_x10_if_valid(cJSON *obj, const char *key, uint32_t x10) {
  if (x10 == 0xFFFFFFFFu) {
    return;
  }
  cJSON_AddNumberToObject(obj, key, (double)x10 / 10.0);
}

static inline void json_add_i16_x100_if_valid(cJSON *obj, const char *key, int16_t x100) {
  if (x100 == (int16_t)INT16_MIN) {
    return;
  }
  cJSON_AddNumberToObject(obj, key, (double)x100 / 100.0);
}

static inline void json_add_u16_x100_if_valid(cJSON *obj, const char *key, uint16_t x100) {
  if (x100 == 0xFFFF) {
    return;
  }
  cJSON_AddNumberToObject(obj, key, (double)x100 / 100.0);
}

static inline void json_add_u16_if_valid(cJSON *obj, const char *key, uint16_t v) {
  if (v == 0xFFFF) {
    return;
  }
  cJSON_AddNumberToObject(obj, key, (double)v);
}

// Backwards-compatible names (callers should prefer *_if_valid).
static inline void json_add_u16_x10_or_null(cJSON *obj, const char *key, uint16_t x10) {
  json_add_u16_x10_if_valid(obj, key, x10);
}

static inline void json_add_u32_x10_or_null(cJSON *obj, const char *key, uint32_t x10) {
  json_add_u32_x10_if_valid(obj, key, x10);
}

static inline void json_add_i16_x100_or_null(cJSON *obj, const char *key, int16_t x100) {
  json_add_i16_x100_if_valid(obj, key, x100);
}

static inline void json_add_u16_x100_or_null(cJSON *obj, const char *key, uint16_t x100) {
  json_add_u16_x100_if_valid(obj, key, x100);
}

static inline void json_add_u16_or_null(cJSON *obj, const char *key, uint16_t v) {
  json_add_u16_if_valid(obj, key, v);
}

} // namespace go_utils

#endif // AIRGRADIENT_GO_MAIN_UTILS_HPP
