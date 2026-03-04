#pragma once

#include <esp_err.h>

#define RETURN_ON_ERROR(expr)                                                                      \
  do {                                                                                             \
    const esp_err_t _err = (expr);                                                                 \
    if (_err != ESP_OK) {                                                                          \
      return _err;                                                                                 \
    }                                                                                              \
  } while (0)

#define RETURN_IF_ERROR(err)                                                                       \
  do {                                                                                             \
    if ((err) != ESP_OK) {                                                                         \
      return (err);                                                                                \
    }                                                                                              \
  } while (0)

#define GOTO_ON_ERROR(expr, label)                                                                 \
  do {                                                                                             \
    err = (expr);                                                                                  \
    if (err != ESP_OK) {                                                                           \
      goto label;                                                                                  \
    }                                                                                              \
  } while (0)
