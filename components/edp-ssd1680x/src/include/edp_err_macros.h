#ifndef EDP_ERR_MACROS_H
#define EDP_ERR_MACROS_H

#define EDP_RETURN_ON_ERROR(expr)        \
  do {                              \
    esp_err_t _err = (expr);         \
    if (_err != ESP_OK) return _err; \
  } while (0)

#define EDP_RETURN_IF_ERROR(err)           \
  do {                                \
    if ((err) != ESP_OK) return (err); \
  } while (0)

#define EDP_GOTO_ON_ERROR(expr, label) \
  do {                             \
    err = (expr);                  \
    if (err != ESP_OK) goto label; \
  } while (0)

#endif // EDP_ERR_MACROS_H
