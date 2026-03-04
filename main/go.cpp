
#include <stdint.h>

#include "esp_mac.h"
#include <memory>
#include <new>
#include <string>
#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_http_client.h"
#include "esp_console.h"
#include "esp_vfs_fat.h"
#include "cJSON.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "button_service.h"
#include "gps_service.h"
#include "nand_storage_service.h"
#include "go_constants.h"
#include "utils.hpp"
#include "ble_stream.h"
#include "WiFiManager.h"
#include "bq25629.h"

#include "gdey0213b74.h"
#include "dashboard/dashboard.h"
#include "ui/dashboard_ui.h"

#include "PMSensor.hpp"
#include "SPS30Sensor.hpp"

#include "SGP41.hpp"

#include "STCC4Sensor.hpp"

#include "dps368.h"

// embedded-i2c-scd4x (test-only)
extern "C" {
#include "scd4x_i2c.h"
#include "sensirion_i2c_hal.h"
#include "sensirion_i2c_hal_esp_idf.h"
}

// Senseair I2C CO2 (test-only)
#include "s12_i2c.h"
#include "sunrise_i2c.h"

// NOTE: Temporary constants
#define NO_INACTIVE_NO_SLEEP 1
#define TRACKING_DISPLAY_SLEEP 0

enum class State {
  Idle = 0,
  Inactive,
  Sync,
  Tracking,
  Shutdown,
};

struct Scd4xTest {
  bool initialized = false;
};

RTC_DATA_ATTR static State RTC_LAST_STATE = State::Idle;
RTC_DATA_ATTR static uint32_t RTC_TRACKING_SESSION_ID = 0;
RTC_DATA_ATTR static uint32_t RTC_TRACKING_SLEEP_INTERVAL_S = GO_TRACKING_SLEEP_INTERVAL_S;
static WiFiManager g_wifiManager;

static bool is_valid_rtc_state(State s) {
  switch (s) {
  case State::Idle:
  case State::Inactive:
  case State::Sync:
  case State::Tracking:
  case State::Shutdown:
    return true;
  }
  return false;
}

struct Inputs {
  bool button_short = false;
  bool button_long = false;
  bool boot_long = false;
  bool touch_right_long = false;
  bool touch_left_long = false;
  bool touch_enter_long = false;
};

enum class GoInputEventType : uint8_t {
  ButtonShort = 1,
  ButtonLong = 2,
  BootLong = 3,
  TouchRightLong = 4,
  TouchLeftLong = 5,
  TouchEnterLong = 6,
};

struct GoInputEvent {
  GoInputEventType type;
};

static inline uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

static inline void sleep_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

static esp_err_t init_ext_watchdog(void) {
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL << GO_WDT_GPIO);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  esp_err_t err = gpio_config(&io_conf);
  if (err != ESP_OK) {
    return err;
  }
  (void)gpio_set_level(GO_WDT_GPIO, 0);
  return ESP_OK;
}

static void reset_ext_watchdog(void) {
  (void)gpio_set_level(GO_WDT_GPIO, 1);
  sleep_ms(GO_WDT_RESET_PULSE_MS);
  (void)gpio_set_level(GO_WDT_GPIO, 0);
}

// Forward declaration (defined later in this file).
static void format_rfc3339_utc(uint64_t epoch_ms, char *out, size_t out_len);
static const char *state_name(State s);

static std::string build_ble_measure_payload(const NandStorageService::Record &r) {
  auto trim_float_str = [](char *buf) {
    if (buf == nullptr) {
      return;
    }
    size_t n = strlen(buf);
    while (n > 0 && buf[n - 1] == '0') {
      buf[n - 1] = '\0';
      n--;
    }
    if (n > 0 && buf[n - 1] == '.') {
      buf[n - 1] = '\0';
    }
  };

  auto append_field = [](std::string &out, const char *s) {
    if (s != nullptr) {
      out.append(s);
    }
    out.push_back(';');
  };

  auto append_empty = [&](std::string &out) { append_field(out, nullptr); };

  auto append_u64 = [&](std::string &out, uint64_t v) {
    char buf[24];
    (void)snprintf(buf, sizeof(buf), "%" PRIu64, v);
    append_field(out, buf);
  };
  auto append_u32 = [&](std::string &out, uint32_t v) {
    char buf[16];
    (void)snprintf(buf, sizeof(buf), "%" PRIu32, v);
    append_field(out, buf);
  };
  auto append_u16 = [&](std::string &out, uint16_t v) {
    char buf[8];
    (void)snprintf(buf, sizeof(buf), "%u", (unsigned)v);
    append_field(out, buf);
  };
  auto append_i16 = [&](std::string &out, int16_t v) {
    char buf[8];
    (void)snprintf(buf, sizeof(buf), "%d", (int)v);
    append_field(out, buf);
  };

  // Measures payload: positional semicolon-separated fields (fixed order).
  // Invalid values are emitted as empty fields.
  // Format:
  // ts_ms;lat;lng;pm01_x10;pm25_x10;pm10_x10;pc05_x10;pc10_x10;pc25_x10;pc100_x10;
  // rco2_ppm;scd4x_ppm;atmp_c_x100;rhum_x100;pres_pa;tvoc_raw;nox_raw;s12_ppm;sunlight_ppm;
  std::string out;
  out.reserve(216);

  if (r.timestamp_ms != 0) {
    append_u64(out, r.timestamp_ms);
  } else {
    append_empty(out);
  }

  if (r.latitude_e7 != INT32_MIN) {
    char buf[24];
    (void)snprintf(buf, sizeof(buf), "%.7f", (double)r.latitude_e7 / 10000000.0);
    trim_float_str(buf);
    append_field(out, buf);
  } else {
    append_empty(out);
  }

  if (r.longitude_e7 != INT32_MIN) {
    char buf[24];
    (void)snprintf(buf, sizeof(buf), "%.7f", (double)r.longitude_e7 / 10000000.0);
    trim_float_str(buf);
    append_field(out, buf);
  } else {
    append_empty(out);
  }

  if (r.pm01_ugm3_x10 != 0xFFFF) {
    append_u16(out, r.pm01_ugm3_x10);
  } else {
    append_empty(out);
  }
  if (r.pm25_ugm3_x10 != 0xFFFF) {
    append_u16(out, r.pm25_ugm3_x10);
  } else {
    append_empty(out);
  }
  if (r.pm10_ugm3_x10 != 0xFFFF) {
    append_u16(out, r.pm10_ugm3_x10);
  } else {
    append_empty(out);
  }

  if (r.pc05_x10 != 0xFFFFFFFFu) {
    append_u32(out, r.pc05_x10);
  } else {
    append_empty(out);
  }
  if (r.pc10_x10 != 0xFFFFFFFFu) {
    append_u32(out, r.pc10_x10);
  } else {
    append_empty(out);
  }
  if (r.pc25_x10 != 0xFFFFFFFFu) {
    append_u32(out, r.pc25_x10);
  } else {
    append_empty(out);
  }
  if (r.pc100_x10 != 0xFFFFFFFFu) {
    append_u32(out, r.pc100_x10);
  } else {
    append_empty(out);
  }

  if (r.co2_ppm != 0xFFFF) {
    append_u16(out, r.co2_ppm);
  } else {
    append_empty(out);
  }
  if (r.scd4x != 0xFFFF) {
    append_u16(out, r.scd4x);
  } else {
    append_empty(out);
  }

  if (r.temperature_c_x100 != (int16_t)INT16_MIN) {
    append_i16(out, r.temperature_c_x100);
  } else {
    append_empty(out);
  }
  if (r.humidity_rh_x100 != 0xFFFF) {
    append_u16(out, r.humidity_rh_x100);
  } else {
    append_empty(out);
  }

  if (r.pressure_pa != 0xFFFFFFFFu) {
    append_u32(out, r.pressure_pa);
  } else {
    append_empty(out);
  }

  if (r.tvoc_raw != 0xFFFF) {
    append_u16(out, r.tvoc_raw);
  } else {
    append_empty(out);
  }
  if (r.nox_raw != 0xFFFF) {
    append_u16(out, r.nox_raw);
  } else {
    append_empty(out);
  }

  if (r.s12 != 0xFFFF) {
    append_u16(out, r.s12);
  } else {
    append_empty(out);
  }

  if (r.sunlight != 0xFFFF) {
    append_u16(out, r.sunlight);
  } else {
    append_empty(out);
  }

  return out;
}

static std::string build_ble_history_payload(const NandStorageService::Record &r, bool last) {
  // History payload prefixes measures payload with route id and last flag.
  // Format:
  // route_id;last;ts_ms;lat;lng;pm01_x10;pm25_x10;pm10_x10;pc05_x10;pc10_x10;pc25_x10;pc100_x10;
  // rco2_ppm;scd4x_ppm;atmp_c_x100;rhum_x100;pres_pa;tvoc_raw;nox_raw;s12_ppm;sunlight_ppm;
  std::string out;
  out.reserve(256);

  char buf[16];
  (void)snprintf(buf, sizeof(buf), "%" PRIu32, r.id);
  out.append(buf);
  out.push_back(';');
  out.push_back(last ? '1' : '0');
  out.push_back(';');

  out.append(build_ble_measure_payload(r));
  return out;
}

static std::string build_ble_status_payload(State s,
                                            bool gps_ok,
                                            const GPSService::Data &gps,
                                            bool battery_ok,
                                            int battery_percent,
                                            bool flash_ok,
                                            uint32_t flash_avail_kb,
                                            bool have_charging,
                                            bool charging,
                                            uint32_t route_id,
                                            uint32_t tracking_sleep_s,
                                            bool co2_calibrating) {
  cJSON *root = cJSON_CreateObject();
  if (root == nullptr) {
    return {};
  }

  cJSON_AddStringToObject(root, "state", state_name(s));
  cJSON_AddNumberToObject(root, "trackingSleepS", (double)tracking_sleep_s);
  if (co2_calibrating) {
    cJSON_AddBoolToObject(root, "co2Calibrating", true);
  }
  if (s == State::Tracking) {
    cJSON_AddNumberToObject(root, "route", (double)route_id);
  }
  if (gps_ok) {
    cJSON_AddBoolToObject(root, "gps_fix", gps.fix_valid);
    cJSON_AddNumberToObject(root, "gps_sats", (double)gps.satellites);
  }
  if (battery_ok) {
    cJSON_AddNumberToObject(root, "battery_percent", (double)battery_percent);
  }
  if (flash_ok) {
    cJSON_AddNumberToObject(root, "flashAvail", (double)flash_avail_kb);
  }
  if (have_charging) {
    cJSON_AddBoolToObject(root, "charging", charging);
  }

  char *json = cJSON_PrintUnformatted(root);
  std::string out;
  if (json != nullptr) {
    out.assign(json);
    cJSON_free(json);
  }
  cJSON_Delete(root);
  return out;
}

static std::string build_measures_payload(const NandStorageService::Record *recs,
                                          const uint64_t *ts_ms,
                                          uint32_t n) {
  if (recs == nullptr || ts_ms == nullptr || n == 0) {
    return {};
  }

  cJSON *root = cJSON_CreateObject();
  cJSON *arr = cJSON_CreateArray();
  cJSON_AddItemToObject(root, "measures", arr);

  for (uint32_t i = 0; i < n; ++i) {
    cJSON *m = cJSON_CreateObject();
    char date[32];
    format_rfc3339_utc(ts_ms[i], date, sizeof(date));
    cJSON_AddStringToObject(m, "date", date);

    if (recs[i].latitude_e7 != INT32_MIN && recs[i].longitude_e7 != INT32_MIN) {
      cJSON_AddNumberToObject(m, "lat", (double)recs[i].latitude_e7 / 10000000.0);
      cJSON_AddNumberToObject(m, "lng", (double)recs[i].longitude_e7 / 10000000.0);
    }

    // PM mass (atmospheric)
    go_utils::json_add_u16_x10_if_valid(m, "pm01", recs[i].pm01_ugm3_x10);
    go_utils::json_add_u16_x10_if_valid(m, "pm02", recs[i].pm25_ugm3_x10);
    go_utils::json_add_u16_x10_if_valid(m, "pm10", recs[i].pm10_ugm3_x10);

    // PM counts / bins
    go_utils::json_add_u32_x10_if_valid(m, "pm005Count", recs[i].pc05_x10);
    go_utils::json_add_u32_x10_if_valid(m, "pm01Count", recs[i].pc10_x10);
    go_utils::json_add_u32_x10_if_valid(m, "pm02Count", recs[i].pc25_x10);
    go_utils::json_add_u32_x10_if_valid(m, "pm10Count", recs[i].pc100_x10);

    // Temporary server mapping: SCD4x CO2 is posted as pm003Count
    go_utils::json_add_u16_if_valid(m, "pm003Count", recs[i].scd4x);

    // Senseair I2C CO2 (temporary field names).
    go_utils::json_add_u16_if_valid(m, "s12", recs[i].s12);
    go_utils::json_add_u16_if_valid(m, "sunlight", recs[i].sunlight);

    go_utils::json_add_u16_if_valid(m, "rco2", recs[i].co2_ppm);
    go_utils::json_add_i16_x100_if_valid(m, "atmp", recs[i].temperature_c_x100);
    go_utils::json_add_u16_x100_if_valid(m, "rhum", recs[i].humidity_rh_x100);

    if (recs[i].pressure_pa != 0xFFFFFFFFu) {
      cJSON_AddNumberToObject(m, "pres", (double)recs[i].pressure_pa / 100.0);
    }

    go_utils::json_add_u16_if_valid(m, "tvocRaw", recs[i].tvoc_raw);
    go_utils::json_add_u16_if_valid(m, "noxRaw", recs[i].nox_raw);

    cJSON_AddNumberToObject(m, "route", (double)recs[i].id);
    cJSON_AddItemToArray(arr, m);
  }

  char *json = cJSON_PrintUnformatted(root);
  std::string payload;
  if (json != nullptr) {
    payload.assign(json);
    cJSON_free(json);
  }
  cJSON_Delete(root);
  return payload;
}

static bool utc_to_epoch_ms(const GPSService::UtcTime &utc, uint64_t *out_ms) {
  if (out_ms == nullptr) {
    return false;
  }
  *out_ms = 0;
  if (!utc.date_valid || !utc.time_valid) {
    return false;
  }

  // Basic range validation.
  if (utc.year < 1970 || utc.month < 1 || utc.month > 12 || utc.day < 1 || utc.day > 31) {
    return false;
  }
  if (utc.hour < 0 || utc.hour > 23 || utc.min < 0 || utc.min > 59 || utc.sec < 0 || utc.sec > 59) {
    return false;
  }

  // days_from_civil() (Howard Hinnant) adapted for UTC.
  int y = utc.year;
  unsigned m = (unsigned)utc.month;
  unsigned d = (unsigned)utc.day;

  if (m <= 2) {
    y -= 1;
  }

  int era = 0;
  if (y >= 0) {
    era = y / 400;
  } else {
    era = (y - 399) / 400;
  }

  const unsigned yoe = (unsigned)(y - era * 400);

  unsigned mp = 0;
  if (m > 2) {
    mp = m - 3;
  } else {
    mp = m + 9;
  }
  const unsigned doy = (153U * mp + 2U) / 5U + d - 1U;

  const unsigned doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;
  const int64_t days = (int64_t)era * 146097 + (int64_t)doe - 719468;

  const int64_t sec =
      days * 86400LL + (int64_t)utc.hour * 3600LL + (int64_t)utc.min * 60LL + (int64_t)utc.sec;
  if (sec < 0) {
    return false;
  }
  *out_ms = (uint64_t)sec * 1000ULL;
  return true;
}

static void format_rfc3339_utc(uint64_t epoch_ms, char *out, size_t out_len) {
  if (out == nullptr || out_len == 0) {
    return;
  }
  out[0] = '\0';

  const time_t sec = (time_t)(epoch_ms / 1000ULL);
  struct tm tm_utc;
  memset(&tm_utc, 0, sizeof(tm_utc));
  if (gmtime_r(&sec, &tm_utc) == nullptr) {
    (void)snprintf(out, out_len, "1970-01-01T00:00:00Z");
    return;
  }

  (void)snprintf(out, out_len, "%04d-%02d-%02dT%02d:%02d:%02dZ", tm_utc.tm_year + 1900,
                 tm_utc.tm_mon + 1, tm_utc.tm_mday, tm_utc.tm_hour, tm_utc.tm_min, tm_utc.tm_sec);
}

static const char *state_name(State s) {
  switch (s) {
  case State::Idle:
    return "IDLE";
  case State::Inactive:
    return "INACTIVE";
  case State::Sync:
    return "SYNC";
  case State::Tracking:
    return "TRACKING";
  case State::Shutdown:
    return "SHUTDOWN";
  }
  return "UNKNOWN";
}

static const char *vbus_status_name(drivers::VBusStatus s) {
  switch (s) {
  case drivers::VBusStatus::NO_ADAPTER:
    return "NO_ADAPTER";
  case drivers::VBusStatus::USB_SDP:
    return "USB_SDP";
  case drivers::VBusStatus::USB_CDP:
    return "USB_CDP";
  case drivers::VBusStatus::USB_DCP:
    return "USB_DCP";
  case drivers::VBusStatus::UNKNOWN_ADAPTER:
    return "UNKNOWN_ADAPTER";
  case drivers::VBusStatus::NON_STANDARD:
    return "NON_STANDARD";
  case drivers::VBusStatus::OTG_MODE:
    return "OTG_MODE";
  }
  return "UNDEFINED";
}

static const char *charge_status_name(drivers::ChargeStatus s) {
  switch (s) {
  case drivers::ChargeStatus::NOT_CHARGING:
    return "NOT_CHARGING";
  case drivers::ChargeStatus::TRICKLE_PRECHARGE_FASTCHARGE:
    return "PRECHARGE_OR_FAST";
  case drivers::ChargeStatus::TAPER_CHARGE:
    return "TAPER";
  case drivers::ChargeStatus::TOPOFF_TIMER_ACTIVE:
    return "TOPOFF";
  }
  return "UNKNOWN";
}

static void log_bq25629_debug_snapshot(drivers::BQ25629 *charger) {
  if (charger == nullptr) {
    return;
  }

  uint8_t reg16 = 0;
  esp_err_t err = charger->read_register(drivers::BQ25629_REG::CHARGER_CONTROL_0, reg16);
  if (err != ESP_OK) {
    ESP_LOGW(GO_TAG, "BQ25629 read REG0x16 failed: %s", esp_err_to_name(err));
    return;
  }

  uint8_t reg18 = 0;
  err = charger->read_register(drivers::BQ25629_REG::CHARGER_CONTROL_2, reg18);
  if (err != ESP_OK) {
    ESP_LOGW(GO_TAG, "BQ25629 read REG0x18 failed: %s", esp_err_to_name(err));
    return;
  }

  drivers::BQ25629_Status status = {};
  err = charger->read_status(status);
  if (err != ESP_OK) {
    ESP_LOGW(GO_TAG, "BQ25629 read_status failed: %s", esp_err_to_name(err));
    return;
  }

  drivers::BQ25629_Fault fault = {};
  err = charger->read_fault(fault);
  if (err != ESP_OK) {
    ESP_LOGW(GO_TAG, "BQ25629 read_fault failed: %s", esp_err_to_name(err));
    return;
  }

  drivers::BQ25629_ADC_Data adc = {};
  err = charger->read_adc(adc);
  if (err != ESP_OK) {
    ESP_LOGW(GO_TAG, "BQ25629 read_adc failed: %s", esp_err_to_name(err));
    return;
  }

  const bool en_chg = (reg16 & (1U << 5)) != 0;
  const bool en_hiz = (reg16 & (1U << 4)) != 0;
  const bool force_pmid_dis = (reg16 & (1U << 3)) != 0;
  const bool force_ibatdis = (reg16 & (1U << 6)) != 0;
  const bool wd_rst = (reg16 & (1U << 2)) != 0;
  const bool en_otg = (reg18 & (1U << 6)) != 0;
  const unsigned batfet_ctrl = (unsigned)(reg18 & 0x03U);

  ESP_LOGI("app_main",
           "BQ25629 reg - REG0x16: 0x%02X (FORCE_IBATDIS:%u EN_CHG:%u EN_HIZ:%u "
           "FORCE_PMID_DIS:%u WD_RST:%u) "
           "REG0x18: 0x%02X (EN_OTG:%u BATFET_CTRL:%u)",
           (unsigned)reg16, (unsigned)force_ibatdis, (unsigned)en_chg, (unsigned)en_hiz,
           (unsigned)force_pmid_dis, (unsigned)wd_rst, (unsigned)reg18, (unsigned)en_otg, batfet_ctrl);

  ESP_LOGI("app_main",
           "BQ25629 dbg - VBUS: %u, CHG: %u (%s), IOTG_REG: %u, VOTG_REG: %u, WD: %u",
           (unsigned)status.vbus_status, (unsigned)status.charge_status,
           charge_status_name(status.charge_status), (unsigned)status.iindpm_stat,
           (unsigned)status.vindpm_stat, (unsigned)status.wd_stat);

  ESP_LOGI("app_main", "BQ25629 fault - OTG: %u, TSHUT: %u, TS_STAT: %u, VBUS: %u, BAT: %u, SYS: %u",
           (unsigned)fault.otg_fault, (unsigned)fault.tshut_fault, (unsigned)fault.ts_stat,
           (unsigned)fault.vbus_fault, (unsigned)fault.bat_fault, (unsigned)fault.sys_fault);

  ESP_LOGI("app_main",
           "BQ25629 adc - VPMID: %u mV, VBAT: %u mV, VSYS: %u mV, VBUS: %u mV, IBAT: %d mA, IBUS: %d mA",
           (unsigned)adc.vpmid_mv, (unsigned)adc.vbat_mv, (unsigned)adc.vsys_mv, (unsigned)adc.vbus_mv,
           (int)adc.ibat_ma, (int)adc.ibus_ma);
}

static void log_sensor_summary_banner(void) {
  ESP_LOGI("app_main",
           "\xE2\x94\x81\xE2\x94\x81\xE2\x94\x81\xE2\x94\x81\xE2\x94\x81\xE2\x94\x81\xE2\x94\x81"
           "\xE2\x94\x81\xE2\x94\x81\xE2\x94\x81\xE2\x94\x81\xE2\x94\x81\xE2\x94\x81 SENSOR SUMMARY "
           "\xE2\x94\x81\xE2\x94\x81\xE2\x94\x81\xE2\x94\x81\xE2\x94\x81\xE2\x94\x81\xE2\x94\x81"
           "\xE2\x94\x81\xE2\x94\x81\xE2\x94\x81\xE2\x94\x81\xE2\x94\x81\xE2\x94\x81");
}

static bool is_usb_c_adapter_present(drivers::VBusStatus s) {
  switch (s) {
  case drivers::VBusStatus::USB_SDP:
  case drivers::VBusStatus::USB_CDP:
  case drivers::VBusStatus::USB_DCP:
  case drivers::VBusStatus::UNKNOWN_ADAPTER:
  case drivers::VBusStatus::NON_STANDARD:
    return true;
  case drivers::VBusStatus::NO_ADAPTER:
  case drivers::VBusStatus::OTG_MODE:
    return false;
  }
  return false;
}

static esp_err_t pm_power_on(void) {
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL << GO_PM_POWER_GPIO);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  esp_err_t err = gpio_config(&io_conf);
  if (err != ESP_OK) {
    return err;
  }

  (void)gpio_set_level(GO_PM_POWER_GPIO, GO_PM_POWER_ON_LEVEL);
  sleep_ms(GO_SPS30_POWER_STABILIZE_DELAY_MS);
  return ESP_OK;
}

static esp_err_t init_charger(i2c_master_bus_handle_t bus_handle, drivers::BQ25629 **out) {
  if (out == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }
  *out = nullptr;

  static drivers::BQ25629 charger(bus_handle);

  drivers::BQ25629_Config cfg = {
      .charge_voltage_mv = 4200,
      .charge_current_ma = 500,
      .input_current_limit_ma = 1500,
      .input_voltage_limit_mv = 4600,
      .min_system_voltage_mv = 3520,
      .precharge_current_ma = 30,
      .term_current_ma = 20,
      .enable_charging = true,
      .enable_otg = false,
      .enable_adc = true,
  };

  esp_err_t err = charger.init(cfg);
  if (err != ESP_OK) {
    return err;
  }

  // Enable EN_AUTO_IBATDIS (CHARGER_CONTROL_0 bit7) so the charger can automatically
  // enable IBAT discharge when appropriate.
  err = charger.enable_auto_ibat_discharge(true);
  if (err != ESP_OK) {
    ESP_LOGW(GO_TAG, "BQ25629 EN_AUTO_IBATDIS enable failed: %s", esp_err_to_name(err));
    // Best-effort: keep running.
  }

  err = charger.set_watchdog_timeout(drivers::WatchdogTimeout::Sec200);
  if (err != ESP_OK) {
    return err;
  }

  err = charger.enable_pmid_5v_boost();
  if (err != ESP_OK) {
    return err;
  }

  err = charger.reset_watchdog();
  if (err != ESP_OK) {
    return err;
  }

  ESP_LOGI(GO_TAG, "BQ25629 PMID boost enabled");
  *out = &charger;
  return ESP_OK;
}

static void log_gps_data(const GPSService::Data &d) {
  if (!d.has_sentence) {
    ESP_LOGI(GO_TAG, "gps: no sentence");
    return;
  }

  char time_buf[32];
  time_buf[0] = '\0';
  if (d.utc.date_valid && d.utc.time_valid) {
    (void)snprintf(time_buf, sizeof(time_buf), "%04d-%02d-%02d %02d:%02d:%02dZ", d.utc.year,
                   d.utc.month, d.utc.day, d.utc.hour, d.utc.min, d.utc.sec);
  } else if (d.utc.time_valid) {
    (void)snprintf(time_buf, sizeof(time_buf), "%02d:%02d:%02dZ", d.utc.hour, d.utc.min, d.utc.sec);
  } else {
    (void)snprintf(time_buf, sizeof(time_buf), "--");
  }

  if (d.fix_valid) {
    ESP_LOGI(
        GO_TAG, "gps: fix=1 q=%d sats=%d lat=%.6f lon=%.6f time=%s last_sentence=%" PRIu64 "ms",
        d.fix_quality, d.satellites, d.latitude_deg, d.longitude_deg, time_buf, d.last_sentence_ms);
  } else {
    ESP_LOGI(GO_TAG, "gps: fix=0 q=%d sats=%d time=%s last_sentence=%" PRIu64 "ms", d.fix_quality,
             d.satellites, time_buf, d.last_sentence_ms);
  }
}

static esp_err_t init_display(ui::DashboardUI **ui_out, ssd1680x::panels::GDEY0213B74 **epd_out) {
  if (ui_out == nullptr || epd_out == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }
  *ui_out = nullptr;
  *epd_out = nullptr;

  ssd1680x::Config cfg;
  cfg.host = GO_SPI_HOST;
  cfg.mirror_x = true;
  cfg.pins.busy = GO_EPD_BUSY_GPIO;
  cfg.pins.rst = GO_EPD_RST_GPIO;
  cfg.pins.dc = GO_EPD_DC_GPIO;
  cfg.pins.cs = GO_EPD_CS_GPIO;

  cfg.devcfg.clock_speed_hz = GO_EPD_CLOCK_SPEED_HZ;
  cfg.devcfg.mode = GO_EPD_SPI_MODE;
  cfg.devcfg.queue_size = GO_EPD_SPI_QUEUE_SIZE;
  cfg.devcfg.flags = GO_EPD_SPI_DEVICE_FLAGS;

  static ssd1680x::panels::GDEY0213B74 epd(cfg);
  static ui::DashboardUI ui(epd);

  esp_err_t err = ui.init();
  if (err != ESP_OK) {
    return err;
  }

  *ui_out = &ui;
  *epd_out = &epd;
  return ESP_OK;
}

static std::string buildSerialNumber() {
  uint8_t mac_address[6];
  esp_err_t err = esp_read_mac(mac_address, ESP_MAC_WIFI_STA);
  if (err != ESP_OK) {
    ESP_LOGE(GO_TAG, "Failed to get MAC address (%s)", esp_err_to_name(err));
    return {};
  }

  char result[13] = {0};
  snprintf(result, sizeof(result), "%02x%02x%02x%02x%02x%02x", mac_address[0], mac_address[1],
           mac_address[2], mac_address[3], mac_address[4], mac_address[5]);
  std::string sn = std::string(result);

  return sn;
}

static bool wifi_connect(const std::string &sn) {
  std::string ssid = std::string("airgradient-") + sn;
  if (g_wifiManager.autoConnect(ssid.c_str(), "cleanair") == false) {
    ESP_LOGE(GO_TAG, "Failed connect to WiFi");
    return false;
  }
  return true;
}

void wifi_disconnect() { g_wifiManager.disconnect(true); }

static bool post_request(const std::string &sn, const std::string &data) {
  esp_http_client_config_t config = {};
  char url[96] = {0};
  (void)snprintf(url, sizeof(url), "http://hw.airgradient.com/sensors/airgradient:%s/measures",
                 sn.c_str());
  config.url = url;
  config.method = HTTP_METHOD_POST;
  config.cert_pem = nullptr;
  config.timeout_ms = 10000;

  esp_http_client_handle_t client = esp_http_client_init(&config);
  if (client == nullptr) {
    ESP_LOGW(GO_TAG, "http client init failed");
    return false;
  }

  esp_http_client_set_header(client, "Content-Type", "application/json");
  esp_http_client_set_post_field(client, data.c_str(), data.length());

  const esp_err_t perr = esp_http_client_perform(client);
  if (perr != ESP_OK) {
    ESP_LOGW(GO_TAG, "http perform failed (%s)", esp_err_to_name(perr));
    esp_http_client_cleanup(client);
    return false;
  }

  const int responseCode = esp_http_client_get_status_code(client);
  if (responseCode != 200 && responseCode != 201) {
    static constexpr int MAX_LOG_BODY = 512;
    char body[MAX_LOG_BODY + 1];
    int total = 0;
    while (total < MAX_LOG_BODY) {
      const int n = esp_http_client_read(client, body + total, MAX_LOG_BODY - total);
      if (n <= 0) {
        break;
      }
      total += n;
    }
    body[total] = '\0';

    if (total > 0) {
      ESP_LOGW(GO_TAG, "http status=%d url=%s body=%s", responseCode, url, body);
    } else {
      ESP_LOGW(GO_TAG, "http status=%d url=%s (no body)", responseCode, url);
    }
  } else {
    ESP_LOGI(GO_TAG, "http status=%d", responseCode);
  }
  esp_http_client_cleanup(client);
  return (responseCode == 200 || responseCode == 201);
}

static void initConsole() {
  fflush(stdout);
  fsync(fileno(stdout));
  esp_console_deinit();

  /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
  usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
  /* Move the caret to the beginning of the next line on '\n' */
  usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

  /* Enable blocking mode on stdin and stdout */
  fcntl(fileno(stdout), F_SETFL, 0);
  fcntl(fileno(stdin), F_SETFL, 0);

  usb_serial_jtag_driver_config_t jtag_config = {
      .tx_buffer_size = 256,
      .rx_buffer_size = 256,
  };

  /* Install USB-SERIAL-JTAG driver for interrupt-driven reads and writes */
  ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&jtag_config));

  /* Tell vfs to use usb-serial-jtag driver */
  usb_serial_jtag_vfs_use_driver();

  /* Initialize the console */
  esp_console_config_t console_config = {};
  console_config.max_cmdline_length = CONSOLE_MAX_CMDLINE_LENGTH;
  console_config.max_cmdline_args = CONSOLE_MAX_CMDLINE_ARGS;
#if CONFIG_LOG_COLORS
  console_config.hint_color = atoi(LOG_COLOR_CYAN);
#endif
  console_config.hint_bold = 0;
  ESP_ERROR_CHECK(esp_console_init(&console_config));
}

class GoController {
public:
  GoController(ButtonService *buttons, QueueHandle_t input_queue, PMSensor *pm_sensor,
               TVOCNOxSensor *tvoc_nox_sensor, CO2Sensor *co2_sensor, dps368_handle_t *dps368,
               Scd4xTest *scd4x, s12_i2c_t *s12, sunrise_i2c_t *sunrise,
               BLEStream *ble,
               i2c_master_bus_handle_t i2c_bus, GPSService *gps, dashboard::Dashboard *dash,
               NandStorageService *storage,
               drivers::BQ25629 *charger, uint32_t last_wdt_reset_ms, uint32_t last_bq_wdt_reset_ms)
       : buttons_(buttons), input_queue_(input_queue), pm_sensor_(pm_sensor),
          tvoc_nox_sensor_(tvoc_nox_sensor), co2_sensor_(co2_sensor), dps368_(dps368), scd4x_(scd4x),
          s12_(s12), sunrise_(sunrise), ble_(ble),
          i2c_bus_(i2c_bus), gps_(gps),
          dash_(dash),
          storage_(storage), charger_(charger),
          last_wdt_reset_ms_(last_wdt_reset_ms), last_bq_wdt_reset_ms_(last_bq_wdt_reset_ms) {}

  void OnButtonEvent(int32_t id, const ButtonService::Payload *p) {
    if (p == nullptr) {
      return;
    }
    if (input_queue_ == nullptr) {
      return;
    }

    const ButtonService::Event ev = static_cast<ButtonService::Event>(id);
    if (p->source == ButtonService::Source::Physical) {
      if (p->id == 0) {
        if (ev == ButtonService::Event::ShortPress) {
          GoInputEvent e;
          e.type = GoInputEventType::ButtonShort;
          ESP_LOGI(GO_TAG, "event: QON short");
          (void)xQueueSend(input_queue_, &e, 0);
        } else if (ev == ButtonService::Event::LongPress) {
          GoInputEvent e;
          e.type = GoInputEventType::ButtonLong;
          ESP_LOGI(GO_TAG, "event: QON long");
          (void)xQueueSend(input_queue_, &e, 0);
        }
      } else if (p->id == 1) {
        if (ev == ButtonService::Event::LongPress) {
          GoInputEvent e;
          e.type = GoInputEventType::BootLong;
          ESP_LOGI(GO_TAG, "event: BOOT long");
          (void)xQueueSend(input_queue_, &e, 0);
        }
      }
      return;
    }

    if (p->source == ButtonService::Source::Touch) {
      if (ev != ButtonService::Event::LongPress) {
        return;
      }

      GoInputEvent e;
      if (p->id == GO_TOUCH_RIGHT_ID) {
        e.type = GoInputEventType::TouchRightLong;
        ESP_LOGI(GO_TAG, "event: touch right long");
        (void)xQueueSend(input_queue_, &e, 0);
      } else if (p->id == GO_TOUCH_ENTER_ID) {
        e.type = GoInputEventType::TouchEnterLong;
        ESP_LOGI(GO_TAG, "event: touch enter long");
        (void)xQueueSend(input_queue_, &e, 0);
      }
      return;
    }
  }

  void Run(void) {
    _init();
    while (true) {
      if (ble_ != nullptr && ble_->take_pending_history_start()) {
        if (_state == State::Idle) {
          _ble_history_export_run_blocking_();
        }
        // Requirement: ignore if not in IDLE.
      }

      _apply_ble_config_if_needed();
      _co2_force_calibrate_if_needed();
      _apply_ble_flash_erase_if_needed();
      _apply_ble_tracking_if_needed();
      _kick_watchdogs_if_needed();
      _poll_usb_c_if_needed();
      Inputs inputs = _poll_inputs();
      _step(inputs);
      _ble_status_notify_if_needed();
      sleep_ms(GO_MAIN_LOOP_DELAY_MS);
    }
  }

private:
  ButtonService *buttons_ = nullptr;
  QueueHandle_t input_queue_ = nullptr;
  PMSensor *pm_sensor_ = nullptr;
  TVOCNOxSensor *tvoc_nox_sensor_ = nullptr;
  CO2Sensor *co2_sensor_ = nullptr;
  dps368_handle_t *dps368_ = nullptr;
  Scd4xTest *scd4x_ = nullptr;
  s12_i2c_t *s12_ = nullptr;
  sunrise_i2c_t *sunrise_ = nullptr;
  BLEStream *ble_ = nullptr;
  i2c_master_bus_handle_t i2c_bus_ = nullptr;
  GPSService *gps_ = nullptr;
  dashboard::Dashboard *dash_ = nullptr;
  NandStorageService *storage_ = nullptr;
  drivers::BQ25629 *charger_ = nullptr;

  uint32_t last_wdt_reset_ms_ = 0;
  uint32_t tracking_session_id_ = 0;
  uint32_t tracking_sleep_interval_s_ = GO_TRACKING_SLEEP_INTERVAL_S;
  uint32_t last_bq_vbus_poll_ms_ = 0;
  uint32_t last_bq_wdt_reset_ms_ = 0;

  State _state = State::Idle;
  uint32_t _state_enter_ms = 0;
  uint32_t _last_idle_measure_ms = 0;
  uint32_t _tracking_next_cycle_ms = 0;
  bool _sync_started = false;
  bool _tracking_started = false;
  bool _shutdown_started = false;
  std::string _serial_number;
  bool _sync_wifi_connected = false;
  bool charger_vbus_seen_ = false;
  bool usb_c_adapter_present_ = false;
  drivers::VBusStatus last_vbus_status_ = drivers::VBusStatus::NO_ADAPTER;

  std::string ble_device_name_;

  bool battery_percent_ok_ = false;
  int battery_percent_ = -1;

  dashboard::Values dash_values_{400, 0, 0, 0, 0, 0, 0, false, 0};

  bool flash_avail_ok_ = false;
  uint32_t flash_avail_kb_ = 0;
  uint32_t last_flash_avail_sample_ms_ = 0;

  bool scd4x_last_valid_ = false;
  uint16_t scd4x_last_ppm_ = 0;

  bool s12_last_valid_ = false;
  uint16_t s12_last_ppm_ = 0;

  bool sunlight_last_valid_ = false;
  uint16_t sunlight_last_ppm_ = 0;

  bool pending_co2_force_calib_ = false;
  uint16_t pending_co2_force_calib_ppm_ = 400;

  bool pending_ble_tracking_ = false;
  bool pending_ble_tracking_enabled_ = false;

  bool pending_ble_flash_erase_ = false;

  bool co2_calibrating_ = false;


  bool ble_status_dirty_ = true;
  bool last_ble_status_subscribed_ = false;

  void _ble_notify_status_now(State s) {
    if (ble_ == nullptr) {
      return;
    }
    if (!ble_->is_running() || !ble_->status_subscribed()) {
      return;
    }

    GPSService::Data d = {};
    bool gps_ok = false;
    if (gps_ != nullptr) {
      d = gps_->get();
      gps_ok = true;
    }
    _sample_battery_percent();
    _sample_flash_avail_if_needed();

    const std::string payload = build_ble_status_payload(
        s, gps_ok, d, battery_percent_ok_, battery_percent_, flash_avail_ok_, flash_avail_kb_,
        charger_vbus_seen_, usb_c_adapter_present_,
        tracking_session_id_, tracking_sleep_interval_s_, co2_calibrating_);
    ble_->notify_status(payload);
  }

  uint8_t _dashboard_status_mask_(bool gps_ok, const GPSService::Data &gps) const {
    uint8_t m = 0;
    if (_state == State::Sync) {
      m = (uint8_t)(m | dashboard::STATUS_SYNC);
    }
    if (_state == State::Tracking) {
      m = (uint8_t)(m | dashboard::STATUS_TRACKING);
    }
    if (gps_ok && gps.fix_valid) {
      m = (uint8_t)(m | dashboard::STATUS_GPS_FIX);
    }
    return m;
  }

  void _dashboard_update_status_only_(void) {
    if (dash_ == nullptr) {
      return;
    }

    GPSService::Data d = {};
    bool gps_ok = false;
    if (gps_ != nullptr) {
      d = gps_->get();
      gps_ok = true;
    }

    dashboard::Values v = dash_values_;
    v.status_mask = _dashboard_status_mask_(gps_ok, d);

    if (gps_ok && d.utc.time_valid) {
      v.hour = (uint8_t)d.utc.hour;
      v.minute = (uint8_t)d.utc.min;
    }

    _sample_battery_percent();
    if (battery_percent_ok_) {
      const int bp = (battery_percent_ < 0) ? 0 : ((battery_percent_ > 100) ? 100 : battery_percent_);
      v.battery_pct = (uint8_t)bp;
    }

    if (charger_ != nullptr) {
      bool ch = false;
      if (charger_->is_charging(ch) == ESP_OK) {
        v.is_battery_charging = ch;
      }
    }

    dash_->update(v);
    dash_values_ = v;
  }

  void _ble_status_notify_if_needed(void) {
    if (ble_ == nullptr || !ble_->is_running()) {
      last_ble_status_subscribed_ = false;
      return;
    }

    const bool subscribed = ble_->status_subscribed();
    if (subscribed && !last_ble_status_subscribed_) {
      // Client subscribed: push the current status once.
      ble_status_dirty_ = true;
    }
    last_ble_status_subscribed_ = subscribed;

    if (!ble_status_dirty_ || !subscribed) {
      return;
    }

    _ble_notify_status_now(_state);
    ble_status_dirty_ = false;
  }

  void _sample_battery_percent(void) {
    battery_percent_ok_ = false;
    battery_percent_ = -1;
    if (charger_ == nullptr) {
      return;
    }
    uint8_t perc = 0;
    const esp_err_t err = charger_->estimate_battery_percent(perc);
    if (err != ESP_OK) {
      return;
    }
    battery_percent_ok_ = true;
    battery_percent_ = (int)perc;
  }

  void _sample_flash_avail_if_needed(void) {
    // Only available when NAND storage is mounted/ready.
    if (storage_ == nullptr || !storage_->is_ready()) {
      flash_avail_ok_ = false;
      return;
    }

    static constexpr uint32_t SAMPLE_INTERVAL_MS = 10000;
    const uint32_t now = now_ms();
    if (last_flash_avail_sample_ms_ != 0 && (now - last_flash_avail_sample_ms_) < SAMPLE_INTERVAL_MS) {
      return;
    }
    last_flash_avail_sample_ms_ = now;

    uint64_t total_bytes = 0;
    uint64_t free_bytes = 0;
    const esp_err_t err = esp_vfs_fat_info(GO_NAND_MOUNT_PATH, &total_bytes, &free_bytes);
    if (err != ESP_OK) {
      flash_avail_ok_ = false;
      return;
    }

    flash_avail_ok_ = true;
    flash_avail_kb_ = (uint32_t)(free_bytes / 1024ULL);

    ESP_LOGI(GO_TAG, "flash: total=%" PRIu64 " free=%" PRIu64 " avail_kb=%" PRIu32,
             total_bytes, free_bytes, flash_avail_kb_);
  }

  void _apply_ble_config_if_needed(void) {
    if (ble_ == nullptr) {
      return;
    }

    uint32_t s = 0;
    if (ble_->take_pending_tracking_sleep_interval_s(&s)) {
      if (s != 0) {
        tracking_sleep_interval_s_ = s;
        RTC_TRACKING_SLEEP_INTERVAL_S = s;
        ESP_LOGI(GO_TAG, "config trackingSleepS=%" PRIu32, (uint32_t)s);
        ble_status_dirty_ = true;

#if NO_INACTIVE_NO_SLEEP == 1
        if (_state == State::Tracking && _tracking_next_cycle_ms != 0) {
          _tracking_next_cycle_ms = now_ms() + tracking_sleep_interval_s_ * 1000U;
        }
#endif
      }
    }

    uint16_t ppm = 0;
    if (ble_->take_pending_co2_force_calib(&ppm)) {
      if (ppm == 0) {
        ppm = 400;
      }
      pending_co2_force_calib_ppm_ = ppm;
      pending_co2_force_calib_ = true;
      co2_calibrating_ = true;
      ble_status_dirty_ = true;

      // Ensure the client sees co2Calibrating=true before we potentially block
      // in the calibration routine.
      if (ble_->is_running() && ble_->status_subscribed()) {
        _ble_notify_status_now(_state);
        ble_status_dirty_ = false;
      }
    }

    bool enabled = false;
    if (ble_->take_pending_tracking(&enabled)) {
      pending_ble_tracking_ = true;
      pending_ble_tracking_enabled_ = enabled;
    }

    if (ble_->take_pending_flash_erase()) {
      pending_ble_flash_erase_ = true;
    }
  }

  void _apply_ble_flash_erase_if_needed(void) {
    if (!pending_ble_flash_erase_) {
      return;
    }
    pending_ble_flash_erase_ = false;

    // Only allow flash erase from IDLE.
    if (_state != State::Idle) {
      ESP_LOGW(GO_TAG, "BLE flashErase ignored: state=%s", state_name(_state));
      return;
    }

    ESP_LOGW(GO_TAG, "BLE flashErase requested");
    _clear_tracking_logs();
  }

  void _apply_ble_tracking_if_needed(void) {
    if (!pending_ble_tracking_) {
      return;
    }
    pending_ble_tracking_ = false;

    if (pending_ble_tracking_enabled_) {
      // Start tracking: only allowed in IDLE.
      if (_state != State::Idle) {
        ESP_LOGW(GO_TAG, "BLE tracking start ignored: state=%s", state_name(_state));
        return;
      }
      _start_new_tracking_session();
      _transition(State::Tracking);
      return;
    }

    // Stop tracking: only allowed in TRACKING.
    if (_state != State::Tracking) {
      ESP_LOGW(GO_TAG, "BLE tracking stop ignored: state=%s", state_name(_state));
      return;
    }

#if NO_INACTIVE_NO_SLEEP == 1
    // In dev mode we don't reboot between modes, but TRACKING deep-sleeps the panel after
    // full_refresh(). Ensure we wake and restore basemap prerequisites before switching to
    // IDLE (which uses partial refresh).
    _dashboard_update_status_only_();
#endif

    _transition(State::Idle);
  }

  void _co2_force_calibrate_if_needed(void) {
    if (!pending_co2_force_calib_) {
      return;
    }
    if (_state != State::Idle) {
      return;
    }
    const uint16_t ppm = pending_co2_force_calib_ppm_;
    pending_co2_force_calib_ = false;

    // Blocking calibrations can take tens of seconds; give watchdogs a fresh interval.
    {
      const uint32_t now = now_ms();
      reset_ext_watchdog();
      last_wdt_reset_ms_ = now;
      if (charger_ != nullptr) {
        (void)charger_->reset_watchdog();
        last_bq_wdt_reset_ms_ = now;
      }
    }

    // Run STCC4 calibration first (if supported).
    if (co2_sensor_ != nullptr && co2_sensor_->support_force_calibration()) {
      ESP_LOGI(GO_TAG, "co2ForceCalib (stcc4) begin target=%u ppm", (unsigned)ppm);
      const bool ok = co2_sensor_->force_calibration(ppm);
      ESP_LOGI(GO_TAG, "co2ForceCalib (stcc4) %s", ok ? "ok" : "failed");
    } else {
      ESP_LOGW(GO_TAG, "co2ForceCalib: CO2 sensor doesn't support calibration");
    }

    // Then run SCD4x (SCD43) calibration (test-only integration).
    if (scd4x_ != nullptr && scd4x_->initialized) {
      _scd4x_force_calibration_(ppm);
    }

    // Senseair I2C test-only sensors (blocking; ok for up to ~1 minute).
    if (s12_ != nullptr) {
      ESP_LOGI(GO_TAG, "co2ForceCalib (s12) begin target=%u ppm", (unsigned)ppm);
      const esp_err_t err = s12_i2c_force_calibration(s12_, ppm, 0);
      ESP_LOGI(GO_TAG, "co2ForceCalib (s12) %s", (err == ESP_OK) ? "ok" : esp_err_to_name(err));
    }
    if (sunrise_ != nullptr) {
      ESP_LOGI(GO_TAG, "co2ForceCalib (sunlight) begin target=%u ppm", (unsigned)ppm);
      const esp_err_t err = sunrise_i2c_force_calibration(sunrise_, ppm, 0);
      ESP_LOGI(GO_TAG, "co2ForceCalib (sunlight) %s", (err == ESP_OK) ? "ok" : esp_err_to_name(err));
    }

    co2_calibrating_ = false;
    ble_status_dirty_ = true;
    if (ble_ != nullptr && ble_->is_running() && ble_->status_subscribed()) {
      _ble_notify_status_now(_state);
      ble_status_dirty_ = false;
    }
  }

  void _ble_history_export_run_blocking_(void) {
    if (ble_ == nullptr) {
      return;
    }
    if (_state != State::Idle) {
      return;
    }
    if (storage_ == nullptr || !storage_->is_ready()) {
      ESP_LOGW(GO_TAG, "history export ignored: storage not ready");
      return;
    }
    if (!ble_->is_running() || !ble_->history_subscribed()) {
      ESP_LOGW(GO_TAG, "history export ignored: BLE history not subscribed");
      return;
    }

    const TickType_t to = pdMS_TO_TICKS(GO_NAND_CMD_TIMEOUT_MS);
    uint32_t total = 0;
    esp_err_t err = storage_->get_count_sync(&total, to);
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "history export start failed: %s", esp_err_to_name(err));
      return;
    }

    ESP_LOGI(GO_TAG, "history export begin: total=%" PRIu32, total);

    if (total == 0) {
      // Emit a single terminal packet.
      // route empty, last=1, then measures payload with ts=null and the rest empty.
      NandStorageService::Record empty;
      std::string payload;
      payload.reserve(64);
      payload.push_back(';');
      payload.push_back('1');
      payload.push_back(';');
      payload.append(build_ble_measure_payload(empty));
      (void)ble_->notify_history(payload);
      ESP_LOGI(GO_TAG, "history export done (empty)");
      return;
    }

    static constexpr uint32_t CHUNK = 3;
    NandStorageService::Record recs[CHUNK];

    uint32_t idx = 0;
    while (idx < total) {
      // Requirement: block everything, but keep watchdogs alive.
      _kick_watchdogs_if_needed();

      if (_state != State::Idle) {
        ESP_LOGW(GO_TAG, "history export aborted: left IDLE");
        return;
      }
      if (!ble_->is_running() || !ble_->history_subscribed()) {
        ESP_LOGW(GO_TAG, "history export aborted: BLE history not subscribed");
        return;
      }
      if (storage_ == nullptr || !storage_->is_ready()) {
        ESP_LOGW(GO_TAG, "history export aborted: storage not ready");
        return;
      }

      uint32_t nread = 0;
      err = storage_->read_range_sync(idx, recs, CHUNK, &nread, to);
      if (err != ESP_OK || nread == 0) {
        ESP_LOGW(GO_TAG, "history export read failed at idx=%" PRIu32 ": %s", idx,
                 esp_err_to_name(err));
        return;
      }

      for (uint32_t i = 0; i < nread; ++i) {
        const bool last = ((idx + 1U) >= total);
        const std::string payload = build_ble_history_payload(recs[i], last);

        while (true) {
          if (_state != State::Idle) {
            ESP_LOGW(GO_TAG, "history export aborted: left IDLE");
            return;
          }
          if (!ble_->is_running() || !ble_->history_subscribed()) {
            ESP_LOGW(GO_TAG, "history export aborted: BLE history not subscribed");
            return;
          }
          _kick_watchdogs_if_needed();

          if (ble_->notify_history(payload)) {
            break;
          }

          // Give NimBLE + IDLE task time; required to avoid task_wdt.
          vTaskDelay(1);
        }

        idx += 1;

        // Yield between records.
        vTaskDelay(1);

        if (last) {
          ESP_LOGI(GO_TAG, "history export done");
          return;
        }
      }
    }
  }

  void _scd4x_force_calibration_(uint16_t target_ppm) {
    if (target_ppm == 0) {
      target_ppm = 400;
    }

    ESP_LOGI(GO_TAG, "co2ForceCalib (scd4x) begin target=%u ppm", (unsigned)target_ppm);

    // SCD4x FRC is only available in idle mode; stop periodic first.
    int16_t err = scd4x_stop_periodic_measurement();
    if (err != 0) {
      ESP_LOGW(GO_TAG, "co2ForceCalib (scd4x) stop_periodic failed: %d", (int)err);
      // Continue anyway; some firmwares tolerate FRC after a failed stop.
    }

    uint16_t corr_raw = 0;
    err = scd4x_perform_forced_recalibration(target_ppm, &corr_raw);
    if (err != 0) {
      ESP_LOGW(GO_TAG, "co2ForceCalib (scd4x) FRC failed: %d", (int)err);
    } else {
      // Library docs: correction in ppm = corr_raw - 0x8000. 0xFFFF indicates failure.
      if (corr_raw == 0xFFFFu) {
        ESP_LOGW(GO_TAG, "co2ForceCalib (scd4x) FRC failed (0xFFFF)");
      } else {
        const int32_t corr_ppm = (int32_t)corr_raw - 0x8000;
        ESP_LOGI(GO_TAG, "co2ForceCalib (scd4x) correction=%" PRId32 " ppm", corr_ppm);
      }
    }

    err = scd4x_start_periodic_measurement();
    if (err != 0) {
      ESP_LOGW(GO_TAG, "co2ForceCalib (scd4x) start_periodic failed: %d", (int)err);
    }
  }

  void _ble_notify(const NandStorageService::Record &rec, bool gps_ok, const GPSService::Data &gps) {
    if (ble_ == nullptr) {
      return;
    }
    if (!ble_->is_running()) {
      return;
    }

    if (ble_->measures_subscribed()) {
      const std::string payload = build_ble_measure_payload(rec);
      ble_->notify_measures(payload);
    }
    if (ble_->status_subscribed()) {
      _sample_flash_avail_if_needed();
      const std::string payload = build_ble_status_payload(
          _state, gps_ok, gps, battery_percent_ok_, battery_percent_, flash_avail_ok_, flash_avail_kb_,
          charger_vbus_seen_,
          usb_c_adapter_present_, rec.id, tracking_sleep_interval_s_, co2_calibrating_);
      ble_->notify_status(payload);
    }
  }

  void _init(void) {
    tracking_session_id_ = RTC_TRACKING_SESSION_ID;
    _serial_number = buildSerialNumber();

    tracking_sleep_interval_s_ = RTC_TRACKING_SLEEP_INTERVAL_S;
    if (tracking_sleep_interval_s_ == 0) {
      tracking_sleep_interval_s_ = GO_TRACKING_SLEEP_INTERVAL_S;
      RTC_TRACKING_SLEEP_INTERVAL_S = tracking_sleep_interval_s_;
    }

    ble_device_name_.clear();
    if (!_serial_number.empty()) {
      ble_device_name_ = std::string("AirGradientGo-") + _serial_number;
    } else {
      ble_device_name_ = "AirGradientGo";
    }

    State last = RTC_LAST_STATE;
    if (!is_valid_rtc_state(last)) {
      last = State::Idle;
    }

    const esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    State initial = State::Idle;
    if (cause == ESP_SLEEP_WAKEUP_EXT1) {
      initial = State::Idle;
    } else if (cause == ESP_SLEEP_WAKEUP_TIMER) {
      if (last == State::Tracking) {
        initial = State::Tracking;
      } else {
        ESP_LOGI(GO_TAG, "Tracking stopped");
        initial = State::Idle;
      }
    } else {
      initial = State::Idle;
    }

    ESP_LOGI(GO_TAG, "wake cause=%d last=%s initial=%s", (int)cause, state_name(last),
             state_name(initial));

    if (ble_ != nullptr && initial != State::Sync) {
      (void)ble_->start(ble_device_name_.c_str());
    }
    _transition(initial);
  }

  void _start_new_tracking_session(void) {
    static constexpr uint32_t MIN_ID = 10000;
    static constexpr uint32_t SPAN = 90000;
    const uint32_t new_id = (esp_random() % SPAN) + MIN_ID;
    RTC_TRACKING_SESSION_ID = new_id;
    tracking_session_id_ = new_id;
    ESP_LOGI(GO_TAG, "tracking session id=%05" PRIu32, tracking_session_id_);
  }

  Inputs _poll_inputs(void) {
    Inputs in;
    if (input_queue_ == nullptr) {
      return in;
    }

    GoInputEvent ev;
    while (xQueueReceive(input_queue_, &ev, 0) == pdTRUE) {
      if (ev.type == GoInputEventType::ButtonShort) {
        in.button_short = true;
      } else if (ev.type == GoInputEventType::ButtonLong) {
        in.button_long = true;
      } else if (ev.type == GoInputEventType::BootLong) {
        in.boot_long = true;
      } else if (ev.type == GoInputEventType::TouchRightLong) {
        in.touch_right_long = true;
      } else if (ev.type == GoInputEventType::TouchLeftLong) {
        in.touch_left_long = true;
      } else if (ev.type == GoInputEventType::TouchEnterLong) {
        in.touch_enter_long = true;
      }
    }
    return in;
  }

  void _clear_tracking_logs(void) {
    if (storage_ == nullptr) {
      ESP_LOGW(GO_TAG, "clear logs: storage not configured");
      return;
    }
    if (!storage_->is_ready()) {
      ESP_LOGW(GO_TAG, "clear logs: storage not ready");
      return;
    }

    // Clear can block; reset watchdogs to give us margin.
    {
      const uint32_t now = now_ms();
      reset_ext_watchdog();
      last_wdt_reset_ms_ = now;
      if (charger_ != nullptr) {
        (void)charger_->reset_watchdog();
        last_bq_wdt_reset_ms_ = now;
      }
    }

    const TickType_t to = pdMS_TO_TICKS(GO_NAND_CMD_TIMEOUT_MS);
    const esp_err_t err = storage_->clear_sync(to);
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "clear logs: failed: %s", esp_err_to_name(err));
      return;
    }
    ESP_LOGI(GO_TAG, "clear logs: ok");
  }

  void _step(const Inputs &in) {
    switch (_state) {
    case State::Idle:
      _state_idle(in);
      break;
    case State::Inactive:
      _state_inactive(in);
      break;
    case State::Sync:
      _state_sync(in);
      break;
    case State::Tracking:
      _state_tracking(in);
      break;
    case State::Shutdown:
      _state_shutdown(in);
      break;
    }
  }

  void _transition(State next) {
    if (next == _state) {
      return;
    }

    const State prev = _state;

    // State transition should emit a status update if a client is subscribed.
    // Special-case SYNC: BLE is stopped to avoid Wi-Fi conflicts, so send the
    // SYNC state before stopping.
    if (ble_ != nullptr && next == State::Sync) {
      _ble_notify_status_now(State::Sync);
      ble_->stop();
    } else if (ble_ != nullptr && prev == State::Sync) {
      (void)ble_->start(ble_device_name_.c_str());
    }

    ESP_LOGI(GO_TAG, "state %s -> %s", state_name(_state), state_name(next));
    _state = next;
    _state_enter_ms = now_ms();

    ble_status_dirty_ = true;

    if (_state == State::Idle) {
      _last_idle_measure_ms = 0;
      _sync_started = false;
      _tracking_started = false;
      _shutdown_started = false;
    }
    if (_state == State::Sync) {
      _sync_started = false;
    }
    if (_state == State::Tracking) {
      _tracking_started = false;
      _tracking_next_cycle_ms = 0;
    }
    if (_state == State::Shutdown) {
      _shutdown_started = false;
    }

    // Best-effort immediate notify if subscribed (except SYNC which stopped BLE).
    _ble_status_notify_if_needed();

    // Best-effort: keep dashboard header icons in sync with state transitions.
    _dashboard_update_status_only_();
  }

  void _state_idle(const Inputs &in) {
    _kick_watchdogs_if_needed();

    // Transitions from diagram.
    if (in.touch_right_long) {
      _start_new_tracking_session();
      _transition(State::Tracking);
      return;
    }
    if (in.touch_enter_long) {
      _transition(State::Sync);
      return;
    }
    if (in.boot_long) {
      _clear_tracking_logs();
      return;
    }
    if (in.button_long) {
      _transition(State::Shutdown);
      return;
    }

#if NO_INACTIVE_NO_SLEEP == 0
    if (in.button_short) {
      _transition(State::Inactive);
      return;
    }

    // Auto-inactive after timeout
    const uint32_t inactive_elapsed_ms = now_ms() - _state_enter_ms;
    if (inactive_elapsed_ms >= (uint32_t)GO_IDLE_INACTIVE_TIMEOUT_MS) {
      _transition(State::Inactive);
      return;
    }
#endif

    // Periodic measurement + display.
    if (_last_idle_measure_ms == 0) {
      _last_idle_measure_ms = now_ms();
    }
    const uint32_t measure_elapsed_ms = now_ms() - _last_idle_measure_ms;
    if (measure_elapsed_ms >= (uint32_t)GO_IDLE_MEASURE_INTERVAL_MS) {
      _idle_measure_and_display();
      _last_idle_measure_ms = now_ms();
    }
  }

  void _kick_watchdogs_if_needed(void) {
    const uint32_t now = now_ms();
    const uint32_t ext_elapsed_ms = now - last_wdt_reset_ms_;
    if (ext_elapsed_ms >= GO_WDT_RESET_INTERVAL_MS) {
      reset_ext_watchdog();
      last_wdt_reset_ms_ = now;
    }

    if (charger_ != nullptr) {
      const uint32_t bq_elapsed_ms = now - last_bq_wdt_reset_ms_;
      if (bq_elapsed_ms >= GO_BQ_WDT_RESET_INTERVAL_MS) {
        const esp_err_t err = charger_->reset_watchdog();
        if (err != ESP_OK) {
          ESP_LOGW(GO_TAG, "BQ25629 watchdog reset failed: %s", esp_err_to_name(err));
        }
        last_bq_wdt_reset_ms_ = now;
      }
    }
  }

  void _recover_charger_charge_path_if_needed(drivers::VBusStatus vbus) {
    if (charger_ == nullptr || !is_usb_c_adapter_present(vbus)) {
      return;
    }

    uint8_t reg16 = 0;
    esp_err_t err = charger_->read_register(drivers::BQ25629_REG::CHARGER_CONTROL_0, reg16);
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "BQ25629 recovery: read REG0x16 failed: %s", esp_err_to_name(err));
      return;
    }

    uint8_t reg18 = 0;
    err = charger_->read_register(drivers::BQ25629_REG::CHARGER_CONTROL_2, reg18);
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "BQ25629 recovery: read REG0x18 failed: %s", esp_err_to_name(err));
      return;
    }

    // Field mapping from observed behavior in logs: bit6 can force battery discharge state.
    const bool force_ibatdis = (reg16 & (1U << 6)) != 0;
    const bool en_chg = (reg16 & (1U << 5)) != 0;
    const bool en_hiz = (reg16 & (1U << 4)) != 0;
    const bool force_pmid_dis = (reg16 & (1U << 3)) != 0;
    const bool en_otg = (reg18 & (1U << 6)) != 0;

    bool changed = false;

    if (force_ibatdis) {
      const uint8_t next = (uint8_t)(reg16 & ~(1U << 6));
      err = charger_->write_register(drivers::BQ25629_REG::CHARGER_CONTROL_0, next);
      if (err == ESP_OK) {
        reg16 = next;
        changed = true;
        ESP_LOGW(GO_TAG, "BQ25629 recovery: cleared REG0x16 bit6 (FORCE_IBATDIS)");
      } else {
        ESP_LOGW(GO_TAG, "BQ25629 recovery: clear REG0x16 bit6 failed: %s", esp_err_to_name(err));
      }
    }

    if (force_pmid_dis) {
      err = charger_->enable_pmid_discharge(false);
      if (err == ESP_OK) {
        reg16 = (uint8_t)(reg16 & ~(1U << 3));
        changed = true;
        ESP_LOGW(GO_TAG, "BQ25629 recovery: disabled FORCE_PMID_DIS");
      } else {
        ESP_LOGW(GO_TAG, "BQ25629 recovery: disable FORCE_PMID_DIS failed: %s",
                 esp_err_to_name(err));
      }
    }

    if (en_hiz) {
      err = charger_->disable_hiz_mode();
      if (err == ESP_OK) {
        reg16 = (uint8_t)(reg16 & ~(1U << 4));
        changed = true;
        ESP_LOGW(GO_TAG, "BQ25629 recovery: disabled HIZ mode");
      } else {
        ESP_LOGW(GO_TAG, "BQ25629 recovery: disable HIZ failed: %s", esp_err_to_name(err));
      }
    }

    if (!en_chg) {
      err = charger_->enable_charging(true);
      if (err == ESP_OK) {
        reg16 = (uint8_t)(reg16 | (1U << 5));
        changed = true;
        ESP_LOGW(GO_TAG, "BQ25629 recovery: re-enabled charging");
      } else {
        ESP_LOGW(GO_TAG, "BQ25629 recovery: enable_charging(true) failed: %s", esp_err_to_name(err));
      }
    }

    if (en_otg) {
      err = charger_->enable_otg(false);
      if (err == ESP_OK) {
        reg18 = (uint8_t)(reg18 & ~(1U << 6));
        changed = true;
        ESP_LOGW(GO_TAG, "BQ25629 recovery: disabled OTG while adapter is present");
      } else {
        ESP_LOGW(GO_TAG, "BQ25629 recovery: disable OTG failed: %s", esp_err_to_name(err));
      }
    }

    if (changed) {
      err = charger_->set_watchdog_timeout(drivers::WatchdogTimeout::Sec200);
      if (err != ESP_OK) {
        ESP_LOGW(GO_TAG, "BQ25629 recovery: set_watchdog_timeout failed: %s", esp_err_to_name(err));
      }

      err = charger_->reset_watchdog();
      if (err != ESP_OK) {
        ESP_LOGW(GO_TAG, "BQ25629 recovery: reset_watchdog failed: %s", esp_err_to_name(err));
      } else {
        last_bq_wdt_reset_ms_ = now_ms();
      }

      ESP_LOGW(GO_TAG, "BQ25629 recovery complete: REG0x16=0x%02X REG0x18=0x%02X", (unsigned)reg16,
               (unsigned)reg18);
    }
  }

  void _poll_usb_c_if_needed(void) {
    if (charger_ == nullptr) {
      return;
    }

    const uint32_t now = now_ms();
    if ((now - last_bq_vbus_poll_ms_) < GO_BQ_VBUS_POLL_INTERVAL_MS) {
      return;
    }
    last_bq_vbus_poll_ms_ = now;

    drivers::VBusStatus vbus = drivers::VBusStatus::NO_ADAPTER;
    const esp_err_t err = charger_->get_vbus_status(vbus);
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "BQ25629 get_vbus_status failed: %s", esp_err_to_name(err));
      return;
    }

    const bool adapter_present = is_usb_c_adapter_present(vbus);
    if (!charger_vbus_seen_) {
      charger_vbus_seen_ = true;
      usb_c_adapter_present_ = adapter_present;
      last_vbus_status_ = vbus;
      ESP_LOGI(GO_TAG, "USB-C initial: %s (vbus=%s)", adapter_present ? "plugged" : "unplugged",
               vbus_status_name(vbus));
      _recover_charger_charge_path_if_needed(vbus);
      return;
    }

    if (vbus != last_vbus_status_) {
      ESP_LOGI(GO_TAG, "BQ25629 vbus: %s -> %s", vbus_status_name(last_vbus_status_),
               vbus_status_name(vbus));
    }

    if (adapter_present != usb_c_adapter_present_) {
      if (adapter_present) {
        ESP_LOGI(GO_TAG, "USB-C plugged");
        _handle_usb_c_plugged_event();
      } else {
        ESP_LOGW(GO_TAG, "USB-C unplugged: wait PMID to 5V then re-init PM sensor");
        _enable_pmid_wait_and_reinit_pm_sensor("USB-C unplugged");
      }
      usb_c_adapter_present_ = adapter_present;
    }

    last_vbus_status_ = vbus;
    _recover_charger_charge_path_if_needed(vbus);
  }

  void _handle_usb_c_plugged_event(void) {
    if (charger_ == nullptr) {
      ESP_LOGW(GO_TAG, "USB-C plugged: charger unavailable");
      return;
    }

    drivers::BQ25629_ADC_Data adc = {};
    const esp_err_t adc_err = charger_->read_adc(adc);
    if (adc_err == ESP_OK) {
      if (adc.vpmid_mv >= GO_BQ_VPMID_READY_MV) {
        ESP_LOGI(GO_TAG, "USB-C plugged: PMID=%umV ready, re-initializing PM sensor",
                 (unsigned)adc.vpmid_mv);
        _reinit_pm_sensor("USB-C plugged");
        return;
      }
      ESP_LOGW(GO_TAG, "USB-C plugged: PMID=%umV not ready, enabling PMID boost",
               (unsigned)adc.vpmid_mv);
    } else {
      ESP_LOGW(GO_TAG, "USB-C plugged: PMID read failed (%s), enabling PMID boost",
               esp_err_to_name(adc_err));
    }

    _enable_pmid_wait_and_reinit_pm_sensor("USB-C plugged");
  }

  void _enable_pmid_wait_and_reinit_pm_sensor(const char *reason) {
    if (charger_ == nullptr) {
      ESP_LOGW(GO_TAG, "PMID wait skipped: charger unavailable");
      return;
    }

    esp_err_t err = charger_->enable_pmid_5v_boost();
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "PMID boost enable failed: %s", esp_err_to_name(err));
      return;
    }

    uint32_t last_log_ms = 0;
    uint32_t last_reenable_ms = now_ms();
    while (true) {
      _kick_watchdogs_if_needed();

      drivers::BQ25629_ADC_Data adc = {};
      err = charger_->read_adc(adc);
      if (err == ESP_OK) {
        if (adc.vpmid_mv >= GO_BQ_VPMID_READY_MV) {
          ESP_LOGI(GO_TAG, "%s: PMID ready %umV, re-initializing PM sensor", reason,
                   (unsigned)adc.vpmid_mv);
          break;
        }

        const uint32_t now = now_ms();
        if ((now - last_log_ms) >= 1000) {
          ESP_LOGI(GO_TAG, "%s: waiting PMID %umV (target >= %umV)", reason, (unsigned)adc.vpmid_mv,
                   (unsigned)GO_BQ_VPMID_READY_MV);
          last_log_ms = now;
        }
      } else {
        const uint32_t now = now_ms();
        if ((now - last_log_ms) >= 1000) {
          ESP_LOGW(GO_TAG, "%s: PMID ADC read failed while waiting: %s", reason,
                   esp_err_to_name(err));
          last_log_ms = now;
        }
      }

      const uint32_t now = now_ms();
      if ((now - last_reenable_ms) >= GO_BQ_VPMID_REENABLE_INTERVAL_MS) {
        (void)charger_->enable_pmid_5v_boost();
        last_reenable_ms = now;
      }

      sleep_ms(GO_BQ_VPMID_POLL_INTERVAL_MS);
    }

    _reinit_pm_sensor(reason);
  }

  void _reinit_pm_sensor(const char *reason) {
    if (pm_sensor_ == nullptr) {
      ESP_LOGW(GO_TAG, "PM sensor re-init skipped: not configured");
      return;
    }

    (void)gpio_set_level(GO_PM_POWER_GPIO, GO_PM_POWER_ON_LEVEL);
    sleep_ms(GO_SPS30_POWER_STABILIZE_DELAY_MS);

    if (!pm_sensor_->reinit()) {
      ESP_LOGW(GO_TAG, "PM sensor re-init failed (%s)", reason);
      return;
    }

    sleep_ms(GO_SPS30_WARMUP_DELAY_MS);
    ESP_LOGI(GO_TAG, "PM sensor re-initialized (%s)", reason);
  }

  void _state_inactive(const Inputs &in) {
    // INACTIVE: deep sleep until physical button is pressed.
    // Note: deep sleep resets the chip; wake handling/persistence comes later.
    (void)in;
    _inactive_enter_deep_sleep();
  }

  void _state_sync(const Inputs &in) {
    (void)in;
    // SYNC: connect to Wi-Fi and send stored data; then return to IDLE.
    if (!_sync_started) {
      _sync_started = true;
      _sync_begin();
    }

    const bool finished = _sync_step();
    if (finished) {
      _sync_end();
      _transition(State::Idle);
      return;
    }
  }

  void _state_tracking(const Inputs &in) {
    // TRACKING: boot -> measure -> save -> display -> sleep.
    // Diagram: touch (long) toggles back to IDLE.
    if (in.touch_right_long) {
#if NO_INACTIVE_NO_SLEEP == 1
      // In dev mode we don't reboot between modes, but TRACKING deep-sleeps the panel after
      // full_refresh(). Ensure we wake and restore basemap prerequisites before switching to
      // IDLE (which uses partial refresh).
      _dashboard_update_status_only_();
#endif
      _transition(State::Idle);
      return;
    }

#if NO_INACTIVE_NO_SLEEP == 1
    if (_tracking_next_cycle_ms != 0) {
      const uint32_t now = now_ms();
      // Handle wraparound safely via signed delta.
      if ((int32_t)(now - _tracking_next_cycle_ms) < 0) {
        return;
      }
      _tracking_next_cycle_ms = 0;
    }
#endif

    if (!_tracking_started) {
      _tracking_started = true;
      _tracking_begin();
    }

    const bool finished = _tracking_step();
    if (finished) {
      _tracking_end();
      _tracking_enter_sleep();
      return;
    }
  }

  void _state_shutdown(const Inputs &in) {
    (void)in;
    if (_shutdown_started) {
      return;
    }
    _shutdown_started = true;
    _shutdown_now();
  }

  void _shutdown_now(void) {
    ESP_LOGI(GO_TAG, "shutdown: begin");

    if (ble_ != nullptr) {
      ble_->stop();
    }

    // Ensure we have time to complete slow steps.
    reset_ext_watchdog();
    last_wdt_reset_ms_ = now_ms();

    // Stop button processing/interrupts so we don't fight I2C while shutting down.
    if (buttons_ != nullptr) {
      const esp_err_t err = buttons_->pre_light_sleep();
      if (err != ESP_OK) {
        ESP_LOGW(GO_TAG, "shutdown: buttons pre_light_sleep failed: %s", esp_err_to_name(err));
      }
    }

    // Best-effort: disconnect Wi-Fi if it was enabled.
    if (_sync_wifi_connected) {
      wifi_disconnect();
      _sync_wifi_connected = false;
    }

    // Flush queued storage writes.
    if (storage_ != nullptr && storage_->is_ready()) {
      const TickType_t to = pdMS_TO_TICKS(GO_NAND_CMD_TIMEOUT_MS);
      const esp_err_t err = storage_->flush_sync(to);
      if (err != ESP_OK) {
        ESP_LOGW(GO_TAG, "shutdown: storage flush failed: %s", esp_err_to_name(err));
      }
    }

    // Put the panel to sleep.
    if (dash_ != nullptr) {
      ESP_LOGI(GO_TAG, "shutdown: display sleep");
      dash_->deep_sleep();
    }
    // else if (epd_ != nullptr) {
    //   ESP_LOGI(GO_TAG, "shutdown: display clear (raw)");
    //   const uint32_t t0 = now_ms();
    //   esp_err_t err = epd_->ensure_init_full();
    //   if (err != ESP_OK) {
    //     ESP_LOGW(GO_TAG, "shutdown: epd ensure_init_full failed: %s", esp_err_to_name(err));
    //   } else {
    //     err = epd_->clear_white();
    //     if (err != ESP_OK) {
    //       ESP_LOGW(GO_TAG, "shutdown: epd clear_white failed: %s", esp_err_to_name(err));
    //     }
    //     sleep_ms(4000);
    //     err = epd_->deep_sleep();
    //     if (err != ESP_OK) {
    //       ESP_LOGW(GO_TAG, "shutdown: epd deep_sleep failed: %s", esp_err_to_name(err));
    //     }
    //   }
    //   ESP_LOGI(GO_TAG, "shutdown: display clear done (%" PRIu32 "ms)", now_ms() - t0);
    // }

    // Cut PM sensor rail.
    (void)gpio_set_level(GO_PM_POWER_GPIO, 0);

    // Stop GPS task/UART.
    if (gps_ != nullptr) {
      const esp_err_t err = gps_->stop();
      if (err != ESP_OK) {
        ESP_LOGW(GO_TAG, "shutdown: gps stop failed: %s", esp_err_to_name(err));
      }
    }

    // Enter ship mode: BATFET disconnects and system powers off.
    if (charger_ == nullptr) {
      ESP_LOGW(GO_TAG, "shutdown: charger not available; ship mode skipped");
    } else {
      const esp_err_t err = charger_->enter_ship_mode();
      if (err != ESP_OK) {
        ESP_LOGW(GO_TAG, "shutdown: enter_ship_mode failed: %s", esp_err_to_name(err));
      }
    }

    // waiting an actual shutdown from BMS
    esp_deep_sleep_start();
  }

  // ----- Placeholder implementations (fill in later) -----

  void _idle_measure_and_display(void) {
    log_bq25629_debug_snapshot(charger_);
    log_sensor_summary_banner();

    // Keep non-PM measurements running even if PM sensor is missing.
    PMData pm;
    pm.pm_01 = MeasuresInvalid::PM;
    pm.pm_25 = MeasuresInvalid::PM;
    pm.pm_10 = MeasuresInvalid::PM;
    pm.pm_01_sp = MeasuresInvalid::PM;
    pm.pm_25_sp = MeasuresInvalid::PM;
    pm.pm_10_sp = MeasuresInvalid::PM;
    pm.pm_03_pc = MeasuresInvalid::PM;
    pm.pm_05_pc = MeasuresInvalid::PM;
    pm.pm_01_pc = MeasuresInvalid::PM;
    pm.pm_25_pc = MeasuresInvalid::PM;
    pm.pm_5_pc = MeasuresInvalid::PM;
    pm.pm_10_pc = MeasuresInvalid::PM;

    if (pm_sensor_ == nullptr) {
      ESP_LOGW(GO_TAG, "PM sensor not initialized");
    } else if (!pm_sensor_->read(pm)) {
      ESP_LOGW(GO_TAG, "PM sensor read failed");
    }

    if (pm.is_pm_25_valid()) {
      ESP_LOGI(GO_TAG, "pm25: %.1f", pm.pm_25);
    }
    if (pm.is_pm_01_valid()) {
      ESP_LOGI(GO_TAG, "pm1.0: %.1f", pm.pm_01);
    }
    if (pm.is_pm_10_valid()) {
      ESP_LOGI(GO_TAG, "pm10: %.1f", pm.pm_10);
    }
    if (pm.is_pm_05_pc_valid()) {
      ESP_LOGI(GO_TAG, "count 0.5: %.1f", pm.pm_05_pc);
    }
    if (pm.is_pm_01_pc_valid()) {
      ESP_LOGI(GO_TAG, "count 1.0: %.1f", pm.pm_01_pc);
    }
    if (pm.is_pm_25_pc_valid()) {
      ESP_LOGI(GO_TAG, "count 2.5: %.1f", pm.pm_25_pc);
    }
    if (pm.is_pm_10_pc_valid()) {
      ESP_LOGI(GO_TAG, "count 10: %.1f", pm.pm_10_pc);
    }

    TVOCNOxData gas = {};
    bool tvoc_valid = false;
    bool nox_valid = false;

    if (tvoc_nox_sensor_ != nullptr) {
      if (tvoc_nox_sensor_->read(gas)) {
        tvoc_valid = gas.is_tvoc_raw_valid();
        nox_valid = gas.is_nox_raw_valid();
        ESP_LOGI(GO_TAG, "tvoc raw: %d", gas.tvoc_raw);
        ESP_LOGI(GO_TAG, "nox raw: %d", gas.nox_raw);

      } else {
        ESP_LOGW(GO_TAG, "TVOC/NOx read failed");
      }
    }

    CO2Data co2 = {};
    bool co2_valid = false;
    TempHumData th = {};
    bool th_temp_valid = false;
    bool th_hum_valid = false;

    if (co2_sensor_ != nullptr) {
      if (co2_sensor_->read(co2)) {
        co2_valid = co2.is_valid();
        ESP_LOGI(GO_TAG, "co2: %d", co2.co2);
        if (co2_sensor_->support_temp_hum()) {
          th = co2_sensor_->temp_hum_data();
          th_temp_valid = th.is_temp_valid();
          th_hum_valid = th.is_hum_valid();
          ESP_LOGI(GO_TAG, "temp: %.2f ; rhum: %.2f", th.temperature, th.humidity);
        }
      } else {
        ESP_LOGW(GO_TAG, "CO2 read failed");
      }
    }

    if (scd4x_ != nullptr && scd4x_->initialized) {
      bool ready = false;
      const int16_t rdy_err = scd4x_get_data_ready_status(&ready);
      if (rdy_err == 0 && ready) {
        uint16_t ppm = 0;
        uint16_t t_raw = 0;
        uint16_t rh_raw = 0;
        const int16_t meas_err = scd4x_read_measurement_raw(&ppm, &t_raw, &rh_raw);
        if (meas_err == 0 && ppm != 0) {
          scd4x_last_valid_ = true;
          scd4x_last_ppm_ = ppm;
          ESP_LOGI(GO_TAG, "scd4x: co2=%u", (unsigned)ppm);
        } else if (meas_err != 0) {
          ESP_LOGW(GO_TAG, "scd4x read failed: %d", (int)meas_err);
        }
      }
    }

    if (s12_ != nullptr) {
      uint16_t ppm = 0;
      const esp_err_t err = s12_i2c_read_co2_ppm(s12_, &ppm);
      if (err == ESP_OK && ppm > 0 && ppm <= 32000) {
        s12_last_valid_ = true;
        s12_last_ppm_ = ppm;
        ESP_LOGI(GO_TAG, "s12: co2=%u", (unsigned)ppm);
      } else if (err != ESP_OK) {
        ESP_LOGW(GO_TAG, "s12 read failed: %s", esp_err_to_name(err));
      }
    }

    if (sunrise_ != nullptr) {
      uint16_t ppm = 0;
      uint8_t e_status = 0;
      const esp_err_t err = sunrise_i2c_read_co2_ppm(sunrise_, &ppm, &e_status);
      if (err == ESP_OK && ppm > 0 && ppm <= 32000) {
        sunlight_last_valid_ = true;
        sunlight_last_ppm_ = ppm;
        ESP_LOGI(GO_TAG, "sunlight: co2=%u", (unsigned)ppm);
      } else if (err != ESP_OK) {
        ESP_LOGW(GO_TAG, "sunlight read failed: %s", esp_err_to_name(err));
      }
    }

    dps368_data_t dps = {};
    bool pressure_valid = false;

    if (dps368_ != nullptr) {
      const esp_err_t err = dps368_read(dps368_, &dps);
      if (err == ESP_OK) {
        pressure_valid = dps.pressure_valid;
        if (dps.pressure_valid && dps.temp_valid) {
          ESP_LOGI(GO_TAG, "dps368: p=%.1fPa t=%.2fC", dps.pressure_pa, dps.temperature_c);
        } else if (dps.pressure_valid) {
          ESP_LOGI(GO_TAG, "dps368: p=%.1fPa", dps.pressure_pa);
        } else if (dps.temp_valid) {
          ESP_LOGI(GO_TAG, "dps368: t=%.2fC", dps.temperature_c);
        }

      } else if (err != ESP_ERR_NOT_FOUND) {
        ESP_LOGW(GO_TAG, "dps368 read failed: %s", esp_err_to_name(err));
      }
    }

    GPSService::Data d = {};
    bool gps_ok = false;
    if (gps_ != nullptr) {
      d = gps_->get();
      gps_ok = true;
      log_gps_data(d);
    }

    _sample_battery_percent();

    if (dash_ != nullptr) {
      dashboard::Values v = dash_values_;
      v.status_mask = _dashboard_status_mask_(gps_ok, d);

      if (gps_ok && d.utc.time_valid) {
        v.hour = (uint8_t)d.utc.hour;
        v.minute = (uint8_t)d.utc.min;
      }

      if (pm.is_pm_25_valid()) {
        v.pm25_ugm3 = (int)lroundf(pm.pm_25);
      }

      if (co2_valid) {
        v.co2_ppm = co2.co2;
      }
      if (th_temp_valid) {
        v.temperature_c = (int)lroundf(th.temperature);
      }
      if (th_hum_valid) {
        v.humidity_pct = (int)lroundf(th.humidity);
      }

      if (battery_percent_ok_) {
        const int bp = (battery_percent_ < 0) ? 0 : ((battery_percent_ > 100) ? 100 : battery_percent_);
        v.battery_pct = (uint8_t)bp;
      }
      if (charger_ != nullptr) {
        bool ch = false;
        if (charger_->is_charging(ch) == ESP_OK) {
          v.is_battery_charging = ch;
        }
      }

      dash_->update(v);
      dash_values_ = v;
    }

    if (ble_ != nullptr && ble_->is_running() && (ble_->measures_subscribed() || ble_->status_subscribed())) {
      NandStorageService::Record rec;
      rec.id = tracking_session_id_;

      uint64_t epoch_ms = 0;
      if (gps_ok) {
        (void)utc_to_epoch_ms(d.utc, &epoch_ms);
      }
      rec.timestamp_ms = epoch_ms;

      if (pm.is_pm_01_valid()) {
        rec.pm01_ugm3_x10 = go_utils::pm_ugm3_to_x10(pm.pm_01);
      }
      if (pm.is_pm_25_valid()) {
        rec.pm25_ugm3_x10 = go_utils::pm_ugm3_to_x10(pm.pm_25);
      }
      if (pm.is_pm_10_valid()) {
        rec.pm10_ugm3_x10 = go_utils::pm_ugm3_to_x10(pm.pm_10);
      }

      if (pm.is_pm_05_pc_valid()) {
        rec.pc05_x10 = go_utils::count_to_x10(pm.pm_05_pc);
      }
      if (pm.is_pm_01_pc_valid()) {
        rec.pc10_x10 = go_utils::count_to_x10(pm.pm_01_pc);
      }
      if (pm.is_pm_25_pc_valid()) {
        rec.pc25_x10 = go_utils::count_to_x10(pm.pm_25_pc);
      }
      if (pm.is_pm_10_pc_valid()) {
        rec.pc100_x10 = go_utils::count_to_x10(pm.pm_10_pc);
      }

      if (co2_valid) {
        rec.co2_ppm = go_utils::u16_from_int_nonneg(co2.co2);
        if (th_temp_valid) {
          rec.temperature_c_x100 = go_utils::temp_c_to_x100(th.temperature);
        }
        if (th_hum_valid) {
          rec.humidity_rh_x100 = go_utils::hum_rh_to_x100(th.humidity);
        }
      }

      if (scd4x_last_valid_) {
        rec.scd4x = scd4x_last_ppm_;
      }

      if (s12_last_valid_) {
        rec.s12 = s12_last_ppm_;
      }
      if (sunlight_last_valid_) {
        rec.sunlight = sunlight_last_ppm_;
      }

      if (pressure_valid) {
        rec.pressure_pa = go_utils::pressure_pa_from_float(dps.pressure_pa);
      }

      if (tvoc_valid) {
        rec.tvoc_raw = go_utils::u16_from_int_nonneg(gas.tvoc_raw);
      }
      if (nox_valid) {
        rec.nox_raw = go_utils::u16_from_int_nonneg(gas.nox_raw);
      }

      if (gps_ok && d.fix_valid) {
        rec.latitude_e7 = go_utils::deg_to_e7(d.latitude_deg);
        rec.longitude_e7 = go_utils::deg_to_e7(d.longitude_deg);
      }

      _ble_notify(rec, gps_ok, d);
    }
  }

  void _inactive_enter_deep_sleep(void) {
    // TODO: configure wakeup source (physical button) and enter deep sleep.
    if (dash_ != nullptr) {
      dash_->deep_sleep();
    }

    ESP_LOGI(GO_TAG, "inactive: entering deep sleep (stub)");
    esp_err_t err = ESP_OK;
    if (buttons_ != nullptr) {
      err = buttons_->enable_deep_sleep_wakeup();
    } else {
      ESP_LOGE(GO_TAG, "deep sleep wake config failed: buttons not initialized");
      return;
    }
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "deep sleep wake config failed: %s", esp_err_to_name(err));
    }

    (void)esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    RTC_LAST_STATE = State::Inactive;
    esp_deep_sleep_start();
  }

  void _sync_begin(void) {
    ESP_LOGI(GO_TAG, "sync: begin");

    _dashboard_update_status_only_();

    _sync_wifi_connected = wifi_connect(_serial_number);
    if (!_sync_wifi_connected) {
      ESP_LOGW(GO_TAG, "sync: wifi connect failed");
    }
  }

  bool _sync_step(void) {
    if (!_sync_wifi_connected) {
      ESP_LOGW(GO_TAG, "sync: wifi not connected");
      return true;
    }
    if (storage_ == nullptr) {
      ESP_LOGW(GO_TAG, "sync: storage not configured");
      return true;
    }
    if (!storage_->is_ready()) {
      ESP_LOGW(GO_TAG, "sync: storage not ready");
      return true;
    }

    const TickType_t to = pdMS_TO_TICKS(GO_SYNC_CMD_TIMEOUT_MS);
    const uint64_t interval_ms = (uint64_t)tracking_sleep_interval_s_ * 1000ULL;

    uint32_t total = 0;
    esp_err_t err = storage_->get_count_sync(&total, to);
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "sync: get_count failed: %s", esp_err_to_name(err));
      return true;
    }

    if (total == 0) {
      ESP_LOGI(GO_TAG, "sync: nothing to send");
      return true;
    }

    ESP_LOGI(GO_TAG, "sync: total records=%" PRIu32, total);

    std::unique_ptr<NandStorageService::Record[]> batch_buf(
        new (std::nothrow) NandStorageService::Record[GO_SYNC_BATCH_MAX]);
    std::unique_ptr<uint64_t[]> ts_buf(new (std::nothrow) uint64_t[GO_SYNC_BATCH_MAX]);
    if (!batch_buf || !ts_buf) {
      ESP_LOGW(GO_TAG, "sync: out of memory allocating batch buffers");
      return true;
    }

    uint32_t idx = 0;
    bool any_route_sent = false;

    while (idx < total) {
      NandStorageService::Record first;
      uint32_t nread = 0;
      err = storage_->read_range_sync(idx, &first, 1, &nread, to);
      if (err != ESP_OK || nread != 1) {
        ESP_LOGW(GO_TAG, "sync: read failed at idx=%" PRIu32 ": %s", idx, esp_err_to_name(err));
        ESP_LOGW(GO_TAG, "sync: failed; keeping log");
        return true;
      }

      const uint32_t route_id = first.id;
      uint32_t route_start = idx;
      uint32_t route_end = route_start;

      while (route_end < total) {
        NandStorageService::Record r;
        nread = 0;
        err = storage_->read_range_sync(route_end, &r, 1, &nread, to);
        if (err != ESP_OK || nread != 1) {
          ESP_LOGW(GO_TAG, "sync: read failed at idx=%" PRIu32 ": %s", route_end,
                   esp_err_to_name(err));
          ESP_LOGW(GO_TAG, "sync: failed; keeping log");
          return true;
        }
        if (r.id != route_id) {
          break;
        }
        route_end += 1;
      }

      ESP_LOGI(GO_TAG, "sync: route=%" PRIu32 " records=%" PRIu32, route_id,
               route_end - route_start);

      bool route_ok = true;

      // Find the first non-zero GPS timestamp for this route so we can backfill earlier
      // records that have timestamp_ms==0.
      bool have_anchor = false;
      uint32_t anchor_idx = 0;
      uint64_t anchor_ts_ms = 0;
      for (uint32_t probe = route_start; probe < route_end; ++probe) {
        NandStorageService::Record r;
        nread = 0;
        err = storage_->read_range_sync(probe, &r, 1, &nread, to);
        if (err != ESP_OK || nread != 1) {
          ESP_LOGW(GO_TAG, "sync: read failed at idx=%" PRIu32 ": %s", probe, esp_err_to_name(err));
          ESP_LOGW(GO_TAG, "sync: failed; keeping log");
          return true;
        }
        if (r.timestamp_ms != 0) {
          have_anchor = true;
          anchor_idx = probe;
          anchor_ts_ms = r.timestamp_ms;
          break;
        }
      }

      if (!have_anchor) {
        // No GPS time ever became valid for this route; we can't synthesize timestamps.
        ESP_LOGW(GO_TAG, "sync: route=%" PRIu32 " has no timestamps; skipping", route_id);
        route_ok = false;
      }

      uint64_t last_ts_ms = 0;
      uint32_t cur = route_start;

      while (route_ok && cur < route_end) {
        NandStorageService::Record *batch = batch_buf.get();
        uint64_t *ts_ms = ts_buf.get();
        uint32_t n = 0;

        while (n < GO_SYNC_BATCH_MAX && cur < route_end) {
          NandStorageService::Record r;
          nread = 0;
          err = storage_->read_range_sync(cur, &r, 1, &nread, to);
          if (err != ESP_OK || nread != 1) {
            ESP_LOGW(GO_TAG, "sync: read failed at idx=%" PRIu32 ": %s", cur, esp_err_to_name(err));
            ESP_LOGW(GO_TAG, "sync: failed; keeping log");
            return true;
          }

          const uint32_t abs_idx = cur;
          uint64_t t = r.timestamp_ms;
          if (t == 0) {
            if (last_ts_ms != 0) {
              t = last_ts_ms + interval_ms;
            } else if (abs_idx < anchor_idx) {
              const uint32_t diff = anchor_idx - abs_idx;
              const uint64_t backfill = (uint64_t)diff * interval_ms;
              if (anchor_ts_ms <= backfill) {
                t = 0;
              } else {
                t = anchor_ts_ms - backfill;
              }
            } else {
              // abs_idx==anchor_idx should have had a non-zero timestamp.
              t = 0;
            }
          }
          if (t == 0) {
            ESP_LOGW(GO_TAG, "sync: route=%" PRIu32 " idx=%" PRIu32 " timestamp unresolved",
                     route_id, abs_idx);
            route_ok = false;
            break;
          }
          last_ts_ms = t;

          batch[n] = r;
          ts_ms[n] = t;
          n += 1;
          cur += 1;
        }

        if (!route_ok) {
          break;
        }
        if (n == 0) {
          break;
        }

        // Stable sort by timestamp ascending.
        for (uint32_t i = 1; i < n; ++i) {
          const NandStorageService::Record r = batch[i];
          const uint64_t t = ts_ms[i];
          uint32_t j = i;
          while (j > 0 && ts_ms[j - 1] > t) {
            batch[j] = batch[j - 1];
            ts_ms[j] = ts_ms[j - 1];
            j -= 1;
          }
          batch[j] = r;
          ts_ms[j] = t;
        }

        const std::string payload = build_measures_payload(batch, ts_ms, n);

        if (payload.empty()) {
          ESP_LOGW(GO_TAG, "sync: json build failed");
          route_ok = false;
          break;
        }

        ESP_LOGI(GO_TAG, "sync: post route=%" PRIu32 " n=%" PRIu32, route_id, n);
        if (!post_request(_serial_number, payload)) {
          ESP_LOGW(GO_TAG, "sync: post failed; skipping route=%" PRIu32, route_id);
          route_ok = false;
          break;
        }
      }

      if (route_ok) {
        any_route_sent = true;
        ESP_LOGI(GO_TAG, "sync: route=%" PRIu32 " ok", route_id);
      }

      idx = route_end;
    }

    if (!any_route_sent) {
      ESP_LOGW(GO_TAG, "sync: no routes sent; keeping log");
      return true;
    }

    err = storage_->clear_sync(to);
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "sync: clear failed: %s", esp_err_to_name(err));
      return true;
    }

    ESP_LOGI(GO_TAG, "sync: cleared");
    return true;
  }

  void _sync_end(void) {
    // TODO: stop Wi-Fi / cleanup.
    ESP_LOGI(GO_TAG, "sync: end");
    if (_sync_wifi_connected) {
      wifi_disconnect();
      _sync_wifi_connected = false;
    }

    _dashboard_update_status_only_();
  }

  void _tracking_begin(void) {
    // TODO: initialize tracking cycle.
    ESP_LOGI(GO_TAG, "tracking: begin route=%05" PRIu32, tracking_session_id_);
  }

  bool _tracking_step(void) {
    // TODO: measure -> save to storage -> display.
    log_bq25629_debug_snapshot(charger_);
    log_sensor_summary_banner();

    // Keep non-PM measurements running even if PM sensor is missing.
    PMData pm;
    pm.pm_01 = MeasuresInvalid::PM;
    pm.pm_25 = MeasuresInvalid::PM;
    pm.pm_10 = MeasuresInvalid::PM;
    pm.pm_01_sp = MeasuresInvalid::PM;
    pm.pm_25_sp = MeasuresInvalid::PM;
    pm.pm_10_sp = MeasuresInvalid::PM;
    pm.pm_03_pc = MeasuresInvalid::PM;
    pm.pm_05_pc = MeasuresInvalid::PM;
    pm.pm_01_pc = MeasuresInvalid::PM;
    pm.pm_25_pc = MeasuresInvalid::PM;
    pm.pm_5_pc = MeasuresInvalid::PM;
    pm.pm_10_pc = MeasuresInvalid::PM;

    if (pm_sensor_ == nullptr) {
      ESP_LOGW(GO_TAG, "PM sensor not initialized");
    } else if (!pm_sensor_->read(pm)) {
      ESP_LOGW(GO_TAG, "PM sensor read failed");
    }

    if (pm.is_pm_25_valid()) {
      ESP_LOGI(GO_TAG, "pm25: %.1f", pm.pm_25);
    }

    if (pm.is_pm_01_valid()) {
      ESP_LOGI(GO_TAG, "pm1.0: %.1f", pm.pm_01);
    }
    if (pm.is_pm_10_valid()) {
      ESP_LOGI(GO_TAG, "pm10: %.1f", pm.pm_10);
    }
    if (pm.is_pm_05_pc_valid()) {
      ESP_LOGI(GO_TAG, "count 0.5: %.1f", pm.pm_05_pc);
    }
    if (pm.is_pm_01_pc_valid()) {
      ESP_LOGI(GO_TAG, "count 1.0: %.1f", pm.pm_01_pc);
    }
    if (pm.is_pm_25_pc_valid()) {
      ESP_LOGI(GO_TAG, "count 2.5: %.1f", pm.pm_25_pc);
    }
    if (pm.is_pm_10_pc_valid()) {
      ESP_LOGI(GO_TAG, "count 10: %.1f", pm.pm_10_pc);
    }

    GPSService::Data d = {};
    bool gps_ok = false;
    if (gps_ != nullptr) {
      d = gps_->get();
      gps_ok = true;
      log_gps_data(d);
    }

    // Read additional sensors once (used for both UI + storage).
    CO2Data co2 = {};
    bool co2_valid = false;
    TempHumData th = {};
    bool th_temp_valid = false;
    bool th_hum_valid = false;
    if (co2_sensor_ != nullptr) {
      if (co2_sensor_->read(co2) && co2.is_valid()) {
        co2_valid = true;
        ESP_LOGI(GO_TAG, "co2: %d", co2.co2);
        if (co2_sensor_->support_temp_hum()) {
          th = co2_sensor_->temp_hum_data();
          th_temp_valid = th.is_temp_valid();
          th_hum_valid = th.is_hum_valid();
          if (th_temp_valid) {
            ESP_LOGI(GO_TAG, "temp: %.2f", th.temperature);
          }
          if (th_hum_valid) {
            ESP_LOGI(GO_TAG, "rhum: %.2f", th.humidity);
          }
        }
      }
    }

    if (scd4x_ != nullptr && scd4x_->initialized) {
      bool ready = false;
      const int16_t rdy_err = scd4x_get_data_ready_status(&ready);
      if (rdy_err == 0 && ready) {
        uint16_t ppm = 0;
        uint16_t t_raw = 0;
        uint16_t rh_raw = 0;
        const int16_t meas_err = scd4x_read_measurement_raw(&ppm, &t_raw, &rh_raw);
        if (meas_err == 0 && ppm != 0) {
          scd4x_last_valid_ = true;
          scd4x_last_ppm_ = ppm;
          ESP_LOGI(GO_TAG, "scd4x: co2=%u", (unsigned)ppm);
        } else if (meas_err != 0) {
          ESP_LOGW(GO_TAG, "scd4x read failed: %d", (int)meas_err);
        }
      }
    }

    if (s12_ != nullptr) {
      uint16_t ppm = 0;
      const esp_err_t err = s12_i2c_read_co2_ppm(s12_, &ppm);
      if (err == ESP_OK && ppm > 0 && ppm <= 32000) {
        s12_last_valid_ = true;
        s12_last_ppm_ = ppm;
        ESP_LOGI(GO_TAG, "s12: co2=%u", (unsigned)ppm);
      } else if (err != ESP_OK) {
        ESP_LOGW(GO_TAG, "s12 read failed: %s", esp_err_to_name(err));
      }
    }

    if (sunrise_ != nullptr) {
      uint16_t ppm = 0;
      uint8_t e_status = 0;
      const esp_err_t err = sunrise_i2c_read_co2_ppm(sunrise_, &ppm, &e_status);
      if (err == ESP_OK && ppm > 0 && ppm <= 32000) {
        sunlight_last_valid_ = true;
        sunlight_last_ppm_ = ppm;
        ESP_LOGI(GO_TAG, "sunlight: co2=%u", (unsigned)ppm);
      } else if (err != ESP_OK) {
        ESP_LOGW(GO_TAG, "sunlight read failed: %s", esp_err_to_name(err));
      }
    }

    TVOCNOxData gas = {};
    bool tvoc_valid = false;
    bool nox_valid = false;
    if (tvoc_nox_sensor_ != nullptr) {
      if (tvoc_nox_sensor_->read(gas)) {
        tvoc_valid = gas.is_tvoc_raw_valid();
        nox_valid = gas.is_nox_raw_valid();
        if (tvoc_valid) {
          ESP_LOGI(GO_TAG, "tvoc raw: %d", gas.tvoc_raw);
        }
        if (nox_valid) {
          ESP_LOGI(GO_TAG, "nox raw: %d", gas.nox_raw);
        }
      }
    }

    dps368_data_t dps = {};
    bool pressure_valid = false;
    if (dps368_ != nullptr) {
      const esp_err_t err = dps368_read(dps368_, &dps);
      if (err == ESP_OK && dps.pressure_valid) {
        pressure_valid = true;
        ESP_LOGI(GO_TAG, "pressure: %.1fPa", dps.pressure_pa);
      }
    }

    _sample_battery_percent();

    if (dash_ != nullptr) {
      dashboard::Values v = dash_values_;
      v.status_mask = _dashboard_status_mask_(gps_ok, d);

      if (gps_ok && d.utc.time_valid) {
        v.hour = (uint8_t)d.utc.hour;
        v.minute = (uint8_t)d.utc.min;
      }

      if (pm.is_pm_25_valid()) {
        v.pm25_ugm3 = (int)lroundf(pm.pm_25);
      }

      if (co2_valid) {
        v.co2_ppm = co2.co2;
      }
      if (th_temp_valid) {
        v.temperature_c = (int)lroundf(th.temperature);
      }
      if (th_hum_valid) {
        v.humidity_pct = (int)lroundf(th.humidity);
      }

      if (battery_percent_ok_) {
        const int bp = (battery_percent_ < 0) ? 0 : ((battery_percent_ > 100) ? 100 : battery_percent_);
        v.battery_pct = (uint8_t)bp;
      }
      if (charger_ != nullptr) {
        bool ch = false;
        if (charger_->is_charging(ch) == ESP_OK) {
          v.is_battery_charging = ch;
        }
      }

      dash_->update(v);
      dash_values_ = v;
    }

#if TRACKING_DISPLAY_SLEEP == 1
    if (dash_ != nullptr) {
      dash_->deep_sleep();
    }
#endif

    NandStorageService::Record rec;
    rec.id = tracking_session_id_;
    {
      uint64_t epoch_ms = 0;
      if (gps_ok) {
        (void)utc_to_epoch_ms(d.utc, &epoch_ms);
      }
      rec.timestamp_ms = epoch_ms;
    }

    if (pm.is_pm_01_valid()) {
      rec.pm01_ugm3_x10 = go_utils::pm_ugm3_to_x10(pm.pm_01);
    }
    if (pm.is_pm_25_valid()) {
      rec.pm25_ugm3_x10 = go_utils::pm_ugm3_to_x10(pm.pm_25);
    }
    if (pm.is_pm_10_valid()) {
      rec.pm10_ugm3_x10 = go_utils::pm_ugm3_to_x10(pm.pm_10);
    }

    if (pm.is_pm_05_pc_valid()) {
      rec.pc05_x10 = go_utils::count_to_x10(pm.pm_05_pc);
    }
    if (pm.is_pm_01_pc_valid()) {
      rec.pc10_x10 = go_utils::count_to_x10(pm.pm_01_pc);
    }
    if (pm.is_pm_25_pc_valid()) {
      rec.pc25_x10 = go_utils::count_to_x10(pm.pm_25_pc);
    }
    if (pm.is_pm_10_pc_valid()) {
      rec.pc100_x10 = go_utils::count_to_x10(pm.pm_10_pc);
    }

    // CO2 + temperature/humidity.
    if (co2_sensor_ != nullptr) {
      if (co2_valid) {
        rec.co2_ppm = go_utils::u16_from_int_nonneg(co2.co2);
        if (th_temp_valid) {
          rec.temperature_c_x100 = go_utils::temp_c_to_x100(th.temperature);
        }
        if (th_hum_valid) {
          rec.humidity_rh_x100 = go_utils::hum_rh_to_x100(th.humidity);
        }
      }
    }

    if (scd4x_last_valid_) {
      rec.scd4x = scd4x_last_ppm_;
    }

    if (s12_last_valid_) {
      rec.s12 = s12_last_ppm_;
    }
    if (sunlight_last_valid_) {
      rec.sunlight = sunlight_last_ppm_;
    }

    // Pressure.
    if (pressure_valid) {
      rec.pressure_pa = go_utils::pressure_pa_from_float(dps.pressure_pa);
    }

    // TVOC/NOx.
    if (tvoc_valid) {
      rec.tvoc_raw = go_utils::u16_from_int_nonneg(gas.tvoc_raw);
    }
    if (nox_valid) {
      rec.nox_raw = go_utils::u16_from_int_nonneg(gas.nox_raw);
    }

    if (gps_ok && d.fix_valid) {
      rec.latitude_e7 = go_utils::deg_to_e7(d.latitude_deg);
      rec.longitude_e7 = go_utils::deg_to_e7(d.longitude_deg);
    }

    _ble_notify(rec, gps_ok, d);

    _tracking_write_record(rec);
    return true;
  }

  void _tracking_write_record(const NandStorageService::Record &r) {
    if (storage_ == nullptr) {
      return;
    }
    if (!storage_->is_ready()) {
      ESP_LOGW(GO_TAG, "storage not ready; skip write");
      return;
    }

    const TickType_t to = pdMS_TO_TICKS(GO_NAND_CMD_TIMEOUT_MS);
    esp_err_t err = storage_->enqueue_record(r, true, to);
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "storage enqueue failed: %s", esp_err_to_name(err));
      return;
    }

#if NO_INACTIVE_NO_SLEEP == 0
    err = storage_->flush_sync(to);
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "storage flush failed: %s", esp_err_to_name(err));
      return;
    }
#endif
  }

  void _tracking_end(void) {
    // TODO: finalize tracking cycle (flush storage, etc.).
    ESP_LOGI(GO_TAG, "tracking: end (stub)");
  }

  void _tracking_enter_sleep(void) {

#if NO_INACTIVE_NO_SLEEP == 1
    _tracking_next_cycle_ms = now_ms() + tracking_sleep_interval_s_ * 1000U;
    _tracking_started = false;
    return;
#endif // NO_INACTIVE_NO_SLEEP == 1

    ESP_LOGI(GO_TAG, "tracking: entering deep sleep (stub)");

    esp_err_t err = ESP_OK;
    if (buttons_ != nullptr) {
      err = buttons_->enable_deep_sleep_wakeup();
    } else {
      ESP_LOGE(GO_TAG, "deep sleep wake config failed: buttons not initialized");
      return;
    }
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "deep sleep wake config failed: %s", esp_err_to_name(err));
    }

    err = esp_sleep_enable_timer_wakeup((uint64_t)tracking_sleep_interval_s_ * 1000000ULL);
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "timer wake config failed: %s", esp_err_to_name(err));
    }

    RTC_LAST_STATE = State::Tracking;
    esp_deep_sleep_start();
  }
};

static void on_button_event(void *arg, esp_event_base_t base, int32_t id, void *event_data) {
  (void)base;
  if (arg == nullptr) {
    return;
  }
  if (event_data == nullptr) {
    return;
  }
  GoController *go = static_cast<GoController *>(arg);
  go->OnButtonEvent(id, static_cast<const ButtonService::Payload *>(event_data));
}

extern "C" void app_main(void) {
  vTaskDelay(pdMS_TO_TICKS(100));
  // Re-initialize console after deepsleep
  esp_sleep_wakeup_cause_t wakeUpReason = esp_sleep_get_wakeup_cause();
  if (wakeUpReason != ESP_SLEEP_WAKEUP_UNDEFINED) {
    initConsole();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  esp_log_level_set(GO_TAG, ESP_LOG_INFO);
  sleep_ms(GO_BOOT_DELAY_MS);

  ESP_ERROR_CHECK(init_ext_watchdog());
  reset_ext_watchdog();
  const uint32_t wdt_last_reset_ms = now_ms();
  uint32_t bq_wdt_last_reset_ms = now_ms();

  i2c_master_bus_config_t bus_cfg = {};
  bus_cfg.i2c_port = GO_I2C_MASTER_PORT;
  bus_cfg.sda_io_num = (gpio_num_t)GO_I2C_MASTER_SDA_IO;
  bus_cfg.scl_io_num = (gpio_num_t)GO_I2C_MASTER_SCL_IO;
  bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_cfg.glitch_ignore_cnt = GO_I2C_GLITCH_IGNORE_CNT;
  bus_cfg.flags.enable_internal_pullup = GO_I2C_INTERNAL_PULLUPS;

  i2c_master_bus_handle_t bus_handle = nullptr;
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

  // Some I2C targets need a short time after boot before responding reliably.
  sleep_ms(50);

  static Scd4xTest scd4x;
  Scd4xTest *scd4x_ptr = nullptr;
  {
    sensirion_i2c_hal_set_bus_handle(bus_handle);
    sensirion_i2c_hal_init();

    scd4x_init(SCD41_I2C_ADDR_62);

    int16_t err = scd4x_wake_up();
    if (err != 0) {
      ESP_LOGW(GO_TAG, "SCD4x wake_up failed: %d", (int)err);
    }

    (void)scd4x_stop_periodic_measurement();
    (void)scd4x_reinit();

    err = scd4x_start_periodic_measurement();
    if (err != 0) {
      ESP_LOGW(GO_TAG, "SCD4x start_periodic_measurement failed: %d", (int)err);
    } else {
      scd4x.initialized = true;
      scd4x_ptr = &scd4x;
      ESP_LOGI(GO_TAG, "SCD4x initialized (test-only)");
    }
  }

  // Senseair I2C CO2 (test-only). Both S12 and Sunlight (Sunrise) share address 0x68.
  static s12_i2c_t s12;
  s12_i2c_t *s12_ptr = nullptr;
  static sunrise_i2c_t sunrise;
  sunrise_i2c_t *sunrise_ptr = nullptr;
  {
    const esp_err_t probe = i2c_master_probe(bus_handle, 0x68, 20);
    if (probe != ESP_OK) {
      ESP_LOGW(GO_TAG, "Senseair I2C CO2 not detected at 0x68: %s", esp_err_to_name(probe));
    } else {
      // Detect S12 by its firmware type register (0x2F == 0xC2 per S12 docs).
      esp_err_t err = s12_i2c_create(bus_handle, S12_I2C_ADDR_DEFAULT, 100000, &s12);
      if (err != ESP_OK) {
        ESP_LOGW(GO_TAG, "S12 init failed: %s", esp_err_to_name(err));
      } else {
        const uint8_t reg = 0x2F;
        uint8_t fw_type = 0;
        err = i2c_master_transmit_receive(s12.dev, &reg, 1, &fw_type, 1, 1000);
        if (err == ESP_OK && fw_type == 0xC2) {
          s12_ptr = &s12;
          ESP_LOGI(GO_TAG, "S12 detected (test-only)");
        } else {
          if (err != ESP_OK) {
            ESP_LOGW(GO_TAG, "S12 detect failed: %s", esp_err_to_name(err));
          }
          s12_i2c_destroy(&s12);
        }
      }

      if (s12_ptr == nullptr) {
        err = sunrise_i2c_create(bus_handle, SUNRISE_I2C_ADDR_DEFAULT, 100000, &sunrise);
        if (err != ESP_OK) {
          ESP_LOGW(GO_TAG, "Sunlight init failed: %s", esp_err_to_name(err));
        } else {
          uint8_t mode = 0;
          int period_ms = 0;
          err = sunrise_i2c_read_config(&sunrise, &mode, &period_ms);
          if (err == ESP_OK) {
            sunrise_ptr = &sunrise;
            ESP_LOGI(GO_TAG, "Sunlight detected (test-only; BLE field name 'sunlight') mode=%u period_ms=%d",
                     (unsigned)mode, period_ms);
          } else {
            ESP_LOGW(GO_TAG, "Sunlight detect failed: %s", esp_err_to_name(err));
            sunrise_i2c_destroy(&sunrise);
          }
        }
      }

      if (s12_ptr == nullptr && sunrise_ptr == nullptr) {
        ESP_LOGW(GO_TAG, "Senseair I2C CO2 init failed: neither S12 nor Sunlight responded");
      }
    }
  }

  dps368_handle_t *dps368_ptr = nullptr;
  {
    const esp_err_t err = dps368_init(bus_handle, DPS368_I2C_ADDR_SDO_VDD, &dps368_ptr);
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "DPS368 init failed: %s", esp_err_to_name(err));
      dps368_ptr = nullptr;
    }
  }

  drivers::BQ25629 *charger_ptr = nullptr;
  {
    const esp_err_t err = init_charger(bus_handle, &charger_ptr);
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "BQ25629 init failed: %s", esp_err_to_name(err));
      charger_ptr = nullptr;
    } else {
      bq_wdt_last_reset_ms = now_ms();
    }
  }

  TVOCNOxSensor *tvoc_nox_sensor_ptr = nullptr;
  static SGP41 sgp41(bus_handle);
  {
    if (!sgp41.init()) {
      ESP_LOGW(GO_TAG, "SGP41 init failed");
    } else {
      tvoc_nox_sensor_ptr = &sgp41;
    }
  }

  CO2Sensor *co2_sensor_ptr = nullptr;
  static STCC4Sensor stcc4(bus_handle);
  {
    if (!stcc4.init()) {
      ESP_LOGW(GO_TAG, "STCC4 init failed");
    } else {
      co2_sensor_ptr = &stcc4;
    }
  }

  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = GO_SPI_MOSI_GPIO;
  buscfg.miso_io_num = GO_SPI_MISO_GPIO;
  buscfg.sclk_io_num = GO_SPI_SCLK_GPIO;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = GO_SPI_MAX_TRANSFER_SZ;
  ESP_ERROR_CHECK(spi_bus_initialize(GO_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

  dashboard::Dashboard *dash_ptr = nullptr;
  {
    constexpr dashboard::display_driver::Config display_cfg{
        GO_SPI_HOST,
        GO_EPD_CLOCK_SPEED_HZ,
        0,
        GO_EPD_CS_GPIO,
        GO_EPD_DC_GPIO,
        GO_EPD_RST_GPIO,
        GO_EPD_BUSY_GPIO,
    };
    static dashboard::Dashboard dash(dashboard::Config{20, display_cfg});

    dashboard::Values v{400, 0, 0, 0, 0, 0, 0, false, 0};
    if (charger_ptr != nullptr) {
      uint8_t pct = 0;
      if (charger_ptr->estimate_battery_percent(pct) == ESP_OK) {
        v.battery_pct = pct;
      }
      bool ch = false;
      if (charger_ptr->is_charging(ch) == ESP_OK) {
        v.is_battery_charging = ch;
      }
    }

    const esp_err_t err = dash.init(v);
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "dashboard init failed: %s", esp_err_to_name(err));
      dash_ptr = nullptr;
    } else {
      dash_ptr = &dash;
    }
  }

  static NandStorageService storage;
  NandStorageService *storage_ptr = nullptr;
  {
    NandStorageService::Config scfg;
    scfg.spi_host = GO_SPI_HOST;
    scfg.cs_pin = GO_NAND_CS_GPIO;
    scfg.clock_speed_hz = GO_NAND_CLOCK_SPEED_HZ;
    scfg.mount_path = GO_NAND_MOUNT_PATH;
    scfg.records_path = GO_NAND_RECORDS_PATH;

    esp_err_t err = storage.init(scfg);
    if (err == ESP_OK) {
      err = storage.start();
    }
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "storage init/start failed: %s", esp_err_to_name(err));
      storage_ptr = nullptr;
    } else {
      storage_ptr = &storage;
      for (uint32_t i = 0; i < GO_NAND_READY_WAIT_RETRIES; ++i) {
        if (storage.is_ready()) {
          break;
        }
        sleep_ms(GO_NAND_READY_WAIT_DELAY_MS);
      }
      if (!storage.is_ready()) {
        ESP_LOGW(GO_TAG, "storage not ready yet; will start logging when ready");
      }
    }
  }

  ButtonService::Config bcfg;
  bcfg.qon_gpio = GO_BUTTON_QON_GPIO;
  bcfg.boot_gpio = GO_BUTTON_BOOT_GPIO;
  bcfg.cap_alert_gpio = GO_TOUCH_ALERT_GPIO;
  bcfg.cap_alert_active_low = GO_TOUCH_ALERT_ACTIVE_LOW;
  bcfg.qon_active_low = GO_BUTTON_QON_ACTIVE_LOW;
  bcfg.boot_active_low = GO_BUTTON_BOOT_ACTIVE_LOW;
  bcfg.cap_required = GO_TOUCH_REQUIRED;
  bcfg.debounce_ms = GO_BUTTON_DEBOUNCE_MS;
  bcfg.long_press_ms = GO_BUTTON_LONG_PRESS_MS;
  bcfg.touch_enable_mask = GO_TOUCH_ENABLE_MASK;
  bcfg.touch_interrupt_enable_mask = GO_TOUCH_INTERRUPT_ENABLE_MASK;
  bcfg.touch_calibrate = GO_TOUCH_CALIBRATE;
  bcfg.touch_calibrate_mask = GO_TOUCH_CALIBRATE_MASK;
  bcfg.touch_delta_sense = 0;

  ButtonService buttons(bus_handle, bcfg);
  ESP_ERROR_CHECK(buttons.init());

  static GPSService gps;
  GPSService *gps_ptr = nullptr;
  {
    GPSService::Config gps_cfg;
    gps_cfg.uart_num = GO_GPS_UART_PORT;
    gps_cfg.rx_pin = GO_GPS_UART_RX_GPIO;
    gps_cfg.tx_pin = GO_GPS_UART_TX_GPIO;
    gps_cfg.baud_rate = GO_GPS_UART_BAUD;
    gps_cfg.log_raw_nmea = GO_GPS_LOG_RAW_NMEA;

    esp_err_t err = gps.init(gps_cfg);
    if (err == ESP_OK) {
      err = gps.start();
    }
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "GPS init/start failed: %s", esp_err_to_name(err));
      gps_ptr = nullptr;
    } else {
      gps_ptr = &gps;
    }
  }

  PMSensor *pm_sensor_ptr = nullptr;
  static SPS30Sensor sps30(bus_handle);
  {
    esp_err_t err = pm_power_on();
    if (err != ESP_OK) {
      ESP_LOGW(GO_TAG, "PM sensor power on failed: %s", esp_err_to_name(err));
    } else if (!sps30.init()) {
      ESP_LOGW(GO_TAG, "PM sensor init failed");
    } else {
      sleep_ms(GO_SPS30_WARMUP_DELAY_MS);
      pm_sensor_ptr = &sps30;
    }
  }

  QueueHandle_t input_queue = xQueueCreate((UBaseType_t)GO_INPUT_QUEUE_LEN, sizeof(GoInputEvent));
  if (input_queue == nullptr) {
    ESP_LOGE(GO_TAG, "input queue create failed");
    return;
  }

  static BLEStream ble;
  GoController go(&buttons, input_queue, pm_sensor_ptr, tvoc_nox_sensor_ptr, co2_sensor_ptr,
                  dps368_ptr, scd4x_ptr, s12_ptr, sunrise_ptr, &ble, bus_handle, gps_ptr, dash_ptr,
                  storage_ptr,
                  charger_ptr, wdt_last_reset_ms, bq_wdt_last_reset_ms);
  ESP_ERROR_CHECK(
      esp_event_handler_register(BUTTON_SERVICE_EVENT, ESP_EVENT_ANY_ID, &on_button_event, &go));

  ESP_LOGI(GO_TAG, "boot");
  go.Run();
}
