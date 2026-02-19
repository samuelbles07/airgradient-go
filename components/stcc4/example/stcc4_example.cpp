#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "stcc4.h"

static const char *TAG = "STCC4_EXAMPLE";

// Example task: Continuous CO2 measurement
void stcc4_continuous_example(void *arg) {
    stcc4_dev_t sensor;
    stcc4_measurement_t measurement;
    
    // Initialize sensor on I2C bus
    // Note: You must create the I2C bus handle elsewhere in your main app
    // This is just a placeholder example
    
    esp_err_t ret = stcc4_init(&sensor, (i2c_master_bus_handle_t)arg, STCC4_I2C_ADDR_DEFAULT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize STCC4: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    
    // Verify sensor communication
    uint32_t product_id;
    uint64_t serial_num;
    ret = stcc4_get_product_id(&sensor, &product_id, &serial_num);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get product ID - sensor not responding");
        stcc4_deinit(&sensor);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "STCC4 sensor found - Product ID: 0x%08X", product_id);
    
    // Perform self-test
    stcc4_self_test_result_t test;
    ret = stcc4_perform_self_test(&sensor, &test);
    ESP_LOGI(TAG, "Self-test: %s", ret == ESP_OK ? "PASSED" : "FAILED");
    
    // Condition sensor after long idle
    ESP_LOGI(TAG, "Conditioning sensor...");
    stcc4_perform_conditioning(&sensor);
    
    // Start continuous measurement
    ESP_LOGI(TAG, "Starting continuous measurement...");
    stcc4_start_continuous_measurement(&sensor);
    
    // Read measurements every second
    for (int i = 0; i < 60; i++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        ret = stcc4_read_measurement(&sensor, &measurement);
        if (ret == ESP_OK) {
            printf("[%d] CO2: %u ppm, Temp: %.2f°C, RH: %.1f%%, Status: 0x%04X\n",
                   i,
                   measurement.co2_ppm,
                   measurement.temperature_c,
                   measurement.humidity_rh,
                   measurement.sensor_status);
        } else {
            ESP_LOGW(TAG, "Failed to read measurement: %s", esp_err_to_name(ret));
        }
    }
    
    // Stop measurement
    stcc4_stop_continuous_measurement(&sensor);
    stcc4_deinit(&sensor);
    
    vTaskDelete(NULL);
}

// Example task: Single-shot low-power measurement
void stcc4_single_shot_example(void *arg) {
    stcc4_dev_t sensor;
    stcc4_measurement_t measurement;
    
    esp_err_t ret = stcc4_init(&sensor, (i2c_master_bus_handle_t)arg, STCC4_I2C_ADDR_DEFAULT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize STCC4: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Single-shot measurement mode (low power)");
    
    for (int cycle = 0; cycle < 10; cycle++) {
        // Wake from sleep
        stcc4_exit_sleep_mode(&sensor);
        vTaskDelay(pdMS_TO_TICKS(10));
        
        // Perform measurement
        stcc4_measure_single_shot(&sensor);
        
        // Read result
        ret = stcc4_read_measurement(&sensor, &measurement);
        if (ret == ESP_OK) {
            printf("Cycle %d: CO2: %u ppm, Temp: %.2f°C, RH: %.1f%%\n",
                   cycle,
                   measurement.co2_ppm,
                   measurement.temperature_c,
                   measurement.humidity_rh);
        }
        
        // Return to sleep (only 1 µA current)
        stcc4_enter_sleep_mode(&sensor);
        
        // Sleep for 10 seconds (configurable interval)
        ESP_LOGI(TAG, "Sleeping...");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
    
    stcc4_deinit(&sensor);
    vTaskDelete(NULL);
}

// Example: Using with external temperature/humidity sensor
void stcc4_with_rht_compensation_example(void *arg) {
    stcc4_dev_t sensor;
    stcc4_measurement_t measurement;
    
    esp_err_t ret = stcc4_init(&sensor, (i2c_master_bus_handle_t)arg, STCC4_I2C_ADDR_DEFAULT);
    if (ret != ESP_OK) {
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Starting continuous measurement with RHT compensation");
    stcc4_start_continuous_measurement(&sensor);
    
    for (int i = 0; i < 30; i++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // Update external RHT values (e.g., from SHT4x sensor)
        // These should be read from your external sensor
        float external_temp = 22.5f + (i * 0.01f);  // Simulated values
        float external_humidity = 45.0f + (i * 0.2f);
        
        stcc4_set_rht_compensation(&sensor, external_temp, external_humidity);
        
        ret = stcc4_read_measurement(&sensor, &measurement);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "CO2: %u ppm (T: %.2f°C, RH: %.1f%%)",
                     measurement.co2_ppm,
                     measurement.temperature_c,
                     measurement.humidity_rh);
        }
    }
    
    stcc4_stop_continuous_measurement(&sensor);
    stcc4_deinit(&sensor);
    vTaskDelete(NULL);
}

// Example: Forced recalibration
void stcc4_frc_example(void *arg) {
    stcc4_dev_t sensor;
    
    esp_err_t ret = stcc4_init(&sensor, (i2c_master_bus_handle_t)arg, STCC4_I2C_ADDR_DEFAULT);
    if (ret != ESP_OK) {
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Starting FRC calibration example");
    
    // Start continuous measurement
    stcc4_start_continuous_measurement(&sensor);
    
    // Let sensor stabilize for 30+ seconds
    ESP_LOGI(TAG, "Stabilizing sensor (30 seconds)...");
    for (int i = 0; i < 30; i++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    // Stop continuous mode before FRC
    stcc4_stop_continuous_measurement(&sensor);
    
    // Perform FRC with known target CO2 concentration (e.g., 400 ppm fresh air)
    int16_t correction;
    ret = stcc4_perform_forced_recalibration(&sensor, 400, &correction);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "FRC successful! Correction applied: %d ppm", correction);
    } else {
        ESP_LOGE(TAG, "FRC failed!");
    }
    
    stcc4_deinit(&sensor);
    vTaskDelete(NULL);
}
