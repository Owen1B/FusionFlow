#ifndef CONFIG_H
#define CONFIG_H

/**
 * @brief System configuration constants and parameters
 * 
 * This header contains all configurable parameters for the smart infusion
 * monitoring system, making it easy to adjust system behavior without
 * modifying core algorithm logic.
 */

// Hardware Pin Configuration
namespace HardwarePins {
    constexpr int WATER_SENSOR_PIN = 11;
    constexpr int NEOPIXEL_PIN = 48;
    constexpr int I2C_SDA_PIN = 36;
    constexpr int I2C_SCL_PIN = 1;
    constexpr int HX711_DATA_PIN = 17;
    constexpr int HX711_CLOCK_PIN = 18;
    constexpr int INIT_BUTTON_PIN = 15;
    constexpr int ABNORMAL_RESET_BUTTON_PIN = 0;
    constexpr int MOTOR_PIN1 = 6;
    constexpr int MOTOR_PIN2 = 7;
}

// Network Configuration
namespace NetworkConfig {
    // Note: In production, these should be loaded from EEPROM or config file
    const char* const WIFI_SSID = "YOUR_WIFI_SSID";
    const char* const WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
    const char* const API_BASE_URL = "YOUR_API_BASE_URL";
    const char* const DEVICE_ID = "001";
    constexpr uint16_t WEBSOCKET_PORT = 81;
    constexpr uint16_t HTTP_PORT = 80;
}

// Sensor Configuration
namespace SensorConfig {
    constexpr float HX711_CALIBRATION_FACTOR = 1687.0f;
    constexpr float EQUIPMENT_TARE_WEIGHT = 12.0f;  // grams
    constexpr float EMPTY_BAG_TARE_WEIGHT = 60.0f;  // grams
    constexpr float TOTAL_TARE_WEIGHT = EQUIPMENT_TARE_WEIGHT + EMPTY_BAG_TARE_WEIGHT;
}

// Kalman Filter Parameters
namespace FilterParams {
    // Weight Kalman Filter
    constexpr float WEIGHT_KF_SIGMA_A = 0.0005f;
    constexpr float WEIGHT_KF_SIGMA_J = 1e-6f;
    constexpr float WEIGHT_KF_R_NOISE = 50.0f;
    
    // Drip Kalman Filter
    constexpr float DRIP_KF_SIGMA_A = 0.00001f;
    constexpr float DRIP_KF_R_NOISE = 0.05f;
    constexpr float DRIP_KF_WPD_Q_NOISE = 0.00000001f;
    constexpr float DRIP_KF_WPD_R_NOISE = 0.0001f;
    
    // Data Fusion Parameters
    constexpr float FUSION_Q_FLOW = 0.0000001f;
    constexpr float FUSION_R_WEIGHT_FLOW = 0.01f;
    constexpr float FUSION_R_DRIP_FLOW = 0.0005f;
    constexpr float FUSION_Q_WEIGHT = 0.01f;
    constexpr float FUSION_R_WEIGHT_WEIGHT = 1.0f;
    constexpr float FUSION_R_DRIP_WEIGHT = 1.0f;
}

// Timing Configuration
namespace TimingConfig {
    constexpr unsigned long MAIN_LOOP_INTERVAL_MS = 1000;
    constexpr unsigned long BUTTON_DEBOUNCE_MS = 200;
    constexpr unsigned long FAST_CONVERGENCE_DURATION_MS = 60000;
    constexpr unsigned long NO_DRIP_TIMEOUT_MS = 10000;
    constexpr unsigned long ABNORMALITY_CHECK_INTERVAL_MS = 10000;
    constexpr unsigned long MOTOR_RUN_DURATION_MS = 2000;
}

// Display Configuration
namespace DisplayConfig {
    constexpr int OLED_WIDTH = 128;
    constexpr int OLED_HEIGHT = 32;
    constexpr int NEOPIXEL_BRIGHTNESS = 50;
    constexpr int MAX_TIMESTAMP_QUEUE_SIZE = 20;
}

// WPD Calibration Configuration
namespace WPDConfig {
    constexpr unsigned long LONG_CALIBRATION_DURATION_MS = 60000;
    constexpr int MIN_DROPS_FOR_CALIBRATION = 30;
}

#endif // CONFIG_H