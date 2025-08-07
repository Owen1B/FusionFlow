#ifndef HARDWARE_MANAGER_H
#define HARDWARE_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_NeoPixel.h>
#include <HX711.h>
#include <HTTPClient.h>
#include "../include/Config.h"
#include "../include/SystemStateManager.h"

/**
 * @brief Hardware Manager class
 * 
 * Encapsulates all hardware interactions including sensors, displays,
 * network communications, and user inputs. Provides a clean interface
 * for the main application logic.
 */
class HardwareManager {
private:
    // Hardware objects
    Adafruit_NeoPixel neopixel_;
    U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C oled_;
    HX711 weight_sensor_;
    WiFiServer http_server_;
    WebSocketsServer websocket_server_;
    
    // Button states
    int last_init_button_state_;
    int last_reset_button_state_;
    unsigned long last_init_button_press_time_;
    unsigned long last_reset_button_press_time_;
    
    // LED state
    LEDColor current_led_color_;
    bool led_blink_state_;
    unsigned long last_led_blink_time_;
    
    // Network status
    bool wifi_connected_;
    bool websocket_client_connected_;
    
    // Motor control
    unsigned long motor_start_time_;
    bool motor_running_;

public:
    /**
     * @brief Constructor
     */
    HardwareManager();
    
    /**
     * @brief Initialize all hardware components
     * @return True if initialization successful
     */
    bool initialize();
    
    /**
     * @brief Update hardware states (should be called regularly)
     * @param current_time_ms Current system time
     */
    void update(unsigned long current_time_ms);
    
    // Sensor Methods
    /**
     * @brief Read raw weight from HX711 sensor
     * @return Weight reading in grams
     */
    float readWeightSensor();
    
    /**
     * @brief Calibrate weight sensor
     */
    void calibrateWeightSensor();
    
    // Display Methods
    /**
     * @brief Update OLED display with infusion data
     * @param ip_address Device IP address
     * @param progress_percent Infusion progress (0-100%)
     * @param remaining_weight_g Remaining liquid weight in grams
     * @param flow_rate_gps Flow rate in grams per second
     * @param remaining_time_min Remaining time in minutes (-1 if unknown)
     */
    void updateOLEDDisplay(const String& ip_address, 
                          float progress_percent,
                          float remaining_weight_g, 
                          float flow_rate_gps, 
                          long remaining_time_min);
    
    /**
     * @brief Set LED color and behavior
     * @param color LED color
     * @param should_blink True if LED should blink
     */
    void setLEDStatus(LEDColor color, bool should_blink = false);
    
    // Input Methods
    /**
     * @brief Check if initialization button was pressed
     * @param current_time_ms Current system time
     * @return True if button pressed (with debounce)
     */
    bool checkInitButtonPressed(unsigned long current_time_ms);
    
    /**
     * @brief Check if reset button was pressed
     * @param current_time_ms Current system time
     * @return True if button pressed (with debounce)
     */
    bool checkResetButtonPressed(unsigned long current_time_ms);
    
    // Network Methods
    /**
     * @brief Initialize WiFi connection
     * @return True if connection successful
     */
    bool initializeWiFi();
    
    /**
     * @brief Check WiFi connection status
     * @return True if WiFi is connected
     */
    bool isWiFiConnected() const { return wifi_connected_; }
    
    /**
     * @brief Get device IP address
     * @return IP address as string
     */
    String getIPAddress() const;
    
    /**
     * @brief Handle HTTP requests
     */
    void handleHTTPRequests();
    
    /**
     * @brief Handle WebSocket events
     */
    void handleWebSocketEvents();
    
    /**
     * @brief Send data to WebSocket clients
     * @param json_data JSON formatted data string
     */
    void sendWebSocketData(const String& json_data);
    
    /**
     * @brief Upload data to cloud server
     * @param json_payload JSON data to upload
     * @return True if upload successful
     */
    bool uploadToCloudServer(const String& json_payload);
    
    // Motor Control Methods
    /**
     * @brief Start motor in forward direction
     */
    void startMotorForward();
    
    /**
     * @brief Start motor in reverse direction
     */
    void startMotorReverse();
    
    /**
     * @brief Stop motor
     */
    void stopMotor();
    
    /**
     * @brief Update motor control (handle timing)
     * @param current_time_ms Current system time
     */
    void updateMotorControl(unsigned long current_time_ms);

private:
    /**
     * @brief Update LED blinking behavior
     * @param current_time_ms Current system time
     */
    void updateLEDBlink(unsigned long current_time_ms);
    
    /**
     * @brief Convert LEDColor enum to NeoPixel color
     * @param color LEDColor enum value
     * @return NeoPixel color value
     */
    uint32_t getLEDColorValue(LEDColor color);
    
    /**
     * @brief WebSocket event callback
     */
    static void webSocketEventCallback(uint8_t client_num, WStype_t type, 
                                     uint8_t* payload, size_t length);
};

#endif // HARDWARE_MANAGER_H