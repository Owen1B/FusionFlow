/**
 * @file main.cpp
 * @brief Smart Infusion Monitoring System - Main Application
 * 
 * This is the main application file for an intelligent infusion monitoring system
 * that uses dual-sensor data fusion (weight sensor + drip sensor) and Kalman
 * filtering algorithms for precise infusion monitoring.
 * 
 * Key Features:
 * - Dual-sensor data fusion for enhanced accuracy
 * - Extended Kalman Filter for sensor noise reduction
 * - Real-time web monitoring interface
 * - Automatic abnormality detection and alerting
 * - Professional modular architecture
 * 
 * @author [Your Name]
 * @date [Current Date]
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include "../include/Config.h"
#include "../include/SystemStateManager.h"
#include "../include/HardwareManager.h"
#include "../include/SensorDataProcessor.h"

// Global system components
SystemStateManager state_manager;
HardwareManager hardware;
SensorDataProcessor data_processor;

// Timing variables
unsigned long last_main_loop_time_ms = 0;
unsigned long last_data_upload_time_ms = 0;
unsigned long system_start_time_ms = 0;

// Data upload interval (5 seconds)
constexpr unsigned long DATA_UPLOAD_INTERVAL_MS = 5000;

/**
 * @brief Interrupt Service Routine for drip detection
 */
void IRAM_ATTR onDripDetected() {
    static unsigned long last_drip_time = 0;
    unsigned long current_time = millis();
    
    // Simple debouncing
    if (current_time - last_drip_time > 50) { // 50ms debounce
        data_processor.updateDropCount(1);
        state_manager.updateLastDripTime(current_time);
        last_drip_time = current_time;
    }
}

/**
 * @brief Initialize system components
 */
bool initializeSystem() {
    Serial.begin(115200);
    Serial.println("=== Smart Infusion Monitoring System ===");
    Serial.println("Initializing system components...");
    
    // Initialize hardware
    if (!hardware.initialize()) {
        Serial.println("ERROR: Hardware initialization failed");
        state_manager.transitionToState(SystemState::INIT_ERROR);
        return false;
    }
    
    // Initialize WiFi
    if (!hardware.initializeWiFi()) {
        Serial.println("ERROR: WiFi initialization failed");
        state_manager.transitionToState(SystemState::INIT_ERROR);
        return false;
    }
    
    // Set up drip detection interrupt
    pinMode(HardwarePins::WATER_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(HardwarePins::WATER_SENSOR_PIN), 
                   onDripDetected, FALLING);
    
    // Initialize system state manager
    state_manager.initialize();
    
    // Get initial weight measurement for data processor
    float initial_weight = hardware.readWeightSensor();
    data_processor.initialize(initial_weight);
    
    Serial.println("System initialization completed successfully");
    Serial.printf("Device IP: %s\n", hardware.getIPAddress().c_str());
    
    system_start_time_ms = millis();
    return true;
}

/**
 * @brief Handle button inputs and system control
 */
void handleUserInputs() {
    unsigned long current_time = millis();
    
    // Check for initialization button press
    if (hardware.checkInitButtonPressed(current_time)) {
        Serial.println("Init button pressed - restarting system");
        
        // Restart initialization process
        float initial_weight = hardware.readWeightSensor();
        data_processor.initialize(initial_weight);
        state_manager.transitionToState(SystemState::INITIALIZING);
    }
    
    // Check for reset button press (abnormality reset)
    if (hardware.checkResetButtonPressed(current_time)) {
        if (state_manager.hasInfusionAbnormality()) {
            Serial.println("Reset button pressed - clearing abnormality");
            state_manager.setInfusionAbnormality(false);
            state_manager.transitionToState(SystemState::NORMAL);
        }
    }
}

/**
 * @brief Process sensor data and update system state
 */
void processSensorData() {
    unsigned long current_time = millis();
    float delta_time_s = (current_time - last_main_loop_time_ms) / 1000.0f;
    
    // Read weight sensor
    float weight_measurement = hardware.readWeightSensor();
    
    // Process sensor data through filters
    ProcessingResult result = data_processor.processSensorData(
        weight_measurement, 0, delta_time_s);
    
    // Update fast convergence mode based on system state
    if (state_manager.shouldEnterFastConvergence() && 
        !data_processor.isFastConvergenceModeActive()) {
        
        data_processor.setFastConvergenceMode(true);
        state_manager.transitionToState(SystemState::FAST_CONVERGENCE);
        Serial.println("Entering fast convergence mode");
    }
    
    // Check for completion
    if (data_processor.isInfusionCompleted(result.fused_remaining_weight_g)) {
        if (state_manager.getCurrentState() != SystemState::COMPLETED) {
            state_manager.transitionToState(SystemState::COMPLETED);
            Serial.println("Infusion completed");
        }
    }
    
    // Update display
    float progress_percent = data_processor.calculateInfusionProgress(
        result.fused_remaining_weight_g);
    long remaining_time_min = (result.remaining_time_seconds > 0) ? 
        (long)(result.remaining_time_seconds / 60.0f) : -1;
    
    hardware.updateOLEDDisplay(
        hardware.getIPAddress(),
        progress_percent,
        result.fused_remaining_weight_g,
        result.fused_flow_rate_gps,
        remaining_time_min
    );
    
    // Output debug data
    Serial.printf("Weight: %.1fg, Flow: %.3fg/s, Remaining: %.1f%%, Time: %ldmin\n",
        result.filtered_weight_g, result.fused_flow_rate_gps, 
        progress_percent, remaining_time_min);
}

/**
 * @brief Upload data to cloud server
 */
void uploadDataToCloud() {
    unsigned long current_time = millis();
    
    if (current_time - last_data_upload_time_ms < DATA_UPLOAD_INTERVAL_MS) {
        return; // Not time to upload yet
    }
    
    // Create JSON payload
    DynamicJsonDocument doc(1024);
    doc["deviceId"] = NetworkConfig::DEVICE_ID;
    doc["timestamp"] = current_time;
    doc["systemState"] = state_manager.getStateDisplayName();
    doc["autoClamp"] = state_manager.isAutoClampEnabled();
    doc["totalDrops"] = data_processor.getTotalDropCount();
    
    String json_payload;
    serializeJson(doc, json_payload);
    
    // Upload to server
    if (hardware.uploadToCloudServer(json_payload)) {
        Serial.println("Data uploaded to cloud server");
    } else {
        Serial.println("Failed to upload data to cloud");
    }
    
    // Send to WebSocket clients
    hardware.sendWebSocketData(json_payload);
    
    last_data_upload_time_ms = current_time;
}

/**
 * @brief Update LED status based on system state
 */
void updateStatusIndicators() {
    LEDColor led_color = state_manager.getCurrentStateLEDColor();
    bool should_blink = (state_manager.getCurrentState() == SystemState::INFUSION_ERROR);
    
    hardware.setLEDStatus(led_color, should_blink);
}

/**
 * @brief Arduino setup function
 */
void setup() {
    // Initialize system
    if (!initializeSystem()) {
        // If initialization fails, enter error state and halt
        while (true) {
            hardware.setLEDStatus(LEDColor::RED, false);
            delay(1000);
        }
    }
    
    last_main_loop_time_ms = millis();
}

/**
 * @brief Arduino main loop function
 */
void loop() {
    unsigned long current_time = millis();
    
    // Main loop timing control
    if (current_time - last_main_loop_time_ms < TimingConfig::MAIN_LOOP_INTERVAL_MS) {
        return;
    }
    
    // Update system components
    state_manager.update(current_time);
    hardware.update(current_time);
    
    // Handle user inputs
    handleUserInputs();
    
    // Process sensor data (only if not in error state)
    if (state_manager.getCurrentState() != SystemState::INIT_ERROR) {
        processSensorData();
    }
    
    // Handle network communications
    hardware.handleHTTPRequests();
    hardware.handleWebSocketEvents();
    
    // Upload data to cloud
    if (hardware.isWiFiConnected()) {
        uploadDataToCloud();
    }
    
    // Update status indicators
    updateStatusIndicators();
    
    // Handle fast convergence mode transitions
    if (data_processor.isFastConvergenceModeActive() && 
        state_manager.shouldEndFastConvergence(current_time)) {
        
        data_processor.setFastConvergenceMode(false);
        state_manager.endFastConvergence();
        state_manager.transitionToState(SystemState::NORMAL);
        Serial.println("Fast convergence mode ended - entering normal operation");
    }
    
    last_main_loop_time_ms = current_time;
}