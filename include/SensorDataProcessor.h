#ifndef SENSOR_DATA_PROCESSOR_H
#define SENSOR_DATA_PROCESSOR_H

#include "WeightKalmanFilter.h"
#include "DripKalmanFilter.h"
#include "DataFusion.h"
#include "Config.h"

/**
 * @brief Sensor data processing result structure
 */
struct ProcessingResult {
    // Raw measurements
    float raw_weight_g;
    float raw_drip_rate_dps;
    
    // Filtered values
    float filtered_weight_g;
    float filtered_drip_rate_dps;
    
    // Flow rate estimates
    float weight_based_flow_gps;
    float drip_based_flow_gps;
    
    // Fused results
    float fused_flow_rate_gps;
    float fused_remaining_weight_g;
    
    // Remaining time estimates
    float remaining_time_seconds;
    
    // WPD calibration data
    float weight_per_drop_g;
    bool wpd_calibrating;
};

/**
 * @brief Sensor Data Processor class
 * 
 * Encapsulates all sensor data processing logic including Kalman filtering,
 * data fusion, and remaining time calculations. Provides a clean interface
 * for processing sensor data and obtaining filtered results.
 */
class SensorDataProcessor {
private:
    WeightKalmanFilter weight_filter_;
    DripKalmanFilter drip_filter_;
    DataFusion data_fusion_;
    
    // System parameters
    float initial_total_weight_g_;
    float target_empty_weight_g_;
    bool system_initialized_;
    
    // Fast convergence mode support
    bool fast_convergence_mode_;
    float original_filter_params_[8]; // Store original parameters
    
    // WPD calibration
    bool wpd_calibration_active_;
    unsigned long wpd_calibration_start_ms_;
    
    // Drip detection
    volatile unsigned long total_drops_;
    unsigned long last_drip_update_time_ms_;

public:
    /**
     * @brief Constructor
     */
    SensorDataProcessor();
    
    /**
     * @brief Initialize the data processor
     * @param initial_weight_g Initial total weight measurement
     */
    void initialize(float initial_weight_g);
    
    /**
     * @brief Process sensor measurements and return filtered results
     * @param weight_measurement Raw weight sensor reading (grams)
     * @param drip_count Number of new drips detected since last call
     * @param delta_time_seconds Time elapsed since last processing (seconds)
     * @return ProcessingResult containing all processed data
     */
    ProcessingResult processSensorData(float weight_measurement, 
                                     unsigned int drip_count,
                                     float delta_time_seconds);
    
    /**
     * @brief Set fast convergence mode
     * @param enabled True to enable fast convergence mode
     */
    void setFastConvergenceMode(bool enabled);
    
    /**
     * @brief Check if fast convergence mode is active
     * @return True if fast convergence mode is active
     */
    bool isFastConvergenceModeActive() const { return fast_convergence_mode_; }
    
    /**
     * @brief Start WPD (Weight Per Drop) calibration
     */
    void startWPDCalibration();
    
    /**
     * @brief Stop WPD calibration
     */
    void stopWPDCalibration();
    
    /**
     * @brief Check if WPD calibration is active
     * @return True if calibration is active
     */
    bool isWPDCalibrationActive() const { return wpd_calibration_active_; }
    
    /**
     * @brief Set target empty weight for completion detection
     * @param target_weight_g Target empty weight in grams
     */
    void setTargetEmptyWeight(float target_weight_g) { target_empty_weight_g_ = target_weight_g; }
    
    /**
     * @brief Get initial total weight
     * @return Initial total weight in grams
     */
    float getInitialTotalWeight() const { return initial_total_weight_g_; }
    
    /**
     * @brief Get total number of drops detected
     * @return Total drop count
     */
    unsigned long getTotalDropCount() const { return total_drops_; }
    
    /**
     * @brief Update drop count (called from ISR or main loop)
     * @param new_drops Number of new drops to add
     */
    void updateDropCount(unsigned int new_drops);
    
    /**
     * @brief Calculate infusion progress percentage
     * @param current_weight_g Current weight
     * @return Progress as percentage (0.0 to 100.0)
     */
    float calculateInfusionProgress(float current_weight_g) const;
    
    /**
     * @brief Check if infusion is completed
     * @param current_weight_g Current weight
     * @return True if infusion is completed
     */
    bool isInfusionCompleted(float current_weight_g) const;

private:
    /**
     * @brief Calculate remaining time based on current flow rate
     * @param current_weight_g Current remaining weight
     * @param flow_rate_gps Current flow rate in grams per second
     * @return Remaining time in seconds (-1 if cannot calculate)
     */
    float calculateRemainingTime(float current_weight_g, float flow_rate_gps) const;
    
    /**
     * @brief Store original filter parameters for fast convergence mode
     */
    void storeOriginalFilterParameters();
    
    /**
     * @brief Restore original filter parameters after fast convergence
     */
    void restoreOriginalFilterParameters();
    
    /**
     * @brief Apply fast convergence parameters to filters
     */
    void applyFastConvergenceParameters();
};

#endif // SENSOR_DATA_PROCESSOR_H