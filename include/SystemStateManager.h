#ifndef SYSTEM_STATE_MANAGER_H
#define SYSTEM_STATE_MANAGER_H

#include <Arduino.h>

/**
 * @brief System state enumeration for infusion monitoring
 */
enum class SystemState {
    INITIALIZING,       // System is initializing
    INIT_ERROR,        // Initialization failed
    NORMAL,            // Normal infusion monitoring
    INFUSION_ERROR,    // Infusion abnormality detected
    FAST_CONVERGENCE,  // Fast convergence mode during startup
    COMPLETED          // Infusion completed
};

/**
 * @brief LED color configuration for system states
 */
enum class LEDColor {
    OFF,
    RED,
    GREEN, 
    BLUE,
    YELLOW,
    WHITE
};

/**
 * @brief System State Manager class
 * 
 * Manages the overall system state transitions and provides
 * a clean interface for state-dependent behavior.
 */
class SystemStateManager {
private:
    SystemState current_state_;
    unsigned long state_start_time_ms_;
    bool auto_clamp_enabled_;
    
    // Fast convergence mode tracking
    bool fast_convergence_active_;
    unsigned long fast_convergence_start_ms_;
    
    // Error tracking
    bool infusion_abnormality_detected_;
    unsigned long last_drip_detected_time_ms_;

public:
    /**
     * @brief Constructor
     */
    SystemStateManager();
    
    /**
     * @brief Initialize the state manager
     */
    void initialize();
    
    /**
     * @brief Update state manager (should be called regularly)
     * @param current_time_ms Current system time in milliseconds
     */
    void update(unsigned long current_time_ms);
    
    /**
     * @brief Transition to a new system state
     * @param new_state Target state to transition to
     */
    void transitionToState(SystemState new_state);
    
    /**
     * @brief Get current system state
     * @return Current SystemState
     */
    SystemState getCurrentState() const { return current_state_; }
    
    /**
     * @brief Get state display name as string
     * @return Human-readable state name
     */
    const char* getStateDisplayName() const;
    
    /**
     * @brief Get LED color for current state
     * @return LEDColor for current state
     */
    LEDColor getCurrentStateLEDColor() const;
    
    /**
     * @brief Check if system should enter fast convergence mode
     * @return True if fast convergence should be active
     */
    bool shouldEnterFastConvergence() const;
    
    /**
     * @brief Check if fast convergence mode is active
     * @return True if in fast convergence mode
     */
    bool isInFastConvergenceMode() const { return fast_convergence_active_; }
    
    /**
     * @brief Start fast convergence mode
     * @param current_time_ms Current system time
     */
    void startFastConvergence(unsigned long current_time_ms);
    
    /**
     * @brief Check if fast convergence should end
     * @param current_time_ms Current system time
     * @return True if fast convergence should end
     */
    bool shouldEndFastConvergence(unsigned long current_time_ms) const;
    
    /**
     * @brief End fast convergence mode
     */
    void endFastConvergence();
    
    /**
     * @brief Set infusion abnormality state
     * @param abnormal True if abnormality detected
     */
    void setInfusionAbnormality(bool abnormal);
    
    /**
     * @brief Check if infusion abnormality is active
     * @return True if abnormality detected
     */
    bool hasInfusionAbnormality() const { return infusion_abnormality_detected_; }
    
    /**
     * @brief Update last drip detection time
     * @param time_ms Time of last drip detection
     */
    void updateLastDripTime(unsigned long time_ms);
    
    /**
     * @brief Check for drip timeout abnormality
     * @param current_time_ms Current system time
     * @return True if drip timeout detected
     */
    bool checkForDripTimeout(unsigned long current_time_ms) const;
    
    /**
     * @brief Set auto clamp state
     * @param enabled True to enable auto clamp
     */
    void setAutoClamp(bool enabled) { auto_clamp_enabled_ = enabled; }
    
    /**
     * @brief Get auto clamp state
     * @return True if auto clamp is enabled
     */
    bool isAutoClampEnabled() const { return auto_clamp_enabled_; }
    
    /**
     * @brief Get time in current state
     * @param current_time_ms Current system time
     * @return Time spent in current state (milliseconds)
     */
    unsigned long getTimeInCurrentState(unsigned long current_time_ms) const;
};

#endif // SYSTEM_STATE_MANAGER_H