#include "../include/SystemStateManager.h"
#include "../include/Config.h"

SystemStateManager::SystemStateManager() 
    : current_state_(SystemState::INITIALIZING),
      state_start_time_ms_(0),
      auto_clamp_enabled_(false),
      fast_convergence_active_(false),
      fast_convergence_start_ms_(0),
      infusion_abnormality_detected_(false),
      last_drip_detected_time_ms_(0) {
}

void SystemStateManager::initialize() {
    transitionToState(SystemState::INITIALIZING);
    fast_convergence_active_ = false;
    infusion_abnormality_detected_ = false;
    auto_clamp_enabled_ = false;
}

void SystemStateManager::update(unsigned long current_time_ms) {
    // Check for drip timeout if in normal operation
    if (current_state_ == SystemState::NORMAL || current_state_ == SystemState::FAST_CONVERGENCE) {
        if (checkForDripTimeout(current_time_ms)) {
            setInfusionAbnormality(true);
            transitionToState(SystemState::INFUSION_ERROR);
        }
    }
    
    // Handle fast convergence mode transitions
    if (fast_convergence_active_ && shouldEndFastConvergence(current_time_ms)) {
        endFastConvergence();
        if (current_state_ == SystemState::FAST_CONVERGENCE) {
            transitionToState(SystemState::NORMAL);
        }
    }
}

void SystemStateManager::transitionToState(SystemState new_state) {
    if (current_state_ != new_state) {
        current_state_ = new_state;
        state_start_time_ms_ = millis();
        
        // Handle state-specific actions
        switch (new_state) {
            case SystemState::FAST_CONVERGENCE:
                startFastConvergence(millis());
                break;
            case SystemState::NORMAL:
                // Clear abnormality when transitioning to normal
                infusion_abnormality_detected_ = false;
                break;
            case SystemState::INFUSION_ERROR:
                // Enable auto clamp on error
                auto_clamp_enabled_ = true;
                break;
            case SystemState::COMPLETED:
                // Enable auto clamp on completion
                auto_clamp_enabled_ = true;
                break;
            default:
                break;
        }
    }
}

const char* SystemStateManager::getStateDisplayName() const {
    switch (current_state_) {
        case SystemState::INITIALIZING:    return "Initializing";
        case SystemState::INIT_ERROR:      return "Init Error";
        case SystemState::NORMAL:          return "Normal";
        case SystemState::INFUSION_ERROR:  return "Infusion Error";
        case SystemState::FAST_CONVERGENCE: return "Fast Convergence";
        case SystemState::COMPLETED:       return "Completed";
        default:                          return "Unknown";
    }
}

LEDColor SystemStateManager::getCurrentStateLEDColor() const {
    switch (current_state_) {
        case SystemState::INITIALIZING:    return LEDColor::YELLOW;
        case SystemState::INIT_ERROR:      return LEDColor::RED;
        case SystemState::NORMAL:          return LEDColor::GREEN;
        case SystemState::INFUSION_ERROR:  return LEDColor::RED; // Should blink
        case SystemState::FAST_CONVERGENCE: return LEDColor::BLUE;
        case SystemState::COMPLETED:       return LEDColor::WHITE;
        default:                          return LEDColor::OFF;
    }
}

bool SystemStateManager::shouldEnterFastConvergence() const {
    return current_state_ == SystemState::INITIALIZING;
}

void SystemStateManager::startFastConvergence(unsigned long current_time_ms) {
    fast_convergence_active_ = true;
    fast_convergence_start_ms_ = current_time_ms;
}

bool SystemStateManager::shouldEndFastConvergence(unsigned long current_time_ms) const {
    if (!fast_convergence_active_) return false;
    
    return (current_time_ms - fast_convergence_start_ms_) >= TimingConfig::FAST_CONVERGENCE_DURATION_MS;
}

void SystemStateManager::endFastConvergence() {
    fast_convergence_active_ = false;
    fast_convergence_start_ms_ = 0;
}

void SystemStateManager::setInfusionAbnormality(bool abnormal) {
    infusion_abnormality_detected_ = abnormal;
    
    if (abnormal) {
        auto_clamp_enabled_ = true;
    }
}

void SystemStateManager::updateLastDripTime(unsigned long time_ms) {
    last_drip_detected_time_ms_ = time_ms;
}

bool SystemStateManager::checkForDripTimeout(unsigned long current_time_ms) const {
    if (last_drip_detected_time_ms_ == 0) return false;
    
    return (current_time_ms - last_drip_detected_time_ms_) > TimingConfig::NO_DRIP_TIMEOUT_MS;
}

unsigned long SystemStateManager::getTimeInCurrentState(unsigned long current_time_ms) const {
    return current_time_ms - state_start_time_ms_;
}