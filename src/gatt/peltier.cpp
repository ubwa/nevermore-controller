#include "config.hpp"
#include "handler_helpers.hpp"
#include "sdk/ble_data_types.hpp"
#include "sdk/btstack.hpp"
#include "sdk/pwm.hpp"
#include "sensors.hpp"
#include "sensors/async_sensor.hpp"
#include "settings.hpp"
#include "utility/humidity.hpp"
#include "utility/timer.hpp"
#include <chrono>
#include <cmath>
#include <limits>

using namespace std;

namespace nevermore::gatt::peltier {

namespace {

struct PeltierMapper final : sensors::SensorPeriodic {
    [[nodiscard]] char const* name() const override {
        return "Peltier ADC Mapper";
    }

    void read() override {
        auto lock = sensors::sensors_guard();
        sensors::p_sensors.temperature_cold = sensors::g_adc_thermistor_temps[0];
        sensors::p_sensors.temperature_hot = sensors::g_adc_thermistor_temps[1];
    }
};

// Global mapper instance (will be initialized in init() if ADC sensors available)
static PeltierMapper* g_peltier_mapper = nullptr;

constexpr uint8_t PELTIER_CONTROL_UPDATE_HZ = 10;

struct PeltierSettings {
    uint8_t control_mode;  // 0=PID, 1=Watermark
    float watermark_high;
    float watermark_low;
    float min_temp_cold;
    float max_temp_cold;
    float min_temp_hot;
    float max_temp_hot;
    float max_deviation;
    float enable_delay;
    float cycle_time;
    float max_pwm;
    float kp;
    float ki;
    float kd;
    float smooth_time;
    // Safety parameters
    float dew_point_safety;  // Dew point offset for condensation prevention
    float hot_side_safety;   // Hot side thermal limit
    float dew_point_base;    // Base dew point calculation (reserved)
    float dew_point_range;   // Dew point range (reserved)
};

PeltierSettings p_settings;

enum class PeltierError {
    NONE = 0,
    COLD_TEMP_TOO_LOW,
    COLD_TEMP_TOO_HIGH,
    HOT_TEMP_TOO_LOW,
    HOT_TEMP_TOO_HIGH,
    TEMP_DEVIATION_EXCEEDED,
    SENSOR_READ_FAILURE,
    PWM_INIT_FAILURE,
    SENSOR_UNSTABLE  // Temperature readings fluctuating excessively
};

struct PeltierControl {
    using Clock = std::chrono::steady_clock;

    void update(sensors::PeltierSensors const& sensors = sensors::p_sensors,
            PeltierSettings const& peltier_settings = p_settings);

    void target(BLE::Temperature target) {
        _target = target;
    }

    bool enable(uint8_t enable) {
        if (enable == 0) {
            _enable = 0;
            _last_enable_time = Clock::now();
            return true;
        }

        if (permanently_disabled) {
            printf("Cannot enable Peltier: permanently disabled due to unstable sensors (requires "
                   "restart)\n");
            return false;
        }

        // If enabling, check enable_delay cooldown period
        auto now = Clock::now();
        auto elapsed = std::chrono::duration<double>(now - _last_enable_time).count();

        if (_last_enable_time != Clock::time_point::min() && elapsed < double(p_settings.enable_delay)) {
            double remaining = double(p_settings.enable_delay) - elapsed;
            printf("Cannot enable Peltier: cooldown period not elapsed (%.1fs elapsed, %.1fs required, %.1fs "
                   "remaining)\n",
                    elapsed, double(p_settings.enable_delay), remaining);
            return false;
        }

        // Check that pins 26 and 27 are configured
        bool has_pin_26 = Pins::active().adc_thermistor[0] && uint8_t(Pins::active().adc_thermistor[0]) == 26;
        bool has_pin_27 = Pins::active().adc_thermistor[1] && uint8_t(Pins::active().adc_thermistor[1]) == 27;

        if (!has_pin_26 || !has_pin_27) {
            printf("Cannot enable Peltier: pins 26 and 27 must be configured (pin26=%d, pin27=%d)\n",
                    has_pin_26, has_pin_27);
            return false;
        }

        // Check that intake environmental sensors are available and valid
        auto sensor_lock = sensors::sensors_guard();
        bool has_intake_temp = sensors::g_sensors.temperature_intake != BLE::NOT_KNOWN &&
                               std::isfinite(double(sensors::g_sensors.temperature_intake));
        bool has_intake_humidity = sensors::g_sensors.humidity_intake != BLE::NOT_KNOWN &&
                                   std::isfinite(double(sensors::g_sensors.humidity_intake));

        if (!has_intake_temp || !has_intake_humidity) {
            printf("Cannot enable Peltier: intake sensors required for dew point calculation "
                   "(temp=%s, humidity=%s)\n",
                    has_intake_temp ? "OK" : "MISSING", has_intake_humidity ? "OK" : "MISSING");
            return false;
        }

        // Check that Peltier sensor readings are valid (not NaN)
        if (!std::isfinite(double(sensors::p_sensors.temperature_cold)) ||
                !std::isfinite(double(sensors::p_sensors.temperature_hot))) {
            printf("Cannot enable Peltier: Peltier sensor readings invalid (cold=%.2f, hot=%.2f)\n",
                    double(sensors::p_sensors.temperature_cold), double(sensors::p_sensors.temperature_hot));
            return false;
        }

        _enable = enable;
        _last_enable_time = Clock::now();
        return true;
    }

    [[nodiscard]] BLE::Percentage8 power() const {
        return _power;
    }

    [[nodiscard]] BLE::Temperature target_temp() const {
        return _target;
    }

    [[nodiscard]] uint8_t enabled() const {
        return _enable;
    }

    [[nodiscard]] bool has_error() const {
        return _error_state;
    }

    [[nodiscard]] PeltierError error_type() const {
        return _error_type;
    }

    void clear_error_state();

    // Restore enable state from persistent storage without validation
    // This is used at startup before sensors are ready
    void restore_enable_state(uint8_t enable) {
        _enable = enable;
    }

private:
    void update_pid(sensors::PeltierSensors const& sensors, PeltierSettings const& settings);
    void update_watermark(sensors::PeltierSensors const& sensors, PeltierSettings const& settings);

    [[nodiscard]] bool check_safety_limits(
            sensors::PeltierSensors const& sensors, PeltierSettings const& settings);

    void set_error_state(PeltierError error_type);

    Clock::time_point prev_temp_time = Clock::time_point::min();
    double prev_error = 0;
    double prev_der = 0;
    double int_sum = 0;
    double prev_temp = 0;
    bool watermark_state = false;

    // Temperature stability tracking
    static constexpr int STABILITY_HISTORY_SIZE = 5;
    double cold_temp_history[STABILITY_HISTORY_SIZE] = {0};
    double hot_temp_history[STABILITY_HISTORY_SIZE] = {0};
    int stability_index = 0;
    int stability_sample_count = 0;

    // Three-strikes lockout for unstable sensors
    static constexpr int MAX_INSTABILITY_STRIKES = 3;
    int instability_strike_count = 0;
    bool permanently_disabled = false;  // Set to true after 3 strikes

    BLE::Percentage8 _power = 0;
    BLE::Temperature _target = 0;
    uint8_t _enable = 0;
    bool _error_state = false;
    PeltierError _error_type = PeltierError::NONE;
    Clock::time_point _last_enable_time = Clock::time_point::min();
};

PeltierControl g_peltier;

// PID Auto-tune using Ziegler-Nichols relay method (similar to Klipper)
struct PIDAutoTune {
    using Clock = std::chrono::steady_clock;

    static constexpr double TUNE_DELTA = 2.0;  // Hysteresis around target (°C)
    static constexpr int MIN_PEAKS = 12;       // Minimum peaks before calculating PID
    static constexpr int MAX_PEAKS = 20;       // Maximum peaks to collect

    bool active = false;
    double calibrate_temp = 0;
    double max_power = 100.0;

    // State
    bool cooling = false;  // true = Peltier ON (cooling), false = OFF
    double peak = 0;
    Clock::time_point peak_time = Clock::time_point::min();

    // Peak recording: pairs of (temperature, time)
    struct Peak {
        double temp;
        Clock::time_point time;
    };
    static constexpr int MAX_PEAK_STORAGE = 24;
    Peak peaks[MAX_PEAK_STORAGE];
    int peak_count = 0;

    // Results
    double result_kp = 0;
    double result_ki = 0;
    double result_kd = 0;
    bool has_result = false;

    void start(double target_temp, double max_pwm) {
        active = true;
        calibrate_temp = target_temp;
        max_power = max_pwm;
        cooling = false;
        peak = -9999999.0;
        peak_time = Clock::now();
        peak_count = 0;
        has_result = false;
        result_kp = result_ki = result_kd = 0;

        printf("[PID AutoTune] Started: target=%.1f°C, max_power=%.1f%%\n", calibrate_temp, max_power);
    }

    void stop() {
        if (active) {
            printf("[PID AutoTune] Stopped after %d peaks\n", peak_count);
        }
        active = false;
    }

    // Returns PWM power (0-100) or -1 if not active
    double update(double current_temp) {
        if (!active) return -1;

        auto now = Clock::now();
        double target = calibrate_temp;

        // Check if temperature crossed threshold - toggle cooling
        if (cooling && current_temp <= target - TUNE_DELTA) {
            // Was cooling, now below target - turn OFF
            cooling = false;
            check_peaks(now);
            printf("[PID AutoTune] Crossed below target: temp=%.2f°C, turning OFF (peak %d)\n", current_temp,
                    peak_count);
        } else if (!cooling && current_temp >= target + TUNE_DELTA) {
            // Was off, now above target - turn ON (cool)
            cooling = true;
            check_peaks(now);
            printf("[PID AutoTune] Crossed above target: temp=%.2f°C, turning ON (peak %d)\n", current_temp,
                    peak_count);
        }

        // Track peaks
        if (cooling) {
            // Cooling - looking for temperature minimum
            if (current_temp < peak) {
                peak = current_temp;
                peak_time = now;
            }
        } else {
            // Not cooling - looking for temperature maximum
            if (current_temp > peak) {
                peak = current_temp;
                peak_time = now;
            }
        }

        // Check if we have enough data
        if (peak_count >= MIN_PEAKS) {
            calc_final_pid();
            active = false;
            printf("[PID AutoTune] Complete! Kp=%.3f Ki=%.3f Kd=%.3f\n", result_kp, result_ki, result_kd);
        }

        if (peak_count >= MAX_PEAKS) {
            active = false;
            printf("[PID AutoTune] Max peaks reached, stopping\n");
        }

        return cooling ? max_power : 0.0;
    }

    void check_peaks(Clock::time_point now) {
        if (peak_count < MAX_PEAK_STORAGE) {
            peaks[peak_count].temp = peak;
            peaks[peak_count].time = peak_time;
            peak_count++;
        }

        // Reset peak tracking for next phase
        if (cooling) {
            peak = 9999999.0;  // Looking for minimum
        } else {
            peak = -9999999.0;  // Looking for maximum
        }

        // Calculate intermediate PID if we have enough peaks
        if (peak_count >= 4) {
            calc_pid(peak_count - 1);
        }
    }

    void calc_pid(int pos) {
        if (pos < 2) return;

        // Temperature difference between last two peaks (amplitude)
        double temp_diff = peaks[pos].temp - peaks[pos - 1].temp;

        // Time for one full cycle (period) - use peaks 2 apart
        double time_diff = 0;
        if (pos >= 2) {
            auto duration = peaks[pos].time - peaks[pos - 2].time;
            time_diff = std::chrono::duration<double>(duration).count();
        }

        if (time_diff <= 0) return;

        // Åström-Hägglund method to estimate ultimate gain (Ku) and period (Tu)
        double amplitude = 0.5 * std::abs(temp_diff);
        if (amplitude < 0.1) amplitude = 0.1;  // Prevent division by zero

        double Ku = 4.0 * max_power / (3.14159 * amplitude);
        double Tu = time_diff;

        // Ziegler-Nichols PID tuning formulas
        double Ti = 0.5 * Tu;
        double Td = 0.125 * Tu;

        result_kp = 0.6 * Ku;
        result_ki = result_kp / Ti;
        result_kd = result_kp * Td;

        printf("[PID AutoTune] Peak %d: amplitude=%.2f°C, period=%.2fs, Ku=%.2f, Tu=%.2f\n", pos, amplitude,
                Tu, Ku, Tu);
        printf("[PID AutoTune] Intermediate: Kp=%.3f Ki=%.3f Kd=%.3f\n", result_kp, result_ki, result_kd);
    }

    void calc_final_pid() {
        if (peak_count < 4) return;

        // Use median cycle time for more stable results
        // Find the middle cycle
        int mid_pos = peak_count / 2;
        if (mid_pos < 2) mid_pos = 2;
        if (mid_pos >= peak_count) mid_pos = peak_count - 1;

        calc_pid(mid_pos);
        has_result = true;
    }

    bool is_complete() const {
        return has_result && !active;
    }
};

PIDAutoTune g_autotune;

struct [[gnu::packed]] PeltierTempAggregate {
    BLE::Temperature temperature_cold = sensors::p_sensors.temperature_cold;
    BLE::Temperature temperature_hot = sensors::p_sensors.temperature_hot;
};

struct [[gnu::packed]] PeltierPowerAggregate {
    BLE::Percentage8 power = g_peltier.power();
};

auto g_notify_peltier_temp = NotifyState<[](hci_con_handle_t conn) {
    att_server_notify(
            conn, HANDLE_ATTR(PELTIER_SENSOR_COLD_TEMP, VALUE), sensors::p_sensors.temperature_cold);
}>();

auto g_notify_peltier_power = NotifyState<[](hci_con_handle_t conn) {
    att_server_notify(conn, HANDLE_ATTR(PELTIER_POWER, VALUE), g_peltier.power());
}>();

auto g_notify_peltier_error = NotifyState<[](hci_con_handle_t conn) {
    uint8_t error_code = static_cast<uint8_t>(g_peltier.error_type());
    att_server_notify(conn, HANDLE_ATTR(PELTIER_ERROR_STATUS, VALUE), error_code);
}>();

// Load settings from persistent storage into p_settings (without control state)
void load_settings_only() {
    auto const& config = settings::g_active.peltier;
    p_settings.min_temp_cold = config.min_temp_cold;
    p_settings.max_temp_cold = config.max_temp_cold;
    p_settings.min_temp_hot = config.min_temp_hot;
    p_settings.max_temp_hot = config.max_temp_hot;
    p_settings.max_deviation = config.max_deviation;
    p_settings.enable_delay = config.enable_delay;
    p_settings.cycle_time = config.cycle_time;
    p_settings.max_pwm = config.max_pwm;
    p_settings.kp = config.kp;
    p_settings.ki = config.ki;
    p_settings.kd = config.kd;
    p_settings.smooth_time = config.smooth_time;
    p_settings.dew_point_safety = config.dew_point_safety;
    p_settings.hot_side_safety = config.hot_side_safety;
    p_settings.dew_point_base = config.dew_point_base;
    p_settings.dew_point_range = config.dew_point_range;
    p_settings.control_mode = config.control_mode;
    p_settings.watermark_high = config.watermark_high;
    p_settings.watermark_low = config.watermark_low;
}

// Load control state (target and enable) from persistent storage
// This should be called AFTER sensors are initialized
void load_control_state() {
    auto const& config = settings::g_active.peltier;
    g_peltier.target(config.target_temp);

    // Only restore enable state if it was enabled AND sensors are providing valid readings
    if (config.enabled) {
        // Wait a moment for sensors to provide initial readings
        // The mapper runs periodically, but readings might not be available yet
        auto sensor_lock = sensors::sensors_guard();
        bool sensors_valid = std::isfinite(double(sensors::p_sensors.temperature_cold)) &&
                             std::isfinite(double(sensors::p_sensors.temperature_hot));

        if (sensors_valid) {
            // Sensors are ready, attempt to enable with full validation
            if (!g_peltier.enable(config.enabled)) {
                printf("Peltier enable failed validation at startup - remaining disabled\n");
            }
        } else {
            // Sensors not ready yet - don't restore enable state
            printf("Peltier: Sensor readings not available yet at startup - enable state not restored\n");
        }
    }
}

// Save settings to persistent storage
void save_settings() {
    auto& config = settings::g_active.peltier;
    config.min_temp_cold = p_settings.min_temp_cold;
    config.max_temp_cold = p_settings.max_temp_cold;
    config.min_temp_hot = p_settings.min_temp_hot;
    config.max_temp_hot = p_settings.max_temp_hot;
    config.max_deviation = p_settings.max_deviation;
    config.enable_delay = p_settings.enable_delay;
    config.cycle_time = p_settings.cycle_time;
    config.max_pwm = p_settings.max_pwm;
    config.kp = p_settings.kp;
    config.ki = p_settings.ki;
    config.kd = p_settings.kd;
    config.smooth_time = p_settings.smooth_time;
    config.dew_point_safety = p_settings.dew_point_safety;
    config.hot_side_safety = p_settings.hot_side_safety;
    config.dew_point_base = p_settings.dew_point_base;
    config.dew_point_range = p_settings.dew_point_range;
    config.control_mode = p_settings.control_mode;
    config.watermark_high = p_settings.watermark_high;
    config.watermark_low = p_settings.watermark_low;
    config.target_temp = g_peltier.target_temp();
    config.enabled = g_peltier.enabled();

    // Trigger settings persistence
    settings::save(settings::g_active);
}

bool PeltierControl::check_safety_limits(
        sensors::PeltierSensors const& sensors, PeltierSettings const& settings) {
    // Check for invalid sensor readings (NaN or infinite values)
    if (!std::isfinite(double(sensors.temperature_cold)) || !std::isfinite(double(sensors.temperature_hot))) {
        set_error_state(PeltierError::SENSOR_READ_FAILURE);
        return false;
    }

    // Update temperature history for stability checking
    cold_temp_history[stability_index] = double(sensors.temperature_cold);
    hot_temp_history[stability_index] = double(sensors.temperature_hot);
    stability_index = (stability_index + 1) % STABILITY_HISTORY_SIZE;
    if (stability_sample_count < STABILITY_HISTORY_SIZE) {
        stability_sample_count++;
    }

    // Check temperature stability (only after we have enough samples)
    if (stability_sample_count >= STABILITY_HISTORY_SIZE) {
        // Calculate standard deviation for cold side
        double cold_sum = 0, cold_sq_sum = 0;
        double hot_sum = 0, hot_sq_sum = 0;
        for (int i = 0; i < STABILITY_HISTORY_SIZE; i++) {
            cold_sum += cold_temp_history[i];
            cold_sq_sum += cold_temp_history[i] * cold_temp_history[i];
            hot_sum += hot_temp_history[i];
            hot_sq_sum += hot_temp_history[i] * hot_temp_history[i];
        }
        double cold_mean = cold_sum / STABILITY_HISTORY_SIZE;
        double hot_mean = hot_sum / STABILITY_HISTORY_SIZE;
        double cold_variance = (cold_sq_sum / STABILITY_HISTORY_SIZE) - (cold_mean * cold_mean);
        double hot_variance = (hot_sq_sum / STABILITY_HISTORY_SIZE) - (hot_mean * hot_mean);
        double cold_std_dev = sqrt(cold_variance);
        double hot_std_dev = sqrt(hot_variance);

        // Also calculate max range (difference between min and max)
        double cold_min = cold_temp_history[0], cold_max = cold_temp_history[0];
        double hot_min = hot_temp_history[0], hot_max = hot_temp_history[0];
        for (int i = 1; i < STABILITY_HISTORY_SIZE; i++) {
            if (cold_temp_history[i] < cold_min) cold_min = cold_temp_history[i];
            if (cold_temp_history[i] > cold_max) cold_max = cold_temp_history[i];
            if (hot_temp_history[i] < hot_min) hot_min = hot_temp_history[i];
            if (hot_temp_history[i] > hot_max) hot_max = hot_temp_history[i];
        }
        double cold_range = cold_max - cold_min;
        double hot_range = hot_max - hot_min;

        // Threshold for stability: std dev > 3°C OR range > 8°C indicates unstable sensor
        // These thresholds are conservative to catch clearly faulty sensors
        // Added ibn after 3 faulty thermisistors observed in testing. Better safe that sorry :)
        // May want to remove or simplify, i was being extra cautios and paranoid here.
        constexpr double MAX_STD_DEV = 3.0;  // Max standard deviation in °C
        constexpr double MAX_RANGE = 8.0;    // Max range (max-min) in °C

        if (cold_std_dev > MAX_STD_DEV || cold_range > MAX_RANGE) {
            instability_strike_count++;
            printf("Peltier safety: Cold sensor unstable (std_dev=%.2f°C, range=%.2f°C, readings: %.1f %.1f "
                   "%.1f %.1f %.1f) [Strike %d/%d]\n",
                    cold_std_dev, cold_range, cold_temp_history[0], cold_temp_history[1],
                    cold_temp_history[2], cold_temp_history[3], cold_temp_history[4],
                    instability_strike_count, MAX_INSTABILITY_STRIKES);

            if (instability_strike_count >= MAX_INSTABILITY_STRIKES) {
                permanently_disabled = true;
                printf("!!! PELTIER PERMANENTLY DISABLED: 3 instability strikes detected - restart required "
                       "!!!\n");
            }

            set_error_state(PeltierError::SENSOR_UNSTABLE);
            return false;
        }

        if (hot_std_dev > MAX_STD_DEV || hot_range > MAX_RANGE) {
            instability_strike_count++;
            printf("Peltier safety: Hot sensor unstable (std_dev=%.2f°C, range=%.2f°C, readings: %.1f %.1f "
                   "%.1f %.1f %.1f) [Strike %d/%d]\n",
                    hot_std_dev, hot_range, hot_temp_history[0], hot_temp_history[1], hot_temp_history[2],
                    hot_temp_history[3], hot_temp_history[4], instability_strike_count,
                    MAX_INSTABILITY_STRIKES);

            if (instability_strike_count >= MAX_INSTABILITY_STRIKES) {
                permanently_disabled = true;
                printf("!!! PELTIER PERMANENTLY DISABLED: 3 instability strikes detected - restart required "
                       "!!!\n");
            }

            set_error_state(PeltierError::SENSOR_UNSTABLE);
            return false;
        }

        // If we've passed stability checks, gradually clear strikes
        // Only clear strikes if we're not in error state and readings are stable
        if (!_error_state && instability_strike_count > 0) {
            // Clear one strike every 100 successful checks (~10 seconds at 10Hz)
            static int stable_check_count = 0;
            stable_check_count++;
            if (stable_check_count >= 100) {
                instability_strike_count--;
                printf("Peltier stability: Sensors stable, clearing one strike (%d strikes remaining)\n",
                        instability_strike_count);
                stable_check_count = 0;
            }
        }
    }

    if (sensors.temperature_cold < settings.min_temp_cold) {
        set_error_state(PeltierError::COLD_TEMP_TOO_LOW);
        return false;
    }

    if (sensors.temperature_cold > settings.max_temp_cold) {
        set_error_state(PeltierError::COLD_TEMP_TOO_HIGH);
        return false;
    }

    if (sensors.temperature_hot < settings.min_temp_hot) {
        set_error_state(PeltierError::HOT_TEMP_TOO_LOW);
        return false;
    }

    if (sensors.temperature_hot > settings.max_temp_hot) {
        set_error_state(PeltierError::HOT_TEMP_TOO_HIGH);
        return false;
    }

    float deviation = abs(double(sensors.temperature_hot) - double(sensors.temperature_cold));
    if (deviation > settings.max_deviation) {
        set_error_state(PeltierError::TEMP_DEVIATION_EXCEEDED);
        return false;
    }

    return true;
}

void PeltierControl::set_error_state(PeltierError error_type) {
    // Only log if this is a new error (not already in error state)
    bool is_new_error = !_error_state || (_error_type != error_type);

    _error_state = true;
    _error_type = error_type;
    _power = 0;  // Immediately shut down PWM output

    if (is_new_error) {
        auto now = Clock::now();
        auto timestamp =
                std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

        const char* error_msg = "Unknown error";
        switch (error_type) {
        case PeltierError::COLD_TEMP_TOO_LOW: error_msg = "Cold side temperature too low"; break;
        case PeltierError::COLD_TEMP_TOO_HIGH: error_msg = "Cold side temperature too high"; break;
        case PeltierError::HOT_TEMP_TOO_LOW: error_msg = "Hot side temperature too low"; break;
        case PeltierError::HOT_TEMP_TOO_HIGH: error_msg = "Hot side temperature too high"; break;
        case PeltierError::TEMP_DEVIATION_EXCEEDED: error_msg = "Temperature deviation exceeded"; break;
        case PeltierError::SENSOR_READ_FAILURE: error_msg = "Sensor read failure"; break;
        case PeltierError::PWM_INIT_FAILURE: error_msg = "PWM initialization failure"; break;
        case PeltierError::SENSOR_UNSTABLE: error_msg = "Temperature sensor readings unstable"; break;
        default: break;
        }

        printf("[%lld] Peltier error: %s (cold=%.2f°C, hot=%.2f°C)\n", timestamp, error_msg,
                double(sensors::p_sensors.temperature_cold), double(sensors::p_sensors.temperature_hot));

        g_notify_peltier_error.notify();
    }
}

void PeltierControl::clear_error_state() {
    if (_error_state) {
        auto now = Clock::now();
        auto timestamp =
                std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

        printf("[%lld] Peltier error cleared (cold=%.2f°C, hot=%.2f°C)\n", timestamp,
                double(sensors::p_sensors.temperature_cold), double(sensors::p_sensors.temperature_hot));

        _error_state = false;
        _error_type = PeltierError::NONE;

        g_notify_peltier_error.notify();
    } else {
        _error_state = false;
        _error_type = PeltierError::NONE;
    }
}

void PeltierControl::update_pid(sensors::PeltierSensors const& sensors, PeltierSettings const& settings) {
    // If not enabled, set power to 0 and reset PID state
    if (!_enable) {
        _power = 0;
        int_sum = 0;
        prev_error = 0;
        prev_der = 0;
        prev_temp_time = Clock::time_point::min();
        return;
    }

    // Initialize timing on first run
    auto now = Clock::now();
    if (prev_temp_time == Clock::time_point::min()) {
        prev_temp_time = now;
        auto sensor_lock = sensors::sensors_guard();
        prev_temp = double(sensors::g_sensors.temperature_intake);
        prev_error = 0;
        prev_der = 0;
        int_sum = 0;
        _power = 0;
        return;
    }

    // Calculate time delta (dt) since last update
    auto dt_duration = now - prev_temp_time;
    double dt = std::chrono::duration<double>(dt_duration).count();

    // Avoid division by zero
    if (dt <= 0) {
        return;
    }

    double effective_target = double(_target);

    // Apply dew point condensation prevention if enabled (dew_point_safety > 0)
    if (settings.dew_point_safety > 0) {
        auto sensor_lock = sensors::sensors_guard();

        // Check if humidity sensor is available
        if (sensors::g_sensors.humidity_intake != BLE::NOT_KNOWN &&
                sensors::g_sensors.temperature_intake != BLE::NOT_KNOWN) {
            // Calculate dew point from environmental sensors
            double calculated_dew_point = humidity::dew_point(double(sensors::g_sensors.humidity_intake),
                    double(sensors::g_sensors.temperature_intake));

            // Apply safety margin
            double min_safe_temp = calculated_dew_point + settings.dew_point_safety;

            // Clamp target temperature to prevent condensation
            if (effective_target < min_safe_temp) {
                static double last_logged_dew_point = 0;
                // Log when dew point protection is active and value changes significantly
                if (abs(calculated_dew_point - last_logged_dew_point) > 1.0) {
                    printf("[Peltier] Dew point protection active: target %.1f°C clamped to %.1f°C "
                           "(dew_point=%.1f°C, safety=%.1f°C)\n",
                            effective_target, min_safe_temp, calculated_dew_point, settings.dew_point_safety);
                    last_logged_dew_point = calculated_dew_point;
                }
                effective_target = min_safe_temp;
            }
        }
    }

    // Calculate error and proportional term using effective target
    // Use intake temperature as the control variable
    double intake_temp;
    {
        auto sensor_lock = sensors::sensors_guard();
        intake_temp = double(sensors::g_sensors.temperature_intake);
    }

    // Only cool when intake is above target - nothing to do if already below
    if (intake_temp <= effective_target) {
        _power = 0;
        // Reset integral to prevent windup when not cooling
        int_sum = 0;
        prev_error = 0;
        prev_temp = intake_temp;
        prev_temp_time = now;
        return;
    }

    // Error is positive when intake is above target (need to cool)
    double error = intake_temp - effective_target;
    double proportional = settings.kp * error;

    // Subtask 5.2: Implement integral term with anti-windup    // Accumulate integral using trapezoidal rule
    double integral_increment = (prev_error + error) / 2.0 * dt;
    int_sum += integral_increment;

    // Clamp integral sum to prevent windup
    // Limit integral contribution to reasonable range based on max_pwm
    double max_integral = settings.max_pwm / (settings.ki + 1e-6);  // Avoid division by zero
    int_sum = std::max(-max_integral, std::min(max_integral, int_sum));

    double integral = settings.ki * int_sum;

    // Calculate raw derivative (positive when temperature is rising = need more cooling)
    double raw_derivative = (intake_temp - prev_temp) / dt;

    // Calculate smoothing factor n = max(1, smooth_time / dt)
    double n = std::max(1.0, settings.smooth_time / dt);

    // Apply exponential smoothing
    double derivative_smoothed = (raw_derivative + (n - 1.0) * prev_der) / n;

    double derivative = settings.kd * derivative_smoothed;

    double output = proportional + integral + derivative;

    // Clamp output to range [0, max_pwm]
    // Higher output = more cooling power needed
    double pwm = std::max(0.0, std::min(double(settings.max_pwm), output));

    // Apply hot side safety limit (soft limit before hard error)
    if (settings.hot_side_safety > 0) {
        double hot_side_limit = settings.max_temp_hot - settings.hot_side_safety;
        static bool hot_side_warning_logged = false;

        if (double(sensors.temperature_hot) >= hot_side_limit) {
            if (!hot_side_warning_logged) {
                printf("[Peltier] Hot side safety limit reached: temp=%.1f°C >= limit=%.1f°C, "
                       "reducing power to zero\n",
                        double(sensors.temperature_hot), hot_side_limit);
                hot_side_warning_logged = true;
            }
            pwm = 0.0;
            // Reset PID state when safety triggered to prevent windup
            int_sum = 0;
            prev_error = 0;
        } else {
            hot_side_warning_logged = false;  // Reset warning when temp drops below limit
        }
    }

    _power = static_cast<BLE::Percentage8>(pwm * 100.0 / settings.max_pwm);

    prev_error = error;
    prev_der = derivative_smoothed;
    prev_temp = intake_temp;
    prev_temp_time = now;
}

void PeltierControl::update_watermark(
        sensors::PeltierSensors const& sensors, PeltierSettings const& settings) {
    // If not enabled, set power to 0 and reset watermark state
    if (!_enable) {
        _power = 0;
        watermark_state = false;
        return;
    }

    // Calculate effective target with dew point prevention
    double effective_target = double(_target);

    // Apply dew point condensation prevention if enabled (dew_point_safety > 0)
    if (settings.dew_point_safety > 0) {
        auto sensor_lock = sensors::sensors_guard();

        // Check if humidity sensor is available
        if (sensors::g_sensors.humidity_intake != BLE::NOT_KNOWN &&
                sensors::g_sensors.temperature_intake != BLE::NOT_KNOWN) {
            // Calculate dew point from environmental sensors
            double calculated_dew_point = humidity::dew_point(double(sensors::g_sensors.humidity_intake),
                    double(sensors::g_sensors.temperature_intake));

            // Apply safety margin
            double min_safe_temp = calculated_dew_point + settings.dew_point_safety;

            // Clamp target temperature to prevent condensation
            if (effective_target < min_safe_temp) {
                static double last_logged_dew_point = 0;
                // Log when dew point protection is active and value changes significantly
                if (abs(calculated_dew_point - last_logged_dew_point) > 1.0) {
                    printf("[Peltier Watermark] Dew point protection active: target %.1f°C clamped to %.1f°C "
                           "(dew_point=%.1f°C, safety=%.1f°C)\n",
                            effective_target, min_safe_temp, calculated_dew_point, settings.dew_point_safety);
                    last_logged_dew_point = calculated_dew_point;
                }
                effective_target = min_safe_temp;
            }
        }
    }

    // Calculate upper and lower thresholds
    double upper_threshold = effective_target + settings.watermark_high;
    double lower_threshold = effective_target - settings.watermark_low;
    // Use intake temperature as the control variable
    double current_temp;
    {
        auto sensor_lock = sensors::sensors_guard();
        current_temp = double(sensors::g_sensors.temperature_intake);
    }

    // Bang-bang control with hysteresis
    if (watermark_state) {
        // Currently ON (cooling) - turn OFF if below lower threshold
        if (current_temp <= lower_threshold) {
            watermark_state = false;
        }
    } else {
        // Currently OFF - turn ON if above upper threshold
        if (current_temp >= upper_threshold) {
            watermark_state = true;
        }
    }

    // Set PWM based on state
    double pwm = watermark_state ? settings.max_pwm : 0.0;

    // Apply hot side safety limit (soft limit before hard error)
    if (settings.hot_side_safety > 0) {
        double hot_side_limit = settings.max_temp_hot - settings.hot_side_safety;
        static bool hot_side_warning_logged = false;

        if (double(sensors.temperature_hot) >= hot_side_limit) {
            if (!hot_side_warning_logged) {
                printf("[Peltier Watermark] Hot side safety limit reached: temp=%.1f°C >= limit=%.1f°C, "
                       "reducing power to zero\n",
                        double(sensors.temperature_hot), hot_side_limit);
                hot_side_warning_logged = true;
            }
            pwm = 0.0;
            watermark_state = false;  // Reset state when safety triggered
        } else {
            hot_side_warning_logged = false;  // Reset warning when temp drops below limit
        }
    }

    // Convert to percentage
    _power = static_cast<BLE::Percentage8>(pwm * 100.0 / settings.max_pwm);
}

void PeltierControl::update(sensors::PeltierSensors const& sensors, PeltierSettings const& peltier_settings) {
    static Clock::time_point last_update_time = Clock::time_point::min();
    static uint32_t update_count = 0;
    static uint32_t timing_report_interval = 100;  // Report every 100 updates (10 seconds at 10 Hz)

    auto now = Clock::now();
    if (last_update_time != Clock::time_point::min()) {
        auto dt_duration = now - last_update_time;
        double dt_ms = std::chrono::duration<double, std::milli>(dt_duration).count();

        // Report timing statistics periodically. THis can be nuked ir made a debug only setting once we are
        // happy
        update_count++;
        if (update_count >= timing_report_interval) {
            double expected_dt_ms = 1000.0 / PELTIER_CONTROL_UPDATE_HZ;  // Expected: 100ms at 10 Hz
            double timing_error_ms = dt_ms - expected_dt_ms;
            printf("[Peltier Timing] Update rate: %.2f Hz (dt=%.2f ms, expected=%.2f ms, error=%.2f ms)\n",
                    1000.0 / dt_ms, dt_ms, expected_dt_ms, timing_error_ms);
            update_count = 0;
        }
    }
    last_update_time = now;

    // Guard 1: Skip if not enabled
    if (!_enable) {
        _power = 0;
        auto duty = uint16_t(0);
        for (auto&& pin : Pins::active().peltier_pwm)
            if (pin) pwm_set_gpio_duty(pin, duty);
        return;
    }

    // Guard 2: Skip if no PWM pins configured
    bool has_pwm_pins = false;
    for (auto&& pin : Pins::active().peltier_pwm)
        if (pin) has_pwm_pins = true;

    if (!has_pwm_pins) {
        _power = 0;
        return;
    }

    // Guard 3: Skip if sensor data is invalid (NaN)
    if (!std::isfinite(double(sensors.temperature_cold)) || !std::isfinite(double(sensors.temperature_hot))) {
        _power = 0;
        auto duty = uint16_t(0);
        for (auto&& pin : Pins::active().peltier_pwm)
            if (pin) pwm_set_gpio_duty(pin, duty);
        return;
    }

    // Check safety limits
    if (!check_safety_limits(sensors, peltier_settings)) {
        // Safety violation - power is already set to 0 by set_error_state()
        auto duty = uint16_t(0);
        for (auto&& pin : Pins::active().peltier_pwm)
            if (pin) pwm_set_gpio_duty(pin, duty);
        return;
    }

    // Clear error state if we were in error and now passed safety checks
    if (_error_state) {
        clear_error_state();
    }

    // Check if auto-tune is active - it takes over control
    if (g_autotune.active) {
        // Get intake temperature for auto-tune
        double intake_temp;
        {
            auto sensor_lock = sensors::sensors_guard();
            intake_temp = double(sensors::g_sensors.temperature_intake);
        }

        double autotune_power = g_autotune.update(intake_temp);
        if (autotune_power >= 0) {
            _power = static_cast<BLE::Percentage8>(autotune_power);

            auto duty = uint16_t(numeric_limits<uint16_t>::max() * (autotune_power / 100.0));
            for (auto&& pin : Pins::active().peltier_pwm)
                if (pin) pwm_set_gpio_duty(pin, duty);

            // If auto-tune just completed, apply results to settings
            if (g_autotune.is_complete()) {
                printf("[PID AutoTune] Applying results: Kp=%.3f Ki=%.3f Kd=%.3f\n", g_autotune.result_kp,
                        g_autotune.result_ki, g_autotune.result_kd);
                p_settings.kp = g_autotune.result_kp;
                p_settings.ki = g_autotune.result_ki;
                p_settings.kd = g_autotune.result_kd;
                // Note: Call save_settings() if you want to persist the results
            }
            return;
        }
    }

    // Store previous power for change detection
    auto prev_power = _power;

    if (peltier_settings.control_mode == 0) {
        // PID control mode
        update_pid(sensors, peltier_settings);
    } else {
        // Watermark
        update_watermark(sensors, peltier_settings);
    }

    // Notify if power changed by more than 1%
    if (abs(double(_power) - double(prev_power)) > 1.0) {
        g_notify_peltier_power.notify();
    }

    auto duty = uint16_t(numeric_limits<uint16_t>::max() * (double(_power) / 100.0));

    static BLE::Percentage8 last_logged_power = 0;
    static uint32_t pwm_log_count = 0;
    static constexpr uint32_t PWM_LOG_INTERVAL = 50;  // Log every 50 updates (5 seconds at 10 Hz)

    pwm_log_count++;
    bool power_changed = abs(double(_power) - double(last_logged_power)) > 5.0;  // Log if changed by >5%

    if (power_changed || pwm_log_count >= PWM_LOG_INTERVAL) {
        printf("[Peltier PWM] Power: %.1f%%, Duty: %u/65535, Target: %.2f°C, Current: %.2f°C\n",
                double(_power), duty, double(_target), double(sensors.temperature_cold));
        last_logged_power = _power;
        pwm_log_count = 0;
    }

    for (auto&& pin : Pins::active().peltier_pwm)
        if (pin) pwm_set_gpio_duty(pin, duty);
}

// Track if hardware is available
bool g_hardware_available = false;

}  // namespace

bool init() {
    // Load settings from persistent storage first (but don't restore enable state yet)
    // We need the settings loaded for PWM initialization
    load_settings_only();

    // Check if Peltier hardware is connected (PWM pin and both sensor pins)
    bool has_pwm_pin = false;
    bool has_cold_sensor = false;
    bool has_hot_sensor = false;

    for (auto&& pin : Pins::active().peltier_pwm) {
        if (pin) {
            has_pwm_pin = true;
            auto cfg = pwm_get_default_config();
            // Fix: Correct PWM frequency calculation to use 1 / cycle_time
            float pwm_freq_hz = 1.0f / p_settings.cycle_time;
            pwm_config_set_freq_hz(cfg, pwm_freq_hz);
            // Initialize PWM hardware
            pwm_init(pwm_gpio_to_slice_num_(pin), &cfg, true);
            printf("Peltier PWM initialized for pin %u: frequency=%.2f Hz (cycle_time=%.3f s)\n",
                    static_cast<uint8_t>(pin), pwm_freq_hz, p_settings.cycle_time);
        }
    }

    // Check if ADC thermistor sensors 0 and 1 are configured (both required)
    if (Pins::active().adc_thermistor[0]) {
        has_cold_sensor = true;
    }
    if (Pins::active().adc_thermistor[1]) {
        has_hot_sensor = true;
    }

    if (has_cold_sensor && has_hot_sensor) {
        printf("Mapping ADC sensors to Peltier: sensor 0 -> cold side, sensor 1 -> hot side\n");
        g_peltier_mapper = new PeltierMapper();
        g_peltier_mapper->start();
    } else {
        printf("Peltier requires two thermistor sensors (cold=%d, hot=%d)\n", has_cold_sensor,
                has_hot_sensor);
    }

    g_hardware_available = has_pwm_pin && has_cold_sensor && has_hot_sensor;

    // Now restore the control state (target and enable) AFTER sensor mapping is initialized
    // This ensures sensor readings are available when we restore the enable flag
    load_control_state();

    // If hardware is not available, disable Peltier and clear enable flag
    if (!g_hardware_available) {
        printf("Peltier disabled (hardware not connected: pwm=%d, cold_sensor=%d, hot_sensor=%d)\n",
                has_pwm_pin, has_cold_sensor, has_hot_sensor);
        g_peltier.enable(0);
    } else {
        // Check if intake environmental sensors are available for dew point calculation
        auto sensor_lock = sensors::sensors_guard();
        bool has_intake_temp = sensors::g_sensors.temperature_intake != BLE::NOT_KNOWN;
        bool has_intake_humidity = sensors::g_sensors.humidity_intake != BLE::NOT_KNOWN;

        if (!has_intake_temp || !has_intake_humidity) {
            printf("Peltier WARNING: intake sensors not available at startup for dew point calculation "
                   "(temp=%s, humidity=%s). Enable will fail until sensors are available.\n",
                    has_intake_temp ? "OK" : "MISSING", has_intake_humidity ? "OK" : "MISSING");
            // Don't disable - sensors might become available later
        }

        printf("Peltier hardware ready, %s\n", g_peltier.enabled() ? "enabled" : "disabled");
    }

    // Temperature notification timer - notify when temperature changes (only if hardware available)
    if (g_hardware_available) {
        mk_timer("gatt-peltier-temp-notify", SENSOR_UPDATE_PERIOD)([](auto*) {
            static sensors::PeltierSensors g_prev;
            if (g_prev.temperature_cold == sensors::p_sensors.temperature_cold &&
                    g_prev.temperature_hot == sensors::p_sensors.temperature_hot)
                return;

            g_prev = sensors::p_sensors;
            g_notify_peltier_temp.notify();
        });

        mk_timer("peltier-control", 1.s / PELTIER_CONTROL_UPDATE_HZ)(
                [](auto*) { g_peltier.update(sensors::p_sensors, p_settings); });
    }

    return true;
}

void disconnected(hci_con_handle_t conn) {
    g_notify_peltier_temp.unregister(conn);
    g_notify_peltier_power.unregister(conn);
    g_notify_peltier_error.unregister(conn);
}

optional<uint16_t> attr_read(
        hci_con_handle_t const conn, uint16_t const attr, uint16_t const offset, span<uint8_t> const buffer) {
    switch (attr) {
        USER_DESCRIBE(PELTIER_SENSOR_COLD_TEMP, "Peltier Cold Side Temperature")
        USER_DESCRIBE(PELTIER_SENSOR_HOT_TEMP, "Peltier Hot Side Temperature")
        USER_DESCRIBE(PELTIER_POWER, "Peltier PWM Power %")
        USER_DESCRIBE(PELTIER_ERROR_STATUS, "Peltier Error Status")

        READ_VALUE(PELTIER_SENSOR_COLD_TEMP, sensors::p_sensors.temperature_cold)
        READ_VALUE(PELTIER_SENSOR_HOT_TEMP, sensors::p_sensors.temperature_hot)
        READ_VALUE(PELTIER_POWER, g_peltier.power())
        READ_VALUE(PELTIER_ERROR_STATUS, static_cast<uint8_t>(g_peltier.error_type()))

        USER_DESCRIBE(PELTIER_TARGET_TEMP, "Peltier Target Temperature")
        USER_DESCRIBE(PELTIER_ENABLE, "Peltier Enable")

        READ_VALUE(PELTIER_TARGET_TEMP, g_peltier.target_temp())
        READ_VALUE(PELTIER_ENABLE, g_peltier.enabled())

        USER_DESCRIBE(PELTIER_MIN_TEMP_COLD_SIDE, "Peltier Min Cold Side Temp")
        USER_DESCRIBE(PELTIER_MAX_TEMP_COLD_SIDE, "Peltier Max Cold Side Temp")
        USER_DESCRIBE(PELTIER_MIN_TEMP_HOT_SIDE, "Peltier Min Hot Side Temp")
        USER_DESCRIBE(PELTIER_MAX_TEMP_HOT_SIDE, "Peltier Max Hot Side Temp")
        USER_DESCRIBE(PELTIER_MAX_DEVIATION, "Peltier Max Temperature Deviation")
        USER_DESCRIBE(PELTIER_ENABLE_DELAY, "Peltier Enable Delay")
        USER_DESCRIBE(PELTIER_CYCLE_TIME, "Peltier PWM Cycle Time")
        USER_DESCRIBE(PELTIER_KP, "Peltier PID Kp")
        USER_DESCRIBE(PELTIER_KI, "Peltier PID Ki")
        USER_DESCRIBE(PELTIER_KD, "Peltier PID Kd")
        USER_DESCRIBE(PELTIER_SMOOTH_TIME, "Peltier PID Smooth Time")
        USER_DESCRIBE(PELTIER_DEW_POINT_SAFETY, "Peltier Dew Point Safety Margin")
        USER_DESCRIBE(PELTIER_HOT_SIDE_SAFETY, "Peltier Hot Side Safety Margin")
        USER_DESCRIBE(PELTIER_CONTROL_MODE, "Peltier Control Mode")
        USER_DESCRIBE(PELTIER_WATERMARK_HIGH, "Peltier Watermark High Threshold")
        USER_DESCRIBE(PELTIER_WATERMARK_LOW, "Peltier Watermark Low Threshold")
        USER_DESCRIBE(PELTIER_AUTOTUNE, "Peltier PID Auto-Tune")

        READ_VALUE(PELTIER_MIN_TEMP_COLD_SIDE, BLE::Temperature(p_settings.min_temp_cold))
        READ_VALUE(PELTIER_MAX_TEMP_COLD_SIDE, BLE::Temperature(p_settings.max_temp_cold))
        READ_VALUE(PELTIER_MIN_TEMP_HOT_SIDE, BLE::Temperature(p_settings.min_temp_hot))
        READ_VALUE(PELTIER_MAX_TEMP_HOT_SIDE, BLE::Temperature(p_settings.max_temp_hot))
        READ_VALUE(PELTIER_MAX_DEVIATION, BLE::Temperature(p_settings.max_deviation))
        READ_VALUE(PELTIER_ENABLE_DELAY, BLE::TimeSecond16(p_settings.enable_delay))
        READ_VALUE(PELTIER_CYCLE_TIME, BLE::TimeSecond16(p_settings.cycle_time))
        READ_VALUE(PELTIER_KP, p_settings.kp)
        READ_VALUE(PELTIER_KI, p_settings.ki)
        READ_VALUE(PELTIER_KD, p_settings.kd)
        READ_VALUE(PELTIER_SMOOTH_TIME, BLE::TimeSecond16(p_settings.smooth_time))
        READ_VALUE(PELTIER_DEW_POINT_SAFETY, BLE::Temperature(p_settings.dew_point_safety))
        READ_VALUE(PELTIER_HOT_SIDE_SAFETY, BLE::Temperature(p_settings.hot_side_safety))
        READ_VALUE(PELTIER_CONTROL_MODE, p_settings.control_mode)
        READ_VALUE(PELTIER_WATERMARK_HIGH, BLE::Temperature(p_settings.watermark_high))
        READ_VALUE(PELTIER_WATERMARK_LOW, BLE::Temperature(p_settings.watermark_low))

        // Auto-tune status: 0=idle, 1=running, 2=complete
    case HANDLE_ATTR(PELTIER_AUTOTUNE, VALUE): {
        uint8_t status = 0;
        if (g_autotune.active) {
            status = 1;  // running
        } else if (g_autotune.is_complete()) {
            status = 2;  // complete (results available)
        }
        return att_read_callback_handle_blob(status, offset, buffer);
    }

        READ_CLIENT_CFG(PELTIER_SENSOR_COLD_TEMP, g_notify_peltier_temp)
        READ_CLIENT_CFG(PELTIER_SENSOR_HOT_TEMP, g_notify_peltier_temp)
        READ_CLIENT_CFG(PELTIER_POWER, g_notify_peltier_power)
        READ_CLIENT_CFG(PELTIER_ERROR_STATUS, g_notify_peltier_error)

    default: return {};
    }
}

optional<int> attr_write(hci_con_handle_t conn, uint16_t attr, span<uint8_t const> buffer) {
    WriteConsumer consume{buffer};

    switch (attr) {
        WRITE_CLIENT_CFG(PELTIER_SENSOR_COLD_TEMP, g_notify_peltier_temp)
        WRITE_CLIENT_CFG(PELTIER_SENSOR_HOT_TEMP, g_notify_peltier_temp)
        WRITE_CLIENT_CFG(PELTIER_POWER, g_notify_peltier_power)
        WRITE_CLIENT_CFG(PELTIER_ERROR_STATUS, g_notify_peltier_error)

    // Control characteristics
    case HANDLE_ATTR(PELTIER_TARGET_TEMP, VALUE): {
        BLE::Temperature value = consume;
        // Validate range [-10, 50]°C
        if (value < -10.0f || value > 50.0f) {
            throw AttrWriteException(ATT_ERROR_VALUE_NOT_ALLOWED);
        }
        g_peltier.target(value);
        save_settings();
        return 0;
    }

    case HANDLE_ATTR(PELTIER_ENABLE, VALUE): {
        uint8_t value = consume;
        // Only allow enabling if hardware is available
        if (value != 0 && !g_hardware_available) {
            printf("Cannot enable Peltier: hardware not connected\n");
            throw AttrWriteException(ATT_ERROR_WRITE_NOT_PERMITTED);
        }
        // Attempt to enable - will validate pins and sensor readings
        if (!g_peltier.enable(value)) {
            throw AttrWriteException(ATT_ERROR_WRITE_NOT_PERMITTED);
        }
        save_settings();
        return 0;
    }

    case HANDLE_ATTR(PELTIER_ERROR_STATUS, VALUE): {
        uint8_t value = consume;
        if (value == 0) {
            g_peltier.clear_error_state();
        }
        return 0;
    }

    case HANDLE_ATTR(PELTIER_MIN_TEMP_COLD_SIDE, VALUE): {
        BLE::Temperature value = consume;
        p_settings.min_temp_cold = double(value);
        save_settings();
        return 0;
    }

    case HANDLE_ATTR(PELTIER_MAX_TEMP_COLD_SIDE, VALUE): {
        BLE::Temperature value = consume;
        p_settings.max_temp_cold = double(value);
        save_settings();
        return 0;
    }

    case HANDLE_ATTR(PELTIER_MIN_TEMP_HOT_SIDE, VALUE): {
        BLE::Temperature value = consume;
        p_settings.min_temp_hot = double(value);
        save_settings();
        return 0;
    }

    case HANDLE_ATTR(PELTIER_MAX_TEMP_HOT_SIDE, VALUE): {
        BLE::Temperature value = consume;
        p_settings.max_temp_hot = double(value);
        save_settings();
        return 0;
    }

    case HANDLE_ATTR(PELTIER_MAX_DEVIATION, VALUE): {
        BLE::Temperature value = consume;
        p_settings.max_deviation = double(value);
        save_settings();
        return 0;
    }

    case HANDLE_ATTR(PELTIER_ENABLE_DELAY, VALUE): {
        BLE::TimeSecond16 value = consume;
        p_settings.enable_delay = double(value);
        save_settings();
        return 0;
    }

    case HANDLE_ATTR(PELTIER_CYCLE_TIME, VALUE): {
        BLE::TimeSecond16 value = consume;
        p_settings.cycle_time = double(value);
        save_settings();
        return 0;
    }

    // PID parameters
    case HANDLE_ATTR(PELTIER_KP, VALUE): {
        float value = consume;
        p_settings.kp = value;
        save_settings();
        return 0;
    }

    case HANDLE_ATTR(PELTIER_KI, VALUE): {
        float value = consume;
        p_settings.ki = value;
        save_settings();
        return 0;
    }

    case HANDLE_ATTR(PELTIER_KD, VALUE): {
        float value = consume;
        p_settings.kd = value;
        save_settings();
        return 0;
    }

    case HANDLE_ATTR(PELTIER_SMOOTH_TIME, VALUE): {
        BLE::TimeSecond16 value = consume;
        p_settings.smooth_time = double(value);
        save_settings();
        return 0;
    }

    // Safety parameters
    case HANDLE_ATTR(PELTIER_DEW_POINT_SAFETY, VALUE): {
        BLE::Temperature value = consume;
        // Validate range [0, 20]°C
        if (value < 0.0f || value > 20.0f) {
            throw AttrWriteException(ATT_ERROR_VALUE_NOT_ALLOWED);
        }
        p_settings.dew_point_safety = double(value);
        save_settings();
        return 0;
    }

    case HANDLE_ATTR(PELTIER_HOT_SIDE_SAFETY, VALUE): {
        BLE::Temperature value = consume;
        // Validate range [0, 30]°C
        if (value < 0.0f || value > 30.0f) {
            throw AttrWriteException(ATT_ERROR_VALUE_NOT_ALLOWED);
        }
        p_settings.hot_side_safety = double(value);
        save_settings();
        return 0;
    }

    // Watermark control parameters
    case HANDLE_ATTR(PELTIER_CONTROL_MODE, VALUE): {
        uint8_t value = consume;
        // Validate (0=PID, 1=Watermark)
        if (value > 1) {
            throw AttrWriteException(ATT_ERROR_VALUE_NOT_ALLOWED);
        }
        p_settings.control_mode = value;
        save_settings();
        return 0;
    }

    case HANDLE_ATTR(PELTIER_WATERMARK_HIGH, VALUE): {
        BLE::Temperature value = consume;

        if (value < 0.1f || value > 10.0f) {
            throw AttrWriteException(ATT_ERROR_VALUE_NOT_ALLOWED);
        }
        p_settings.watermark_high = double(value);
        save_settings();
        return 0;
    }

    case HANDLE_ATTR(PELTIER_WATERMARK_LOW, VALUE): {
        BLE::Temperature value = consume;
        if (value < 0.1f || value > 10.0f) {
            throw AttrWriteException(ATT_ERROR_VALUE_NOT_ALLOWED);
        }
        p_settings.watermark_low = double(value);
        save_settings();
        return 0;
    }

    case HANDLE_ATTR(PELTIER_AUTOTUNE, VALUE): {
        uint8_t command = consume;
        if (command == 1) {
            // Start auto-tune
            if (!g_peltier.enabled()) {
                printf("Cannot start auto-tune: Peltier not enabled\n");
                throw AttrWriteException(ATT_ERROR_WRITE_NOT_PERMITTED);
            }
            if (g_autotune.active) {
                printf("Auto-tune already in progress\n");
                throw AttrWriteException(ATT_ERROR_WRITE_NOT_PERMITTED);
            }
            // Check if intake temperature is available
            if (sensors::g_sensors.temperature_intake == BLE::NOT_KNOWN) {
                printf("Cannot start auto-tune: no intake temperature\n");
                throw AttrWriteException(ATT_ERROR_WRITE_NOT_PERMITTED);
            }
            // Start calibration with current target
            printf("Starting PID auto-tune at target %.2f°C\n", double(g_peltier.target_temp()));
            g_autotune.start(double(g_peltier.target_temp()), 100.0);  // max_power = 100%
            return 0;
        } else if (command == 2) {
            // Apply calculated PID values
            if (g_autotune.is_complete()) {
                printf("Applying auto-tuned PID: Kp=%.4f, Ki=%.6f, Kd=%.4f\n", g_autotune.result_kp,
                        g_autotune.result_ki, g_autotune.result_kd);
                p_settings.kp = g_autotune.result_kp;
                p_settings.ki = g_autotune.result_ki;
                p_settings.kd = g_autotune.result_kd;
                save_settings();
            } else {
                printf("No auto-tune results to apply\n");
                throw AttrWriteException(ATT_ERROR_VALUE_NOT_ALLOWED);
            }
            return 0;
        } else if (command == 0) {
            // Cancel/stop auto-tune
            g_autotune.stop();
            return 0;
        }
        throw AttrWriteException(ATT_ERROR_VALUE_NOT_ALLOWED);
    }

    default: return {};
    }
}

}  // namespace nevermore::gatt::peltier