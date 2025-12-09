#include "sensors/adc_thermistor.hpp"

#include "sensors.hpp"

#include <cmath>
#include <hardware/adc.h>
#include <pico/time.h>

namespace nevermore::sensors {

ADCThermistorSensor::ADCThermistorSensor(uint32_t adc_channel, uint8_t gpio_pin, const char* sensor_name,
        BLE::Temperature& output_ref, ThermistorConfig config)
        : adc_channel_(adc_channel), gpio_pin_(gpio_pin), sensor_name_(sensor_name), output_ref_(output_ref),
          config_(config) {}

bool ADCThermistorSensor::is_thermistor_connected(uint32_t adc_channel) {
    // Select ADC input channel
    adc_select_input(adc_channel);

    // Take multiple samples to average out noise
    constexpr int NUM_SAMPLES = 16;
    uint32_t sum = 0;
    uint16_t min_raw = 0xFFFF;
    uint16_t max_raw = 0;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        uint16_t raw_value = adc_read();
        sum += raw_value;
        if (raw_value < min_raw) min_raw = raw_value;
        if (raw_value > max_raw) max_raw = raw_value;
        // Small delay between samples
        busy_wait_us(100);
    }

    // Calculate average
    uint16_t avg_raw = sum / NUM_SAMPLES;

    // Scale 12-bit reading to 16-bit value
    uint16_t scaled_value = avg_raw << 4;

    // Convert to voltage using V_REF / 65535.0 scale coefficient
    constexpr double V_REF = 3.3;
    double voltage = (scaled_value * V_REF) / 65535.0;

    // Debug output
    printf("  ADC%lu: raw=%u (min=%u, max=%u, range=%u), voltage=%.3fV\n", (unsigned long)adc_channel,
            avg_raw, min_raw, max_raw, max_raw - min_raw, voltage);

    // Detection thresholds:
    // - Open circuit (disconnected): voltage > 3.0V (pulled high by series resistor)
    // - Short to ground: voltage < 0.2V
    // - Valid thermistor: 0.2V to 3.0V
    constexpr double V_LOW = 0.2;   // Below this = short to ground
    constexpr double V_HIGH = 3.0;  // Above this = open circuit (disconnected)

    if (voltage < V_LOW) {
        printf("  -> Rejected: voltage too low (< %.2fV, likely short to ground)\n", V_LOW);
        return false;
    }

    if (voltage > V_HIGH) {
        printf("  -> Rejected: voltage too high (> %.2fV, likely open circuit/disconnected)\n", V_HIGH);
        return false;
    }

    // Voltage is in valid range - thermistor appears to be connected
    printf("  -> Accepted: voltage in valid range [%.2fV - %.2fV]\n", V_LOW, V_HIGH);
    return true;
}

std::optional<double> ADCThermistorSensor::measure_adc_temperature(
        uint32_t adc_channel, const ThermistorConfig& config) {
    // Select ADC input and read raw 12-bit value
    adc_select_input(adc_channel);
    uint16_t raw_value = adc_read();

    // Scale to 16-bit value using bit shifting
    uint16_t scaled_value = raw_value << 4;

    // Convert to voltage using config.v_ref and scale coefficient
    double voltage = (scaled_value * config.v_ref) / 65535.0;

    // Return std::nullopt if voltage <= 0.0 or >= v_ref (invalid reading)
    if (voltage <= 0.0 || voltage >= config.v_ref) {
        return std::nullopt;
    }

    // Calculate thermistor resistance using voltage divider formula
    // Circuit: V_ref --- R_thermistor --- ADC_pin --- R_series --- GND
    // R_thermistor = R_series * (V_ref - V) / V
    double resistance = config.r_series * (config.v_ref - voltage) / voltage;

    // Apply Steinhart-Hart equation: 1/T = 1/T0 + (1/B) * ln(R/R0)
    // T0 is in Kelvin: T0 = t_nominal + 273.15
    double t0_kelvin = config.t_nominal + 273.15;
    double temp_kelvin = 1.0 / ((1.0 / t0_kelvin) + (1.0 / config.b_coefficient) *
                                                            std::log(resistance / config.r_nominal));

    // Convert from Kelvin to Celsius
    double temp_celsius = temp_kelvin - 273.15;

    // Return std::nullopt if temperature < -50°C or > 150°C (sanity check)
    if (temp_celsius < -50.0 || temp_celsius > 150.0) {
        return std::nullopt;
    }

    // Return optional<double> with valid temperature
    return temp_celsius;
}

char const* ADCThermistorSensor::name() const {
    return sensor_name_;
}

void ADCThermistorSensor::read() {
    // Call measure_adc_temperature() to get the temperature reading
    auto temperature = measure_adc_temperature(adc_channel_, config_);

    // Only update output_ref_ if measurement returns valid value
    if (temperature.has_value()) {
        // Acquire sensors_guard() lock before updating output_ref_
        auto lock = sensors_guard();
        output_ref_ = BLE::Temperature(temperature.value());
    }

    // Log ADC temperature every second (read() is called at 1Hz)
    if (++log_counter_ >= 1) {
        log_counter_ = 0;

        // Get intake/exhaust temperatures
        auto lock = sensors_guard();
        double intake_temp = double(g_sensors.temperature_intake);
        double exhaust_temp = double(g_sensors.temperature_exhaust);

        if (temperature.has_value()) {
            printf("[ADC Thermistor ADC%lu] GPIO%u: %.2f°C | Intake: %.2f°C, Exhaust: %.2f°C\n",
                    (unsigned long)adc_channel_, gpio_pin_, temperature.value(), intake_temp, exhaust_temp);
        } else {
            printf("[ADC Thermistor ADC%lu] GPIO%u: invalid reading | Intake: %.2f°C, Exhaust: %.2f°C\n",
                    (unsigned long)adc_channel_, gpio_pin_, intake_temp, exhaust_temp);
        }
    }
}

}  // namespace nevermore::sensors
