#pragma once

#include "sdk/ble_data_types.hpp"
#include "sensors/async_sensor.hpp"

#include <cstdint>
#include <optional>

namespace nevermore::sensors {

// Configuration for NTC thermistor calibration
struct ThermistorConfig {
    double r_series = 10000.0;      // Series resistor (Ohms)
    double r_nominal = 10000.0;     // Thermistor resistance at T_nominal (Ohms)
    double t_nominal = 25.0;        // Nominal temperature (°C)
    double b_coefficient = 3950.0;  // Beta coefficient (K)
    double v_ref = 3.3;             // Reference voltage (V)
};

class ADCThermistorSensor final : public SensorPeriodic {
public:
    // Constructor for a single ADC channel
    ADCThermistorSensor(uint32_t adc_channel, uint8_t gpio_pin, const char* sensor_name,
            BLE::Temperature& output_ref, ThermistorConfig config = ThermistorConfig{});

    // SensorPeriodic interface
    [[nodiscard]] char const* name() const override;
    void read() override;

    // Static utility functions
    static bool is_thermistor_connected(uint32_t adc_channel);
    static std::optional<double> measure_adc_temperature(
            uint32_t adc_channel, const ThermistorConfig& config = ThermistorConfig{});

private:
    uint32_t adc_channel_;
    uint8_t gpio_pin_;
    const char* sensor_name_;
    BLE::Temperature& output_ref_;
    ThermistorConfig config_;
    uint32_t log_counter_ = 0;
};

}  // namespace nevermore::sensors
