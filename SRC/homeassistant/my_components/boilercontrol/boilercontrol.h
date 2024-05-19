#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#include "SolarBoilerController.h"

#include "Max31865.h"
#include "OneWireNg_CurrentPlatform.h"
#include "drivers/DSTherm.h"

namespace esphome {
namespace boilercontrol {

enum class APP_ERROR : uint8_t;
enum class LCD_POSTION : uint8_t;

/// Store data in a class that doesn't use multiple-inheritance (vtables in flash)
struct BoilerControlComponentStore {
  volatile uint32_t last_interrupt{0};
  volatile uint32_t on_time{0};
  volatile bool last_level{false};
  ISRInternalGPIOPin pin;

  static void gpio_intr(BoilerControlComponentStore *arg);
};

class BoilerPump : public binary_sensor::BinarySensor {
 public:

  void set_state_pump_on() { this->state = true; }
  void set_state_pump_off() { this->state = false; }
};

class BoilerControlComponent : public sensor::Sensor, public PollingComponent {
 public:

  BoilerControlComponent();

  void set_pin(InternalGPIOPin *pin) { pin_ = pin; }

  void setup() override;
  float get_setup_priority() const override;
  void dump_config() override;
  void update() override;
  void loop() override;

 void set_temperature_sensor_boiler(sensor::Sensor *temperature_sensor) 
 { temperature_sensor_boiler_ = temperature_sensor; }
 
  void set_temperature_sensor_solar(sensor::Sensor *temperature_sensor) 
 { temperature_sensor_solar_ = temperature_sensor; }

   void set_temperature_sensor_influx(sensor::Sensor *temperature_sensor) 
 { temperature_sensor_influx_ = temperature_sensor; }

   void set_temperature_sensor_reflux(sensor::Sensor *temperature_sensor) 
 { temperature_sensor_reflux_ = temperature_sensor; }


   void set_pump_binary_sensor(BoilerPump *pump) 
 { pump_ = pump; }

 protected:
  InternalGPIOPin *pin_;

  sensor::Sensor *temperature_sensor_boiler_{nullptr};
  sensor::Sensor *temperature_sensor_solar_{nullptr};
  sensor::Sensor *temperature_sensor_influx_{nullptr};
  sensor::Sensor *temperature_sensor_reflux_{nullptr};

  //BoilerPump *pump_{nullptr};
  binary_sensor::BinarySensor *pump_{nullptr};

  BoilerControlComponentStore store_{};
  uint32_t last_update_{0};

  void lcd_init();
  void set_lcd_cursor(const enum LCD_POSTION pos);
  void lcd_print_temperature(const enum LCD_POSTION pos, const int temperature);
  void lcd_print_temp_sensor_error(const enum LCD_POSTION pos, const enum APP_ERROR err);
  void lcd_print_app_error(const enum APP_ERROR err);
  void lcd_pump_indicator(SolarBoilerController::PUMP_STATE state);

  SolarBoilerController::PUMP_STATE pumpAction(SolarBoilerController::PUMP_STATE state);

  float getMax31865Temperature(Max31865 &max, const enum LCD_POSTION pos);

  void oneWirePrintTemperature(const DSTherm::Scratchpad& scrpd);

private:
  float temperatureSolar = 0.0f;
  float temperatureBoiler = 0.0f;
  int temperatureInflow = 0;
  int temperatureReflow = 0;
  SolarBoilerController::PUMP_STATE pump_state;
  // how fast to re-measure temperatures
  // must be greater 0
  static constexpr int UPDATE_TIME_SEC = 1;
  int update_time_sec = UPDATE_TIME_SEC;
};



}  // namespace boilercontrol
}  // namespace esphome
