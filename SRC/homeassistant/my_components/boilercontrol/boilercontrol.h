#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

namespace esphome {
namespace boilercontrol {

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
  void set_pin(InternalGPIOPin *pin) { pin_ = pin; }

  void setup() override;
  float get_setup_priority() const override;
  void dump_config() override;
  void update() override;
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
};



}  // namespace boilercontrol


}  // namespace esphome
