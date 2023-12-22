#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
 // namespace component {
namespace boilercontrol {

/// Store data in a class that doesn't use multiple-inheritance (vtables in flash)
struct BoilerControlComponentStore {
  volatile uint32_t last_interrupt{0};
  volatile uint32_t on_time{0};
  volatile bool last_level{false};
  ISRInternalGPIOPin pin;

  static void gpio_intr(BoilerControlComponentStore *arg);
};

class BoilerControlComponent : public sensor::Sensor, public PollingComponent {
 public:
  void set_pin(InternalGPIOPin *pin) { pin_ = pin; }

  void setup() override;
  float get_setup_priority() const override;
  void dump_config() override;
  void update() override;
 void set_temperature_sensor(sensor::Sensor *temperature_sensor) 
 { temperature_sensor_ = temperature_sensor; }
 
 protected:
  InternalGPIOPin *pin_;

  sensor::Sensor *temperature_sensor_{nullptr};

  BoilerControlComponentStore store_{};
  uint32_t last_update_{0};
};

}  // namespace boilercontrol
//  }
}  // namespace esphome
