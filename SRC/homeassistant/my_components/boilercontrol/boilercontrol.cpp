#include "boilercontrol.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
//namespace components {
namespace boilercontrol {

static const char *const TAG = "boilercontrol";

void BoilerControlComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up boilercontrol Sensor '%s'...", this->get_name().c_str());
  /*this->pin_->setup();
  this->store_.pin = this->pin_->to_isr();
  this->store_.last_level = this->pin_->digital_read();
  this->store_.last_interrupt = micros();*/

  //this->pin_->attach_interrupt(BoilerControlComponentStore::gpio_intr, &this->store_, gpio::INTERRUPT_ANY_EDGE);
}
void BoilerControlComponent::dump_config() {
  LOG_SENSOR("", "BoilerControlComponent Sensor", this);
  LOG_PIN("  Pin: ", this->pin_);
  LOG_UPDATE_INTERVAL(this);
}

void BoilerControlComponent::update() 
{
  float temperature = 23.52f;
  ESP_LOGD(TAG, "'%s' Temperature=%.1f%%", this->get_name().c_str(), temperature);

  if (this->temperature_sensor_boiler_ != nullptr)
    this->temperature_sensor_boiler_->publish_state(temperature);

  if (this->temperature_sensor_solar_ != nullptr)
    this->temperature_sensor_solar_->publish_state(temperature + 20.31f);

  if (this->temperature_sensor_influx_ != nullptr)
    this->temperature_sensor_influx_->publish_state(temperature + 10.33f);

  if (this->temperature_sensor_reflux_ != nullptr)
    this->temperature_sensor_reflux_->publish_state(temperature - 2.25f);

  pump_.publish_state(pump_.state);

  this->status_clear_warning();
}

float BoilerControlComponent::get_setup_priority() const 
{ return setup_priority::DATA; }

void IRAM_ATTR BoilerControlComponentStore::gpio_intr(BoilerControlComponentStore *arg) 
{
  return;
}

}  // namespace duty_cycle
//}
}  // namespace esphome
