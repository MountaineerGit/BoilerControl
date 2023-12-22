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
  float value = 23.5f;
  ESP_LOGD(TAG, "'%s' Temperature=%.1f%%", this->get_name().c_str(), value);
  this->publish_state(value);

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
