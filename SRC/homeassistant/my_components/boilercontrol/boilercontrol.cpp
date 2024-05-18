#include "boilercontrol.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "SolarBoilerController.h"

#include "version.h"

#include "driver/gpio.h"

#define ONEWIRE
#define MAX31865

#ifdef ONEWIRE
#include "OneWireNg_CurrentPlatform.h"
#include "drivers/DSTherm.h"
#endif

#include "../../../HD44780.h"

#ifdef MAX31865
#include "Max31865.h"
#endif
//-----------------------------------------------------------------------------
// constants and defines

static const std::string APP_NAME = " Thermocontrol";

static const std::string APP_VERSION = "    v " +
  MAJOR_VERSION + "." +
  MINOR_VERSION + "." +
  PATCH_VERSION;

enum class LCD_POSTION : uint8_t
{
  POS1_SOLAR = 0,
  POS2_BOILER,
  POS3_INFLOW,
  POS4_REFLOW,
  POS5_PUMP_ON_INDICATOR
};

#ifdef MAX31865
// MAX31865 consts
const max31865_config_t tempConfigMax38165 =
{
  .vbias = true,
  .autoConversion = false,
  .nWires = Max31865NWires::Two,
  .faultDetection = Max31865FaultDetection::AutoDelay,
  .filter = Max31865Filter::Hz50
};

const max31865_rtd_config_t rtdConfigBoiler =
{
  .ref = 4304.0f, // TODO: adjust to proper settings
  .nominal = 1000.0f,
  .sensor_type = SensorType::PT1000
};

const max31865_rtd_config_t rtdConfigSolar =
{
  .ref = 4304.0f, // TODO: adjust to proper settings
  .nominal = 1630.0f,
  .sensor_type = SensorType::KTY81_210
};

Max31865 tempSensorBoiler = Max31865(
  9 /*miso*/,
  10 /*mosi*/,
  8 /*sck*/,
  5 /*cs2 GPIO5, D3*/);

Max31865 tempSensorSolar = Max31865(
  9 /*miso*/,
  10 /*mosi*/,
  8 /*sck*/,
  4 /*cs1 GPIO4, D2*/);

enum class APP_ERROR : uint8_t
{
  TEMP_SENSOR_FAIL = 0,
  WATCHDOG_TRIGGERED
};
#endif

static constexpr uint32_t WATCHDOG_TIMEOUT_MS = 10*1000;

// how fast to re-measure temperatures
// must be greater 0
static constexpr int UPDATE_TIME_SEC = 1;

SolarBoilerController pump_controller = SolarBoilerController(UPDATE_TIME_SEC);

// LED GPIO2/A0/D0
#define LED_GPIO_NUM GPIO_NUM_2
#define LED_ON 1
#define LED_OFF 0
const gpio_config_t LED_GPIO_CFG =
{
  .pin_bit_mask = 1ULL << LED_GPIO_NUM,
  .mode = GPIO_MODE_OUTPUT,
  .pull_up_en = GPIO_PULLUP_DISABLE,
  .pull_down_en = GPIO_PULLDOWN_ENABLE,
  .intr_type = GPIO_INTR_DISABLE
};

// PUMP/MotorControl via Triac GPIO7/A7/D7
#define TRIAC_PUMP_GPIO_NUM GPIO_NUM_7
const gpio_config_t PUMP_CTRL_GPIO_CFG =
{
  .pin_bit_mask = 1ULL << TRIAC_PUMP_GPIO_NUM,
  .mode = GPIO_MODE_OUTPUT,
  .pull_up_en = GPIO_PULLUP_DISABLE,
  .pull_down_en = GPIO_PULLDOWN_ENABLE,
  .intr_type = GPIO_INTR_DISABLE
};

//-----------------------------------------------------------------------------
# define PARASITE_POWER_ARG false
// Globals

//-----------------------------------------------------------------------------
// LCD display
//-----------------------------------------------------------------------------
LCD_I2C lcd;

static constexpr int LCD_SEPARATOR_POSITION = 8;
//////////////////////////////////////
//----------------------------------//
// S o l : 9 0     |   Z u : 8 5    //
// B o i : 4 5     |   A b : 6 0    //
//----------------------------------//
// 0 1 2 3 4 5 6 7 8 9 A B C D E F  //
//////////////////////////////////////

#ifdef ONEWIRE
// OneWireNg_CurrentPlatform ow = OneWireNg_CurrentPlatform(3 /*GPIO3,A1,D1*/, false);
 //DSTherm drv(ow);
#endif

namespace esphome {
//namespace components {
namespace boilercontrol {

static const char *const TAG = "boilercontrol";

static OneWireNg_CurrentPlatform *ow = nullptr;

void BoilerControlComponent::setup() 
{
  ESP_LOGCONFIG(TAG, "Setting up boilercontrol Sensor '%s'...", this->get_name().c_str());

  // -- OneWire Temperature Sensors --
  ow = new OneWireNg_CurrentPlatform(3,false);

  // -- LCD --
  ESP_LOGD("setup", "Init LCD");
  lcd.init(0x27, 6 /*sda, GPIO6*/, 7 /*scl, GPIO7*/, 2, 16);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(APP_NAME);
  lcd.setCursor(0,1);
  lcd.print(APP_VERSION);
  vTaskDelay(pdMS_TO_TICKS(2000));
  lcd.clear();

  lcd.setCursor(0,0); lcd.print("Sol:");
  lcd.setCursor(8,0); lcd.print("| Zu:");
  lcd.setCursor(0,1); lcd.print("Boi:");
  lcd.setCursor(8,1); lcd.print("| Ab:");

  // -- GPIO::LED --
  ESP_LOGD("setup", "GPIO");
  gpio_config(&LED_GPIO_CFG);
  gpio_set_level(LED_GPIO_NUM, LED_OFF);

  // -- GPIO::LED --
  ESP_LOGD("setup", "Max38165");
  ESP_ERROR_CHECK(tempSensorBoiler.begin(tempConfigMax38165));
  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_ERROR_CHECK(tempSensorSolar.begin(tempConfigMax38165));
  vTaskDelay(pdMS_TO_TICKS(100));

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

  if(pump_)
    pump_->publish_state(true);

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
