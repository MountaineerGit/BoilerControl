#include "boilercontrol.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"


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

static const char *const TAG = "boilercontrol";

static OneWireNg_CurrentPlatform *ow = nullptr;

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

namespace esphome {
//namespace components {
namespace boilercontrol {

//-----------------------------------------------------------------------------
void BoilerControlComponent::setup() 
{
  ESP_LOGCONFIG(TAG, "Setting up boilercontrol Sensor '%s'...", this->get_name().c_str());

  // -- OneWire Temperature Sensors --
  ow = new OneWireNg_CurrentPlatform(3,false);

  // -- LCD --
  ESP_LOGD("setup", "Init LCD");
  lcd_init();

  // -- GPIO::LED --
  ESP_LOGD("setup", "GPIO");
  gpio_config(&LED_GPIO_CFG);
  gpio_set_level(LED_GPIO_NUM, LED_OFF);

  // TODO: is this the trouble-maker ???
  //gpio_config(&PUMP_CTRL_GPIO_CFG);
  //gpio_set_level(TRIAC_PUMP_GPIO_NUM, SolarBoilerController::PUMP_STATE::PUMP_OFF);

  // -- Max38165 --
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

void BoilerControlComponent::lcd_init()
{
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
}

void BoilerControlComponent::lcd_print_temperature(const enum LCD_POSTION pos, const int temperature)
{
  set_lcd_cursor(pos);
  lcd.print("   "); /* clear old result*/
  set_lcd_cursor(pos);
  if(temperature < 1000) {
    lcd.print(temperature);
  } else {
    lcd.print("ET2");
  }
  vTaskDelay(pdMS_TO_TICKS(500));
}
void BoilerControlComponent::lcd_print_temp_sensor_error(const enum LCD_POSTION pos, const enum APP_ERROR err)
{
  set_lcd_cursor(pos);
  lcd.print("ET1");
}
void BoilerControlComponent::lcd_print_app_error(const enum APP_ERROR err)
{
    lcd.clear();
    lcd.setCursor(0, 0);
    if(err == APP_ERROR::WATCHDOG_TRIGGERED) {
      lcd.print("Error: ETW");
    }
}
void BoilerControlComponent::lcd_pump_indicator(SolarBoilerController::PUMP_STATE state)
{
  static bool lcd_motor_animation = false;

  if(state != SolarBoilerController::PUMP_STATE::PUMP_ON) {
    lcd.setCursor(LCD_SEPARATOR_POSITION, 0);
    lcd.print("|");
    lcd.setCursor(LCD_SEPARATOR_POSITION, 1);
    lcd.print("|");
  } else {
    lcd.setCursor(LCD_SEPARATOR_POSITION, 0);
    lcd.print(lcd_motor_animation ? "+" : "*");
    lcd.setCursor(LCD_SEPARATOR_POSITION, 1);
    lcd.print(lcd_motor_animation ? "+" : "*");

    lcd_motor_animation = !lcd_motor_animation;
  }
}

SolarBoilerController::PUMP_STATE BoilerControlComponent::pumpAction(SolarBoilerController::PUMP_STATE state)
{
  if(state == SolarBoilerController::PUMP_STATE::TURN_PUMP_ON) {
     gpio_set_level(LED_GPIO_NUM, LED_ON);
     /* TODO: gpio for triac */
     // gpio_set_level(TRIAC_PUMP_GPIO_NUM, SolarBoilerController::PUMP_STATE::PUMP_ON);
  } else {
     gpio_set_level(LED_GPIO_NUM, LED_OFF);
     gpio_set_level(TRIAC_PUMP_GPIO_NUM, SolarBoilerController::PUMP_STATE::PUMP_OFF);

     // TODO: indicate emergency off
  }

  return state;
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

float BoilerControlComponent::getMax31865Temperature(Max31865 &max, const enum LCD_POSTION pos)
{
  uint16_t rtd;
  float temperature = 0.0f;
  Max31865Error fault = Max31865Error::NoError;

    if(max.getRTD(&rtd, &fault) == ESP_OK)
    {
      temperature = Max31865::RTDtoTemperature(rtd, 
        pos==LCD_POSTION::POS1_SOLAR ? 
          rtdConfigSolar : 
          rtdConfigBoiler);

      if(pos==LCD_POSTION::POS1_SOLAR) {
            ESP_LOGI("rtdS", "%.2d ", rtd);
            ESP_LOGI("TemperatureS", "%.2f C", temperature);
      } else {
            ESP_LOGI("rtdB", "%.2d ", rtd);
            ESP_LOGI("TemperatureB", "%.2f C", temperature);
      }

      vTaskDelay(pdMS_TO_TICKS(100));
      lcd_print_temperature(pos, static_cast<int>(temperature));
      vTaskDelay(pdMS_TO_TICKS(100));
    } else  {
      lcd_print_temp_sensor_error(pos, APP_ERROR::TEMP_SENSOR_FAIL);
    }

    return temperature;
}

}  // namespace duty_cycle
//}
}  // namespace esphome
