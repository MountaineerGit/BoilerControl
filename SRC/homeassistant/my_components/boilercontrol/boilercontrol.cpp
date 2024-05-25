#include "boilercontrol.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"


#include "version.h"

#include "driver/gpio.h"

#define ONEWIRE_INIT
#define ONEWIRE_LOOP
#define MAX31865

#include "../../../HD44780.h"
#include "utils/Placeholder.h"

//-----------------------------------------------------------------------------
// constants and defines

namespace esphome {
//namespace components {
namespace boilercontrol {

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
  APP_OK = 0,
  TEMP_SENSOR_FAIL,
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
static DSTherm *drv = nullptr;

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

// PUMP/MotorControl via Triac GPIO20/D7
#define TRIAC_PUMP_GPIO_NUM GPIO_NUM_20
const gpio_config_t PUMP_CTRL_GPIO_CFG =
{
  .pin_bit_mask = 1ULL << TRIAC_PUMP_GPIO_NUM,
  .mode = GPIO_MODE_OUTPUT,
  .pull_up_en = GPIO_PULLUP_DISABLE,
  .pull_down_en = GPIO_PULLDOWN_ENABLE,
  .intr_type = GPIO_INTR_DISABLE
};

//-----------------------------------------------------------------------------
// constructor
BoilerControlComponent::BoilerControlComponent() : PollingComponent(30'000 /* TODO: 15 * 60 * 1000 */ /* update interval milliseconds */ ) 
{
  /* nop */
}

void BoilerControlComponent::setup() 
{
  ESP_LOGCONFIG(TAG, "Setting up boilercontrol Sensor '%s'...", this->get_name().c_str());

  pump_state = SolarBoilerController::PUMP_STATE::PUMP_OFF;

#ifdef ONEWIRE_INIT
  // -- OneWire Temperature Sensors --
  ow = new OneWireNg_CurrentPlatform(3 /*GPIO3,A1,D1*/, false);
  drv = new DSTherm(*ow);
#endif

  // -- LCD --
  ESP_LOGD("setup", "Init LCD");
  lcd_init();

  // -- GPIO::LED --
  ESP_LOGD("setup", "GPIO");
  gpio_config(&LED_GPIO_CFG);
  gpio_set_level(LED_GPIO_NUM, LED_OFF);

  gpio_config(&PUMP_CTRL_GPIO_CFG);
  gpio_set_level(TRIAC_PUMP_GPIO_NUM, SolarBoilerController::PUMP_STATE::PUMP_OFF);

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

void BoilerControlComponent::set_lcd_cursor(const enum LCD_POSTION pos)
{
  int row=0;
  int col=0;

  switch (pos)
  {
    case LCD_POSTION::POS1_SOLAR:  row=0; col=4;  break;
    case LCD_POSTION::POS2_BOILER: row=1; col=4;  break;
    case LCD_POSTION::POS3_INFLOW: row=0; col=13; break;
    case LCD_POSTION::POS4_REFLOW: row=1; col=13; break;
    default: break;
  }

  lcd.setCursor(col, row);
}

void BoilerControlComponent::lcd_print_temperature(const enum LCD_POSTION pos, const int temperature)
{
  set_lcd_cursor(pos);
  
  if(pos <= LCD_POSTION::POS2_BOILER) {
    lcd.print("    "); /* clear old result (one space more) */
  } else {
    lcd.print("   "); /* clear old result,*/
  }
  
  set_lcd_cursor(pos);
    if(temperature > TEMPERATURE_FILTER.min && temperature < TEMPERATURE_FILTER.max) {
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
    lcd.print(lcd_motor_animation ? "-" : "*");
    lcd.setCursor(LCD_SEPARATOR_POSITION, 1);
    lcd.print(lcd_motor_animation ? "-" : "*");

    lcd_motor_animation = !lcd_motor_animation;
  }
}

SolarBoilerController::PUMP_STATE BoilerControlComponent::pumpAction(SolarBoilerController::PUMP_STATE state)
{
  if(state == SolarBoilerController::PUMP_STATE::TURN_PUMP_ON) {
     gpio_set_level(LED_GPIO_NUM, LED_ON);
     gpio_set_level(TRIAC_PUMP_GPIO_NUM, SolarBoilerController::PUMP_STATE::PUMP_ON);
  } else {
     gpio_set_level(LED_GPIO_NUM, LED_OFF);
     gpio_set_level(TRIAC_PUMP_GPIO_NUM, SolarBoilerController::PUMP_STATE::PUMP_OFF);

     // TODO: indicate emergency off
  }

  return state;
}

void BoilerControlComponent::dump_config() {
  // TODO: this break homeassistant ?
  //LOG_SENSOR("", "BoilerControlComponent Sensor", this);
  //LOG_PIN("  Pin: ", this->pin_);
  //LOG_UPDATE_INTERVAL(this);
}

void BoilerControlComponent::loop()
{
  lcd_pump_indicator(pump_state);  // we call this multiple times for nice display-animation

  if(getMax31865Temperature(tempSensorSolar, temperatureSolar, LCD_POSTION::POS1_SOLAR) == APP_ERROR::APP_OK)
  {
    pump_controller.setSolarTemperature(temperatureSolar);
  } else {
    pump_controller.setSolarTemperature(FLT_MAX);
  }
  vTaskDelay(pdMS_TO_TICKS(200));

  lcd_pump_indicator(pump_state);  // we call this multiple times for nice display-animation

  if(getMax31865Temperature(tempSensorBoiler, temperatureBoiler, LCD_POSTION::POS2_BOILER) == APP_ERROR::APP_OK)
  {
    pump_controller.setBoilerTemperature(temperatureBoiler);
  } else {
    pump_controller.setBoilerTemperature(FLT_MAX);
  }
  vTaskDelay(pdMS_TO_TICKS(200));
  
  /* feed readings to controller and check for pump action */
  pump_state = pumpAction(pump_controller.getPumpAction());

  if(pump_state == SolarBoilerController::PUMP_STATE::PUMP_ON) {
     ESP_LOGI(TAG,"** pump is ON");
  } else {
    ESP_LOGI(TAG, "** pump is OFF");
  }

  lcd_pump_indicator(pump_state);  // we call this multiple times for nice display-animation

# ifdef ONEWIRE_LOOP
  /* Influx, Reflux measurement */
  /* convert temperature on all sensors connected... */
  drv->convertTempAll(DSTherm::MAX_CONV_TIME, PARASITE_POWER_ARG);

  /* read sensors one-by-one */
  Placeholder<DSTherm::Scratchpad> scrpd;

  for (const auto& id: *ow) {
    
       if (drv->readScratchpad(id, scrpd) == OneWireNg::EC_SUCCESS) {
         oneWirePrintTemperature(scrpd);
       } else {
         ESP_LOGE(TAG, "  Read scratchpad error.\n");
       }
  }
#endif
}

void BoilerControlComponent::update() 
{
  //ESP_LOGD(TAG,"update()::publis");
  //float temperature = 23.52f;
  //ESP_LOGD(TAG, "'%s' Temperature=%.1f%%", this->get_name().c_str(), temperature);

  if (this->temperature_sensor_boiler_ != nullptr && tempSensorBoiler.getLastError() == Max31865Error::NoError)
    this->temperature_sensor_boiler_->publish_state(temperatureBoiler);

  if (this->temperature_sensor_solar_ != nullptr && tempSensorSolar.getLastError() == Max31865Error::NoError)
    this->temperature_sensor_solar_->publish_state(temperatureSolar);

  if (this->temperature_sensor_influx_ != nullptr)
    this->temperature_sensor_influx_->publish_state(temperatureInflow);

  if (this->temperature_sensor_reflux_ != nullptr)
    this->temperature_sensor_reflux_->publish_state(temperatureReflow);

  if(pump_) {
    pump_->publish_state(pump_state == SolarBoilerController::PUMP_STATE::PUMP_ON);
  }

  this->status_clear_warning();
}

float BoilerControlComponent::get_setup_priority() const 
{ return setup_priority::DATA; }

void IRAM_ATTR BoilerControlComponentStore::gpio_intr(BoilerControlComponentStore *arg) 
{
  return;
}

APP_ERROR BoilerControlComponent::getMax31865Temperature(Max31865 &max, float &temperature, const enum LCD_POSTION pos)
{
  uint16_t rtd;
  Max31865Error fault = Max31865Error::NoError;

    if(max.getRTD(&rtd, &fault) == ESP_OK)
    {
      temperature = Max31865::RTDtoTemperature(rtd, 
        pos==LCD_POSTION::POS1_SOLAR ? 
          rtdConfigSolar : 
          rtdConfigBoiler);

      if(pos==LCD_POSTION::POS1_SOLAR) {
            ESP_LOGD("rtdS", "%.2d ", rtd);
            ESP_LOGI("TemperatureS", "%.2f C", temperature);
      } else {
            ESP_LOGD("rtdB", "%.2d ", rtd);
            ESP_LOGI("TemperatureB", "%.2f C", temperature);
      }

      vTaskDelay(pdMS_TO_TICKS(100));
      lcd_print_temperature(pos, static_cast<int>(temperature));
      vTaskDelay(pdMS_TO_TICKS(100));
    } else  {
      lcd_print_temp_sensor_error(pos, APP_ERROR::TEMP_SENSOR_FAIL);
      return APP_ERROR::TEMP_SENSOR_FAIL;
    }

    return APP_ERROR ::APP_OK;
}

void BoilerControlComponent::oneWirePrintTemperature(const DSTherm::Scratchpad& scrpd)
{
  static bool inflow = false;

#if DEBUG
  const uint8_t *scrpd_raw = scrpd.getRaw();
  ESP_LOGD(TAG,"  Scratchpad:");

  for (size_t i = 0; i < DSTherm::Scratchpad::LENGTH; i++) {
      printf("%c%02x", (!i ? ' ' : ':'), scrpd_raw[i]);
  }


  ESP_LOGI("Th", "%d", scrpd.getTh());
  ESP_LOGI("Tl", "%d", scrpd.getTl());
  ESP_LOGI("Res", "%d", 9 + (int)(scrpd.getResolution() - DSTherm::RES_9_BIT));
  ESP_LOGI("Addr:", "%d", scrpd.getAddr());
#endif

  long temp = scrpd.getTemp2();
  ESP_LOGD("Temp:", "");
  if (temp < 0) {
      temp = -temp;
      ESP_LOGD("-","");
  }

  if(inflow)
  {
    temperatureInflow = (int)temp / 16;
    lcd_print_temperature(LCD_POSTION::POS3_INFLOW, temperatureInflow);
    ESP_LOGI(TAG, "inflow %d.%04d C", temperatureInflow, (10000 * ((int)temp % 16)) / 16);
  } else {
    temperatureReflow = (int)temp / 16;
    lcd_print_temperature(LCD_POSTION::POS4_REFLOW, temperatureReflow);
    ESP_LOGI(TAG, "reflow %d.%04d C", temperatureReflow, (10000 * ((int)temp % 16)) / 16);
  }

  inflow = !inflow;
}

}  // namespace duty_cycle
}  // namespace esphome

