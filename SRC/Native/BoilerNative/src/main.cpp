#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sstream>

#include "driver/gpio.h"
 
#include "SolarBoilerController.h"
#include "Max31865.h"
#include "HD44780.h"
#include "version.h"
#include <math.h>

#include "OneWireNg_CurrentPlatform.h"
#include "drivers/DSTherm.h"
#include "utils/Placeholder.h"
#include "platform/Platform_Delay.h"

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

// MAX31865 consts
const max31865_config_t tempConfig =
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

enum class APP_ERROR : uint8_t
{
  TEMP_SENSOR_FAIL = 0,
};

SolarBoilerController pump_controller = SolarBoilerController(60);

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

//static Placeholder<OneWireNg_CurrentPlatform> ow;
OneWireNg_CurrentPlatform ow = OneWireNg_CurrentPlatform(3 /*GPIO3,A1,D1*/, false);
DSTherm drv(ow);

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
static void init_lcd();
static void lcd_print_temperature(const enum LCD_POSTION pos, const int temperature);
static void lcd_print_error(const enum LCD_POSTION pos, const enum APP_ERROR err);
static void lcd_pump_indicator(SolarBoilerController::PUMP_STATE state);
static void printScratchpad(const DSTherm::Scratchpad& scrpd);

static SolarBoilerController::PUMP_STATE pumpAction(SolarBoilerController::PUMP_STATE state);
static float getMax31865Temperature(Max31865 &max, const enum LCD_POSTION pos);

extern "C"
void app_main()
{
  float temperatureSolar = 0.0f;
  float temperatureBoiler = 0.0f;
  SolarBoilerController::PUMP_STATE pump_state = SolarBoilerController::PUMP_STATE::PUMP_OFF;

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

  //    vTaskDelay(pdMS_TO_TICKS(500));

  ESP_LOGD("main", "Init LCD");
  init_lcd();
  vTaskDelay(pdMS_TO_TICKS(500));

  gpio_config(&LED_GPIO_CFG);
  gpio_set_level(LED_GPIO_NUM, LED_OFF);

  // TODO: is this the trouble-maker ???
  //gpio_config(&PUMP_CTRL_GPIO_CFG);
  //gpio_set_level(TRIAC_PUMP_GPIO_NUM, SolarBoilerController::PUMP_STATE::PUMP_OFF);

  ESP_ERROR_CHECK(tempSensorBoiler.begin(tempConfig));
  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_ERROR_CHECK(tempSensorSolar.begin(tempConfig));
  vTaskDelay(pdMS_TO_TICKS(100));

  //ESP_ERROR_CHECK(tempSensorBoiler.setRTDThresholds(0x2000, 0x2500));
 // ESP_ERROR_CHECK(tempSensorSolar.setRTDThresholds(0x2000, 0x2500));

  ESP_LOGD("main", "Main loop");
  while (true)
  {
    lcd_pump_indicator(pump_state);  // we call this multiple times for nice display-animation

    temperatureSolar = getMax31865Temperature(tempSensorSolar, LCD_POSTION::POS1_SOLAR);
    vTaskDelay(pdMS_TO_TICKS(500));

    lcd_pump_indicator(pump_state);  // we call this multiple times for nice display-animation
  
    temperatureBoiler = getMax31865Temperature(tempSensorBoiler, LCD_POSTION::POS2_BOILER);
    vTaskDelay(pdMS_TO_TICKS(500));

    /* feed readings to controller and check for pump action */
    pump_controller.setSolarTemperature(temperatureSolar);
    pump_controller.setBoilerTemperature(temperatureBoiler);
    pump_state = pumpAction(pump_controller.getPumpAction());

    lcd_pump_indicator(pump_state);  // we call this multiple times for nice display-animation

    if(pump_state == SolarBoilerController::PUMP_STATE::PUMP_ON) {
       ESP_LOGI("","** pump ON");
    }
    else
    {
      ESP_LOGI("", "** pump OFF");
    }


    /* Influx, Reflux measurement */
    /* convert temperature on all sensors connected... */
    drv.convertTempAll(DSTherm::MAX_CONV_TIME, PARASITE_POWER_ARG);

    /* read sensors one-by-one */
    Placeholder<DSTherm::Scratchpad> scrpd;

    for (const auto& id: ow) {
    
          if (drv.readScratchpad(id, scrpd) == OneWireNg::EC_SUCCESS)
              printScratchpad(scrpd);
          else
              printf("  Read scratchpad error.\n");
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

static void init_lcd()
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


#ifdef LCD_DEBUG_PRINT
  print_temperature(LCD_POSTION::POS1_SOLAR,120);
  vTaskDelay(pdMS_TO_TICKS(1000));
  print_temperature(LCD_POSTION::POS2_BOILER,140);
  vTaskDelay(pdMS_TO_TICKS(1000));
  print_temperature(LCD_POSTION::POS3_INFLOW, 900);
  vTaskDelay(pdMS_TO_TICKS(1000));
  print_temperature(LCD_POSTION::POS4_REFLOW, 1410);
#endif
}

static void set_lcd_cursor(const enum LCD_POSTION pos)
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

static void lcd_print_error(const enum LCD_POSTION pos, const enum APP_ERROR err)
{
  set_lcd_cursor(pos);
  lcd.print("ET1");
}

static void lcd_pump_indicator(SolarBoilerController::PUMP_STATE state)
{
  static bool turnaround = false;

  if(state != SolarBoilerController::PUMP_STATE::PUMP_ON) {
    lcd.setCursor(LCD_SEPARATOR_POSITION, 0);
    lcd.print("|");
    lcd.setCursor(LCD_SEPARATOR_POSITION, 1);
    lcd.print("|");
  } else {
    lcd.setCursor(LCD_SEPARATOR_POSITION, 0);
    lcd.print(turnaround ? "/" : "*");
    lcd.setCursor(LCD_SEPARATOR_POSITION, 1);
    lcd.print(turnaround ? "*" : "/");

    turnaround = !turnaround;
  }
}

static void lcd_print_temperature(const enum LCD_POSTION pos, const int temperature)
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


static void printScratchpad(const DSTherm::Scratchpad& scrpd)
{
    static bool zu = false;
    const uint8_t *scrpd_raw = scrpd.getRaw();

    ESP_LOGD("","  Scratchpad:");
    for (size_t i = 0; i < DSTherm::Scratchpad::LENGTH; i++)
        printf("%c%02x", (!i ? ' ' : ':'), scrpd_raw[i]);


      ESP_LOGI("Th", "%d", scrpd.getTh());
      ESP_LOGI("Tl", "%d", scrpd.getTl());
      ESP_LOGI("Res", "%d", 9 + (int)(scrpd.getResolution() - DSTherm::RES_9_BIT));
      ESP_LOGI("Addr:", "%d", scrpd.getAddr());

    long temp = scrpd.getTemp2();
    ESP_LOGI("Temp:", "");
    if (temp < 0) {
        temp = -temp;
        ESP_LOGD("-","");
    }
    ESP_LOGI("jo", "%d.%04d C\n", (int)temp / 16, (10000 * ((int)temp % 16)) / 16);

    if(zu)
    {
      lcd_print_temperature(LCD_POSTION::POS3_INFLOW, (int)temp / 16);
      zu = false;
    } else {
      lcd_print_temperature(LCD_POSTION::POS4_REFLOW, (int)temp / 16);
      zu = true;
    }
}

static SolarBoilerController::PUMP_STATE pumpAction(SolarBoilerController::PUMP_STATE state)
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

static float getMax31865Temperature(Max31865 &max, const enum LCD_POSTION pos)
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
      vTaskDelay(pdMS_TO_TICKS(500));
    } else  {
      lcd_print_error(pos, APP_ERROR::TEMP_SENSOR_FAIL);
    }

    return temperature;
}
