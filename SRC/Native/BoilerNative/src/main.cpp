#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sstream>

#include "driver/gpio.h"
 
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
  POS4_REFLOW
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

//-----------------------------------------------------------------------------
# define PARASITE_POWER_ARG false
// Globals
LCD_I2C lcd;

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

//-----------------------------------------------------------------------------
static void init_lcd();
static void print_temperature(const enum LCD_POSTION pos, const int temperature);
static void print_error(const enum LCD_POSTION pos, const enum APP_ERROR err);
static void printScratchpad(const DSTherm::Scratchpad& scrpd);


extern "C"
void app_main()
{
  uint16_t rtd;
  Max31865Error fault = Max31865Error::NoError;
  float temperatureSolar = 0.0f;
  float temperatureBoiler = 0.0f;

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

  ESP_ERROR_CHECK(tempSensorBoiler.begin(tempConfig));
  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_ERROR_CHECK(tempSensorSolar.begin(tempConfig));
  vTaskDelay(pdMS_TO_TICKS(100));

  //ESP_ERROR_CHECK(tempSensorBoiler.setRTDThresholds(0x2000, 0x2500));
 // ESP_ERROR_CHECK(tempSensorSolar.setRTDThresholds(0x2000, 0x2500));

  ESP_LOGD("main", "Main loop");
  while (true)
  {
    if(tempSensorSolar.getRTD(&rtd, &fault) == ESP_OK)
    {
      ESP_LOGI("rtdS", "%.2d ", rtd);
      temperatureSolar = Max31865::RTDtoTemperature(rtd, rtdConfigSolar);

      ESP_LOGI("TemperatureS", "%.2f C", temperatureSolar);
      print_temperature(LCD_POSTION::POS1_SOLAR, static_cast<int>(temperatureSolar));
      vTaskDelay(pdMS_TO_TICKS(500));
    } else  {
      print_error(LCD_POSTION::POS1_SOLAR, APP_ERROR::TEMP_SENSOR_FAIL);
    }


    if(tempSensorBoiler.getRTD(&rtd, &fault) == ESP_OK)
    {
      ESP_LOGI("rtdB", "%.2d ", rtd);
      temperatureBoiler = Max31865::RTDtoTemperature(rtd, rtdConfigBoiler);
      ESP_LOGI("TemperatureB", "%.2f C", temperatureBoiler);
      print_temperature(LCD_POSTION::POS2_BOILER, static_cast<int>(temperatureBoiler));
      vTaskDelay(pdMS_TO_TICKS(500));

    } else  {
      print_error(LCD_POSTION::POS1_SOLAR, APP_ERROR::TEMP_SENSOR_FAIL);
    }

    if(temperatureSolar - temperatureBoiler > 7.0f) {
      gpio_set_level(LED_GPIO_NUM, LED_ON);
    } else {
      gpio_set_level(LED_GPIO_NUM, LED_OFF);
    }

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
    default:break;
  }

  lcd.setCursor(col, row);
}

static void print_error(const enum LCD_POSTION pos, const enum APP_ERROR err)
{
  set_lcd_cursor(pos);
  lcd.print("ET1");
}

static void print_temperature(const enum LCD_POSTION pos, const int temperature)
{
 // return ;
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
      print_temperature(LCD_POSTION::POS3_INFLOW, (int)temp / 16);
      zu = false;
    } else {
      print_temperature(LCD_POSTION::POS4_REFLOW, (int)temp / 16);
      zu = true;
    }
}
