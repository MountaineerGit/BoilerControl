#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sstream>

#include "Max31865.h"
#include "HD44780.h"
#include "version.h"

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
  .autoConversion = true,
  .nWires = Max31865NWires::Two,
  //.faultDetection = Max31865FaultDetection::AutoDelay,
  .filter = Max31865Filter::Hz50
};

const max31865_rtd_config_t rtdConfig =
{
  .ref = 4313.0f, // TODO: adjust to proper settings
  .nominal = 1000.0f
};

enum class APP_ERROR : uint8_t
{
  TEMP_SENSOR_FAIL = 0,
};

//-----------------------------------------------------------------------------
// Globals
LCD_I2C lcd;

//-----------------------------------------------------------------------------
static void init_lcd();
static void print_temperature(const enum LCD_POSTION pos, const int temperature);
static void print_error(const enum LCD_POSTION pos, const enum APP_ERROR err);


extern "C"
void app_main()
{
  uint16_t rtd;
  Max31865Error fault = Max31865Error::NoError;
  float temperature;

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

 init_lcd();
      vTaskDelay(pdMS_TO_TICKS(500));


  ESP_ERROR_CHECK(tempSensorBoiler.begin(tempConfig));
  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_ERROR_CHECK(tempSensorSolar.begin(tempConfig));
  vTaskDelay(pdMS_TO_TICKS(100));

  //ESP_ERROR_CHECK(tempSensorBoiler.setRTDThresholds(0x2000, 0x2500));
 // ESP_ERROR_CHECK(tempSensorSolar.setRTDThresholds(0x2000, 0x2500));

  ESP_LOGD("main", "Init LCD");

  
  ESP_LOGD("main", "Main loop");
  while (true)
  {
    if(tempSensorSolar.getRTD(&rtd, &fault) == ESP_OK)
    {
      ESP_LOGI("rtdS", "%.2d ", rtd);
      temperature = Max31865::RTDtoTemperature(rtd, rtdConfig);
      ESP_LOGI("TemperatureS", "%.2f C", temperature);
      print_temperature(LCD_POSTION::POS1_SOLAR, static_cast<int>(temperature));
      vTaskDelay(pdMS_TO_TICKS(500));
    } else  {
      print_error(LCD_POSTION::POS1_SOLAR, APP_ERROR::TEMP_SENSOR_FAIL);
    }

#if 1
    if(tempSensorBoiler.getRTD(&rtd, &fault) == ESP_OK)
    {
      ESP_LOGI("rtdB", "%.2d ", rtd);
      temperature = Max31865::RTDtoTemperature(rtd, rtdConfig);
      ESP_LOGI("TemperatureB", "%.2f C", temperature);
      print_temperature(LCD_POSTION::POS2_BOILER, static_cast<int>(temperature));
      vTaskDelay(pdMS_TO_TICKS(500));

    } else  {
      print_error(LCD_POSTION::POS1_SOLAR, APP_ERROR::TEMP_SENSOR_FAIL);
    }
#endif
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
