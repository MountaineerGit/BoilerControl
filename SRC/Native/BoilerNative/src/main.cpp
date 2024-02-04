#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sstream>

#include "Max31865.h"

#include <components/HD44780.h>

LCD_I2C lcd;

extern "C" 
void app_main() 
{
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

  max31865_config_t tempConfig = {};
  tempConfig.autoConversion = true;
  tempConfig.vbias = true;
  tempConfig.filter = Max31865Filter::Hz50;
  tempConfig.nWires = Max31865NWires::Two;
  max31865_rtd_config_t rtdConfig = 
  {
    .ref = 4313.0f,
    .nominal = 1000.0f
  };

  ESP_ERROR_CHECK(tempSensorBoiler.begin(tempConfig));
  ESP_ERROR_CHECK(tempSensorSolar.begin(tempConfig));
  //ESP_ERROR_CHECK(tempSensor.setRTDThresholds(0x2000, 0x2500));

  lcd.init(0x27, 6 /*sda, GPIO6*/, 7 /*scl, GPIO7*/, 2, 16);



  while (true) {

  #if 1
    uint16_t rtd;
    Max31865Error fault = Max31865Error::NoError;
    ESP_ERROR_CHECK(tempSensorBoiler.getRTD(&rtd, &fault));
    ESP_LOGI("rtd", "%.2d ", rtd);

    float tempBoiler = Max31865::RTDtoTemperature(rtd, rtdConfig);
    ESP_LOGI("TemperatureB", "%.2f C", tempBoiler);
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_ERROR_CHECK(tempSensorSolar.getRTD(&rtd, &fault));
    ESP_LOGI("rtd", "%.2d ", rtd);

    float tempSolar = Max31865::RTDtoTemperature(rtd, rtdConfig);
    ESP_LOGI("TemperatureS", "%.2f C", tempSolar);
    
    #endif
    vTaskDelay(pdMS_TO_TICKS(500));

std::ostringstream ss;
ss << "Boiler:" << tempBoiler;


//      lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(ss.str());
  lcd.setCursor(0,1);
  lcd.print("Solar: 75.8°C ");
  vTaskDelay(pdMS_TO_TICKS(1500));

  }
}
