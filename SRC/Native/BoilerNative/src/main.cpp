#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "Max31865.h"

extern "C" 
void app_main() 
{
  auto tempSensor = Max31865(
  9 /*miso*/, 
  10 /*mosi*/, 
  8 /*sck*/, 
  5 /*cs2 GPIO5, D3*/);
  max31865_config_t tempConfig = {};
  tempConfig.autoConversion = true;
  tempConfig.vbias = true;
  tempConfig.filter = Max31865Filter::Hz50;
  tempConfig.nWires = Max31865NWires::Two;
  max31865_rtd_config_t rtdConfig = 
  {
    .ref = 4753.0f,
    .nominal = 1000.0f
  };

  ESP_ERROR_CHECK(tempSensor.begin(tempConfig));
  //ESP_ERROR_CHECK(tempSensor.setRTDThresholds(0x2000, 0x2500));

  while (true) {
    uint16_t rtd;
    Max31865Error fault = Max31865Error::NoError;
    ESP_ERROR_CHECK(tempSensor.getRTD(&rtd, &fault));
    ESP_LOGI("rtd", "%.2d ", rtd);

    float temp = Max31865::RTDtoTemperature(rtd, rtdConfig);
    ESP_LOGI("Temperature", "%.2f C", temp);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
