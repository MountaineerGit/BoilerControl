
#ifndef SOLAR_BOILER_CONTROLLER_H
#define SOLAR_BOILER_CONTROLLER_H

#include <float.h>

class TemperatureRange
{
    public:
        float min;
        float max;
public:
    TemperatureRange() {};
    TemperatureRange(float min, float max) : min(min), max(max) 
    {  };
    virtual ~TemperatureRange() {};
};

class SolarBoilerController
{

public:

    /* Temperature difference with the best efficiency. 
    * When the temperature difference between solar and boiler is
    * greater than seven degrees, than turn the pump on (heat water).
    * Value retrieved from some local plumping guy :).
    */
    static constexpr float OPTIMAL_DELTA_TEMPERATURE = 7.00f;

    enum PUMP_STATE {
        EMERGENCY_TURN_OFF = -1,
        PUMP_OFF = 0,
        TURN_PUMP_OFF = PUMP_OFF,
        PUMP_ON,
        TURN_PUMP_ON = PUMP_ON
    };

private:
    float _boiler_temperature = FLT_MAX;
    float _solar_temperature = FLT_MAX;
    unsigned int _invalid_temperature_counter = 0;

    enum PUMP_STATE _pump_state = PUMP_STATE::PUMP_OFF;
    enum PUMP_STATE _previous_pump_state = PUMP_STATE::PUMP_OFF;

    /* Expected temperature range, for filtering invalid values */
    const TemperatureRange TEMPERATURE_FILTER = TemperatureRange(-40.0f, 180.0f);

    /* How often ist the temperature expected to be updated */
    int _temperature_update_time_sec = 60;

    const int STATE_CHANGE_COUNTER_THRESHOLD;
    const int EMERGENCY_TURN_OFF_THRESHOLD;

public:
    SolarBoilerController(int temperature_update_time);

    virtual ~SolarBoilerController() {}

    void setBoilerTemperature(const float temp) {
        if(temp > TEMPERATURE_FILTER.min && temp < TEMPERATURE_FILTER.max) {
            _boiler_temperature = temp;
        } else {
            _boiler_temperature = FLT_MAX;
            _invalid_temperature_counter++;
        }
    }

    float getBoilerTemperature() {
        return _boiler_temperature;
    }

    void setSolarTemperature(const float temp) {
        if(temp > TEMPERATURE_FILTER.min && temp < TEMPERATURE_FILTER.max) {
            _solar_temperature = temp;
        } else {
            _solar_temperature = FLT_MAX;
            _invalid_temperature_counter++;
        }
    }

    float getSolarTemperature() {
        return _solar_temperature;
    }

    PUMP_STATE getPumpAction(); 

};

#endif /* SOLAR_BOILER_CONTROLLER_H */

