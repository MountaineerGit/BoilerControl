#include "SolarBoilerController.h"

SolarBoilerController::SolarBoilerController(int temperature_update_time) :
    _temperature_update_time_sec(temperature_update_time),
    STATE_CHANGE_COUNTER_THRESHOLD(5 /* TOOD: calc based on _temperature_update_time_sec */),
    EMERGENCY_TURN_OFF_THRESHOLD(5 /* TOOD: calc based on _temperature_update_time_sec */)
{

}

SolarBoilerController::PUMP_STATE SolarBoilerController::getPumpAction()
{
    static int state_change_counter = 0;
    static int invalid_state_counter = 0;

    if(_boiler_temperature < FLT_MAX && _solar_temperature < FLT_MAX)
    {
        const float delta = _solar_temperature - _boiler_temperature;

        if(delta > 0.0f)
        {
            if(delta > OPTIMAL_DELTA_TEMPERATURE) {
                if(state_change_counter < STATE_CHANGE_COUNTER_THRESHOLD) {
                    state_change_counter++;
                    return _previous_pump_state;
                } else {
                    _previous_pump_state = PUMP_ON;
                    return TURN_PUMP_ON;
                }
            } else {
                /* the delta is too small, turn pump off*/
                if(state_change_counter > 0) {
                    state_change_counter--;
                    return _previous_pump_state;
                } else {
                    _previous_pump_state = PUMP_OFF;
                    return TURN_PUMP_OFF;
                }
            }
        } else {
            /* Negative deltas are unexpected. We simply turn pump off. */
            state_change_counter = 0;
            _previous_pump_state = PUMP_OFF;
            return TURN_PUMP_OFF;
        }
    } else {
        if(invalid_state_counter < EMERGENCY_TURN_OFF_THRESHOLD) {
            invalid_state_counter++;
            return _previous_pump_state;
        } else {
            /* No valid values for EMERGENCY_TURN_OFF_THRESHOLD-times.
             * Lets better turn off the pump.
             */
            invalid_state_counter = 0;
            _previous_pump_state = EMERGENCY_TURN_OFF;
            return EMERGENCY_TURN_OFF;
        }
    }

    return _previous_pump_state;
}
