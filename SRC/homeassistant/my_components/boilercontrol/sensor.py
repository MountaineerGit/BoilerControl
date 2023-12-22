import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import sensor
from esphome.const import (
    CONF_PIN,
    CONF_ID,
    STATE_CLASS_MEASUREMENT,
    CONF_TEMPERATURE,
    UNIT_PERCENT,
    ICON_PERCENT,
    UNIT_CELSIUS,
    DEVICE_CLASS_TEMPERATURE
)


boiler_control_ns = cg.esphome_ns.namespace("boilercontrol")
BoilerControlComponent = boiler_control_ns.class_(
    "BoilerControlComponent",   sensor.Sensor, cg.PollingComponent
)


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(BoilerControlComponent),
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            )
        }
    )
    .extend(cv.polling_component_schema("60s"))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))