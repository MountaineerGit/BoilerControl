import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome import automation
from esphome.components import binary_sensor, sensor
from esphome.const import (
    CONF_PIN,
    CONF_ID,
    STATE_CLASS_MEASUREMENT,
    CONF_TEMPERATURE,
    UNIT_PERCENT,
    ICON_PERCENT,
    UNIT_CELSIUS,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_POWER
)

CONF_TEMPERATURE_BOILER = "temperature_boiler"
CONF_TEMPERATURE_SOLAR = "temperature_solar"
CONF_TEMPERATURE_INFLUX = "temperature_influx"
CONF_TEMPERATURE_REFLUX = "temperature_reflux"

boiler_control_ns = cg.esphome_ns.namespace("boilercontrol")
BoilerControlComponent = boiler_control_ns.class_(
    "BoilerControlComponent",
    sensor.Sensor,
    binary_sensor.BinarySensor,
    cg.PollingComponent
)

BoilerPump = boiler_control_ns.class_(
    "BoilerPump", binary_sensor.BinarySensor, cg.PollingComponent
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(BoilerControlComponent),
            cv.Optional(CONF_TEMPERATURE_BOILER): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TEMPERATURE_SOLAR): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TEMPERATURE_INFLUX): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TEMPERATURE_REFLUX): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TEMPERATURE_REFLUX): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional("binary_sensor"): 
                binary_sensor.binary_sensor_schema(BoilerPump)
        }
    )
    .extend(cv.polling_component_schema("60s"))
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if "binary_sensor" in config:
        var2 = await binary_sensor.new_binary_sensor(config["binary_sensor"])
        cg.add(var.set_pump_binary_sensor(var2))

    if CONF_TEMPERATURE_BOILER in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE_BOILER])
        cg.add(var.set_temperature_sensor_boiler(sens))

    if CONF_TEMPERATURE_SOLAR in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE_SOLAR])
        cg.add(var.set_temperature_sensor_solar(sens))

    if CONF_TEMPERATURE_INFLUX in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE_INFLUX])
        cg.add(var.set_temperature_sensor_influx(sens))

    if CONF_TEMPERATURE_REFLUX in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE_REFLUX])
        cg.add(var.set_temperature_sensor_reflux(sens))
