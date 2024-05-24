import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import (
    DEVICE_CLASS_CONNECTIVITY,
    ENTITY_CATEGORY_DIAGNOSTIC,
)
from . import CONF_WWRESI_ID, WWRESIComponent, wwresi_ns

CONF_CONNECTED = "connected"


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_WWRESI_ID): cv.use_id(WWRESIComponent),
        cv.Required(CONF_CONNECTED): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_CONNECTIVITY,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    }
)


async def to_code(config):
    server = await cg.get_variable(config[CONF_WWRESI_ID])

    sens = await binary_sensor.new_binary_sensor(config[CONF_CONNECTED])
    cg.add(server.set_connected_sensor(sens))
