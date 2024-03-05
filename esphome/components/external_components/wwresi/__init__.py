import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

CODEOWNERS = ["@ed-so"]

DEPENDENCIES = ["uart"]

MULTI_CONF = True

wwresi_ns = cg.esphome_ns.namespace("wwresi")
WWRESIComponent = wwresi_ns.class_("WWRESIComponent", cg.Component, uart.UARTDevice)

CONF_WWRESI_ID = "wwresi_ns"

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(WWRESIComponent),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "wwresi_uart",
    require_tx=True,
    require_rx=True,
    parity="NONE",
    stop_bits=1,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
