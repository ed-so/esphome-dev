import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

CODEOWNERS = ["@ed-so"]

DEPENDENCIES = ["uart"]

MULTI_CONF = True

wwmr01_ns = cg.esphome_ns.namespace("wwmr01")
WWMR01Component = wwmr01_ns.class_("WWMR01Component", cg.Component, uart.UARTDevice)

CONF_WWMR01_ID = "wwmr01_ns"

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(WWMR01Component),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "wwmr01_uart",
    require_tx=True,
    require_rx=True,
    parity="NONE",
    stop_bits=1,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
