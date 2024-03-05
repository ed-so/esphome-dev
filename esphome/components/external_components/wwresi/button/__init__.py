import esphome.codegen as cg
from esphome.components import button
import esphome.config_validation as cv
from esphome.const import (
    DEVICE_CLASS_RESTART,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ENTITY_CATEGORY_CONFIG,
    ICON_RESTART,
    ICON_RESTART_ALERT,
    ICON_DATABASE,
)
from .. import CONF_WWRESI_ID, WWRESIComponent, wwresi_ns

WWRESIApplyConfigButton = wwresi_ns.class_("WWRESIApplyConfigButton", button.Button)
WWRESIRevertConfigButton = wwresi_ns.class_("WWRESIRevertConfigButton", button.Button)
WWRESIRestartModuleButton = wwresi_ns.class_("WWRESIRestartModuleButton", button.Button)
WWRESIFactoryResetButton = wwresi_ns.class_("WWRESIFactoryResetButton", button.Button)

CONF_APPLY_CONFIG = "apply_config"
CONF_REVERT_CONFIG = "revert_config"
CONF_RESTART_MODULE = "restart_module"
CONF_FACTORY_RESET = "factory_reset"


CONFIG_SCHEMA = {
    cv.GenerateID(CONF_WWRESI_ID): cv.use_id(WWRESIComponent),
    cv.Required(CONF_APPLY_CONFIG): button.button_schema(
        WWRESIApplyConfigButton,
        device_class=DEVICE_CLASS_RESTART,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon=ICON_RESTART_ALERT,
    ),
    cv.Optional(CONF_REVERT_CONFIG): button.button_schema(
        WWRESIRevertConfigButton,
        device_class=DEVICE_CLASS_RESTART,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon=ICON_RESTART,
    ),
    cv.Optional(CONF_RESTART_MODULE): button.button_schema(
        WWRESIRestartModuleButton,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        icon=ICON_DATABASE,
    ),
    cv.Optional(CONF_FACTORY_RESET): button.button_schema(
        WWRESIFactoryResetButton,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon=ICON_DATABASE,
    ),
}


async def to_code(config):
    wwresi_component = await cg.get_variable(config[CONF_WWRESI_ID])
    if apply_config := config.get(CONF_APPLY_CONFIG):
        b = await button.new_button(apply_config)
        await cg.register_parented(b, config[CONF_WWRESI_ID])
        cg.add(wwresi_component.set_apply_config_button(b))
    if revert_config := config.get(CONF_REVERT_CONFIG):
        b = await button.new_button(revert_config)
        await cg.register_parented(b, config[CONF_WWRESI_ID])
        cg.add(wwresi_component.set_revert_config_button(b))
    if restart_config := config.get(CONF_RESTART_MODULE):
        b = await button.new_button(restart_config)
        await cg.register_parented(b, config[CONF_WWRESI_ID])
        cg.add(wwresi_component.set_restart_module_button(b))
    if factory_reset := config.get(CONF_FACTORY_RESET):
        b = await button.new_button(factory_reset)
        await cg.register_parented(b, config[CONF_WWRESI_ID])
        cg.add(wwresi_component.set_factory_reset_button(b))
