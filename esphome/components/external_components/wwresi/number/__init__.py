import esphome.codegen as cg
from esphome.components import number
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_DISTANCE,
    ICON_OMEGA,
    UNIT_OHM,
    UNIT_SECOND,
    ENTITY_CATEGORY_CONFIG,
    ICON_MOTION_SENSOR,
    ICON_TIMELAPSE,
    ICON_SCALE,
)
from .. import CONF_WWRESI_ID, WWRESIComponent, wwresi_ns

WWRESITimeoutNumber = wwresi_ns.class_("WWRESITimeoutNumber", number.Number)
# WW_MR01MoveSensFactorNumber = wwresi_ns.class_(
#     "WW_MR01MoveSensFactorNumber", number.Number
# )
# WW_MR01StillSensFactorNumber = wwresi_ns.class_(
#     "WW_MR01StillSensFactorNumber", number.Number
# )
WWRESIResistanceNumber = wwresi_ns.class_("WWRESIResistanceNumber", number.Number)
# WW_MR01MaxDistanceNumber = wwresi_ns.class_("WW_MR01MaxDistanceNumber", number.Number)
# WW_MR01GateSelectNumber = wwresi_ns.class_("WW_MR01GateSelectNumber", number.Number)
# WW_MR01MoveThresholdNumbers = wwresi_ns.class_(
#     "WW_MR01MoveThresholdNumbers", number.Number
# )
# WW_MR01StillThresholdNumbers = wwresi_ns.class_(
#     "WW_MR01StillThresholdNumbers", number.Number
# )
CONF_RESISTANCE = "resistance"
# CONF_MAX_GATE_DISTANCE = "max_gate_distance"
# CONF_STILL_THRESHOLD = "still_threshold"
# CONF_MOVE_THRESHOLD = "move_threshold"
# CONF_GATE_MOVE_SENSITIVITY = "gate_move_sensitivity"
# CONF_GATE_STILL_SENSITIVITY = "gate_still_sensitivity"
# CONF_GATE_SELECT = "gate_select"
CONF_TIMEOUT = "timeout"
# GATE_GROUP = "gate_group"
TIMEOUT_GROUP = "timeout_group"


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_WWRESI_ID): cv.use_id(WWRESIComponent),

        cv.Inclusive(CONF_TIMEOUT, TIMEOUT_GROUP): number.number_schema(
            WWRESITimeoutNumber,
            unit_of_measurement=UNIT_SECOND,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon=ICON_TIMELAPSE,
        ),

        cv.Inclusive(CONF_RESISTANCE, TIMEOUT_GROUP): number.number_schema(
            WWRESIResistanceNumber,
            unit_of_measurement=UNIT_OHM,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon=ICON_OMEGA,
        ),

        # cv.Inclusive(CONF_MAX_GATE_DISTANCE, TIMEOUT_GROUP): number.number_schema(
        #     WW_MR01MaxDistanceNumber,
        #     device_class=DEVICE_CLASS_DISTANCE,
        #     entity_category=ENTITY_CATEGORY_CONFIG,
        #     icon=ICON_MOTION_SENSOR,
        # ),
        # cv.Inclusive(CONF_GATE_SELECT, GATE_GROUP): number.number_schema(
        #     WW_MR01GateSelectNumber,
        #     device_class=DEVICE_CLASS_DISTANCE,
        #     entity_category=ENTITY_CATEGORY_CONFIG,
        #     icon=ICON_MOTION_SENSOR,
        # ),
        # cv.Inclusive(CONF_STILL_THRESHOLD, GATE_GROUP): number.number_schema(
        #     WW_MR01StillThresholdNumbers,
        #     entity_category=ENTITY_CATEGORY_CONFIG,
        #     icon=ICON_MOTION_SENSOR,
        # ),
        # cv.Inclusive(CONF_MOVE_THRESHOLD, GATE_GROUP): number.number_schema(
        #     WW_MR01MoveThresholdNumbers,
        #     entity_category=ENTITY_CATEGORY_CONFIG,
        #     icon=ICON_MOTION_SENSOR,
        # ),
        # cv.Optional(CONF_GATE_MOVE_SENSITIVITY): number.number_schema(
        #     WW_MR01MoveSensFactorNumber,
        #     device_class=DEVICE_CLASS_DISTANCE,
        #     entity_category=ENTITY_CATEGORY_CONFIG,
        #     icon=ICON_SCALE,
        # ),
        # cv.Optional(CONF_GATE_STILL_SENSITIVITY): number.number_schema(
        #     WW_MR01StillSensFactorNumber,
        #     device_class=DEVICE_CLASS_DISTANCE,
        #     entity_category=ENTITY_CATEGORY_CONFIG,
        #     icon=ICON_SCALE,
        # ),
    }
)
# CONFIG_SCHEMA = CONFIG_SCHEMA.extend(
#     {
#         cv.Optional(f"gate_{x}"): (
#             {
#                 cv.Required(CONF_MOVE_THRESHOLD): number.number_schema(
#                     WW_MR01MoveThresholdNumbers,
#                     entity_category=ENTITY_CATEGORY_CONFIG,
#                     icon=ICON_MOTION_SENSOR,
#                 ),
#                 cv.Required(CONF_STILL_THRESHOLD): number.number_schema(
#                     WW_MR01StillThresholdNumbers,
#                     entity_category=ENTITY_CATEGORY_CONFIG,
#                     icon=ICON_MOTION_SENSOR,
#                 ),
#             }
#         )
#         for x in range(16)
#     }
# )


async def to_code(config):
    WWRESI_component = await cg.get_variable(config[CONF_WWRESI_ID])
    if timeout_config := config.get(CONF_TIMEOUT):
        n = await number.new_number(
            timeout_config, min_value=0, max_value=60, step=1
        )
        await cg.register_parented(n, config[CONF_WWRESI_ID])
        cg.add(WWRESI_component.set_timeout_number(n))

    if resistance_config := config.get(CONF_RESISTANCE):
        n = await number.new_number(
            resistance_config, min_value=-1, max_value=2666665, step=1
        )
        await cg.register_parented(n, config[CONF_WWRESI_ID])
        cg.add(WWRESI_component.set_resistance_number(n))
        
    # if max_distance_gate_config := config.get(CONF_MAX_GATE_DISTANCE):
    #     n = await number.new_number(
    #         max_distance_gate_config, min_value=1, max_value=15, step=1
    #     )
    #     await cg.register_parented(n, config[CONF_WWRESI_ID])
    #     cg.add(WW_MR01_component.set_max_gate_distance_number(n))
    # if gate_move_sensitivity_config := config.get(CONF_GATE_MOVE_SENSITIVITY):
    #     n = await number.new_number(
    #         gate_move_sensitivity_config, min_value=0.05, max_value=1, step=0.025
    #     )
    #     await cg.register_parented(n, config[CONF_WWRESI_ID])
    #     cg.add(WW_MR01_component.set_gate_move_sensitivity_factor_number(n))
    # if gate_still_sensitivity_config := config.get(CONF_GATE_STILL_SENSITIVITY):
    #     n = await number.new_number(
    #         gate_still_sensitivity_config, min_value=0.05, max_value=1, step=0.025
    #     )
    #     await cg.register_parented(n, config[CONF_WWRESI_ID])
    #     cg.add(WW_MR01_component.set_gate_still_sensitivity_factor_number(n))
    # if config.get(CONF_GATE_SELECT):
    #     if gate_number := config.get(CONF_GATE_SELECT):
    #         n = await number.new_number(gate_number, min_value=0, max_value=15, step=1)
    #         await cg.register_parented(n, config[CONF_WWRESI_ID])
    #         cg.add(WW_MR01_component.set_gate_select_number(n))
    #     if gate_still_threshold := config.get(CONF_STILL_THRESHOLD):
    #         n = cg.new_Pvariable(gate_still_threshold[CONF_ID])
    #         await number.register_number(
    #             n, gate_still_threshold, min_value=0, max_value=65535, step=25
    #         )
    #         await cg.register_parented(n, config[CONF_WWRESI_ID])
    #         cg.add(WW_MR01_component.set_gate_still_threshold_numbers(0, n))
    #     if gate_move_threshold := config.get(CONF_MOVE_THRESHOLD):
    #         n = cg.new_Pvariable(gate_move_threshold[CONF_ID])
    #         await number.register_number(
    #             n, gate_move_threshold, min_value=0, max_value=65535, step=25
    #         )
    #         await cg.register_parented(n, config[CONF_WWRESI_ID])
    #         cg.add(WW_MR01_component.set_gate_move_threshold_numbers(0, n))
    # else:
    #     for x in range(16):
    #         if gate_conf := config.get(f"gate_{x}"):
    #             move_config = gate_conf[CONF_MOVE_THRESHOLD]
    #             n = cg.new_Pvariable(move_config[CONF_ID], x)
    #             await number.register_number(
    #                 n, move_config, min_value=0, max_value=65535, step=25
    #             )
    #             await cg.register_parented(n, config[CONF_WWRESI_ID])
    #             cg.add(WW_MR01_component.set_gate_move_threshold_numbers(x, n))

    #             still_config = gate_conf[CONF_STILL_THRESHOLD]
    #             n = cg.new_Pvariable(still_config[CONF_ID], x)
    #             await number.register_number(
    #                 n, still_config, min_value=0, max_value=65535, step=25
    #             )
    #             await cg.register_parented(n, config[CONF_WWRESI_ID])
    #             cg.add(WW_MR01_component.set_gate_still_threshold_numbers(x, n))
