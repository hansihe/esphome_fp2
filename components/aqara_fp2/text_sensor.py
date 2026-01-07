import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_ID, CONF_TYPE

from . import CONF_FP2_ID, FP2Component, FP2Zone, aqara_fp2_ns

CONF_ZONE_ID = "zone_id"

FP2TextSensor = aqara_fp2_ns.class_(
    "FP2TextSensor", text_sensor.TextSensor, cg.Component
)

FP2TextSensorType = aqara_fp2_ns.enum("FP2TextSensorType", is_class=True)
SENSOR_TYPES = {
    "edge_label_grid": FP2TextSensorType.EDGE_LABEL_GRID,
    "entry_exit_grid": FP2TextSensorType.ENTRY_EXIT_GRID,
    "interference_grid": FP2TextSensorType.INTERFERENCE_GRID,
    "zone_map": FP2TextSensorType.ZONE_MAP,
    "mounting_position": FP2TextSensorType.MOUNTING_POSITION,
}

CONFIG_SCHEMA = cv.typed_schema(
    {
        "edge_label_grid": text_sensor.text_sensor_schema(
            FP2TextSensor,
        )
        .extend(
            {
                cv.GenerateID(CONF_FP2_ID): cv.use_id(FP2Component),
            }
        )
        .extend(cv.COMPONENT_SCHEMA),
        "entry_exit_grid": text_sensor.text_sensor_schema(
            FP2TextSensor,
        )
        .extend(
            {
                cv.GenerateID(CONF_FP2_ID): cv.use_id(FP2Component),
            }
        )
        .extend(cv.COMPONENT_SCHEMA),
        "interference_grid": text_sensor.text_sensor_schema(
            FP2TextSensor,
        )
        .extend(
            {
                cv.GenerateID(CONF_FP2_ID): cv.use_id(FP2Component),
            }
        )
        .extend(cv.COMPONENT_SCHEMA),
        "mounting_position": text_sensor.text_sensor_schema(
            FP2TextSensor,
        )
        .extend(
            {
                cv.GenerateID(CONF_FP2_ID): cv.use_id(FP2Component),
            }
        )
        .extend(cv.COMPONENT_SCHEMA),
        "zone_map": text_sensor.text_sensor_schema(
            FP2TextSensor,
        )
        .extend(
            {
                cv.GenerateID(CONF_FP2_ID): cv.use_id(FP2Component),
                cv.Required(CONF_ZONE_ID): cv.use_id(FP2Zone),
            }
        )
        .extend(cv.COMPONENT_SCHEMA),
    },
    lower=True,
)


async def to_code(config):
    var = await text_sensor.new_text_sensor(config)
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_FP2_ID])
    cg.add(var.set_parent(parent))

    sensor_type = SENSOR_TYPES[config[CONF_TYPE]]
    cg.add(var.set_sensor_type(sensor_type))

    # Link zone for zone_map type
    if config[CONF_TYPE] == "zone_map":
        zone = await cg.get_variable(config[CONF_ZONE_ID])
        cg.add(var.set_zone(zone))

    # Grid sensors link themselves to parent in C++ setup() method
