import json

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import binary_sensor, sensor, switch, uart
from esphome.components import text_sensor as text_sensor_
from esphome.const import (
    CONF_DEVICE_CLASS,
    CONF_DEVICE_ID,
    CONF_DISABLED_BY_DEFAULT,
    CONF_ENTITY_CATEGORY,
    CONF_ICON,
    CONF_ID,
    CONF_NAME,
    CONF_SECOND,
    CONF_MOTION,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_OCCUPANCY,
    DEVICE_CLASS_MOTION,
    STATE_CLASS_MEASUREMENT,
    ENTITY_CATEGORY_DIAGNOSTIC,
    UNIT_CELSIUS,
    ICON_THERMOMETER,
    ICON_MOTION_SENSOR,
)
from esphome.core import CORE
from esphome.util import Registry

from ..aqara_fp2_accel import AqaraFP2Accel

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["binary_sensor", "text_sensor", "sensor", "switch", "json"]

aqara_fp2_ns = cg.esphome_ns.namespace("aqara_fp2")
FP2Component = aqara_fp2_ns.class_("FP2Component", cg.Component, uart.UARTDevice)
FP2LocationSwitch = aqara_fp2_ns.class_("FP2LocationSwitch", switch.Switch)
FP2Zone = aqara_fp2_ns.class_("FP2Zone", cg.Component)

CONF_FP2_ID = "fp2_id"

CONF_MOUNTING_POSITION = "mounting_position"
CONF_LEFT_RIGHT_REVERSE = "left_right_reverse"
CONF_INTERFERENCE_GRID = "interference_grid"
CONF_EXIT_GRID = "exit_grid"
CONF_EDGE_GRID = "edge_grid"
CONF_ZONES = "zones"
CONF_GRID = "grid"
CONF_SENSITIVITY = "sensitivity"

# New Options
CONF_RADAR_RESET_PIN = "radar_reset_pin"
CONF_PRESENCE_SENSITIVITY = "presence_sensitivity"
CONF_FALL_DETECTION_SENSITIVITY = "fall_detection_sensitivity"
CONF_PEOPLE_COUNTING_REPORT_ENABLE = "people_counting_report_enable"
CONF_PEOPLE_NUMBER_ENABLE = "people_number_enable"
CONF_TARGET_TYPE_ENABLE = "target_type_enable"
CONF_DWELL_TIME_ENABLE = "dwell_time_enable"
CONF_WALKING_DISTANCE_ENABLE = "walking_distance_enable"
CONF_TARGET_TRACKING = "target_tracking"
CONF_LOCATION_REPORT_SWITCH = "location_report_switch"
CONF_RADAR_TEMPERATURE = "radar_temperature"
CONF_PRESENCE = "presence"
CONF_GLOBAL_ZONE = "global_zone"
CONF_RADAR_SOFTWARE_VERSION = "radar_software_version"

MOUNTING_POSITIONS = {
    "wall": 0x01,
    "left_corner": 0x02,
    "right_corner": 0x03,
}

SENSITIVITY_LEVELS = {
    "low": 1,
    "medium": 2,
    "high": 3,
}


def parse_ascii_grid(value):
    """
    Parses a 14x14 ASCII grid into a 40-byte (320-bit) protocol blob.
    Protocol Grid: 20 rows x 16 cols.
    Active Area: Centered 14x14 (Rows 3-16, Cols 1-14).

    Chars: 'x', 'X' = Active. '.', ' ' = Inactive.
    """
    lines = value.strip().splitlines()
    # Filter out empty lines or comments if needed, but strict 14 lines is better for now
    lines = [li.strip() for li in lines if li.strip()]

    if len(lines) != 14:
        raise cv.Invalid(f"Grid must have exactly 14 rows, got {len(lines)}")

    for i, line in enumerate(lines):
        # Remove whitespace
        clean_line = line.replace(" ", "")
        if len(clean_line) != 14:
            raise cv.Invalid(
                f"Row {i + 1} must have 14 characters (excluding spaces), got {len(clean_line)}: '{clean_line}'"
            )

    # Initialize 20x16 grid (320 bits -> 40 bytes)
    # 20 rows * 16 cols
    grid_data = bytearray(40)

    # Map 14x14 input to 20x16 output
    # Input Row 0 -> Output Row 3
    # Input Col 0 -> Output Col 1

    offset_row = 0
    offset_col = 2

    for r in range(14):
        line = lines[r].replace(" ", "")
        out_r = r + offset_row

        # In the protocol:
        # Each row is 2 bytes (16 bits) Big Endian.
        # byte[2*r] is High Byte (Cols 0-7)
        # byte[2*r + 1] is Low Byte (Cols 8-15)
        # Bit 15 = Col 0 ... Bit 0 = Col 15

        row_val = 0

        for c in range(14):
            char = line[c]
            if char in ("x", "X"):
                out_c = c + offset_col
                # Set bit at out_c
                # Standard convention: MSB is index 0.
                # So Col 0 is 1 << 15
                bit_mask = 1 << (15 - out_c)
                row_val |= bit_mask

        # Write row_val to buffer (Big Endian)
        grid_data[out_r * 2] = (row_val >> 8) & 0xFF
        grid_data[out_r * 2 + 1] = row_val & 0xFF

    gd = list(grid_data)
    return gd


def grid_to_hex_string(grid_data):
    """Convert a 40-byte grid to a compact hex string for storage."""
    return "".join(f"{b:02x}" for b in grid_data)

ZONE_BASE_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_PRESENCE_SENSITIVITY, default="medium"): cv.enum(SENSITIVITY_LEVELS),
        cv.Optional(CONF_PRESENCE): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_OCCUPANCY,
            filters=[{"settle": cv.TimePeriod(milliseconds=1000)}],
        ),
        cv.Optional(CONF_MOTION): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_MOTION,
            icon=ICON_MOTION_SENSOR,
        ),
    }
)

ZONE_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(CONF_ID): cv.declare_id(FP2Zone),
            cv.Required(CONF_GRID): parse_ascii_grid,
            cv.Optional("zone_map_sensor"): text_sensor_.text_sensor_schema(entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
        }
    ).extend(ZONE_BASE_SCHEMA)
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(FP2Component),
            cv.Required("accel"): cv.use_id(AqaraFP2Accel),

            cv.Optional(CONF_RADAR_RESET_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_MOUNTING_POSITION, default="left_corner"): cv.enum(
                MOUNTING_POSITIONS
            ),

            cv.Optional(CONF_LEFT_RIGHT_REVERSE, default=False): cv.boolean,
            cv.Optional(CONF_INTERFERENCE_GRID): parse_ascii_grid,
            cv.Optional(CONF_EXIT_GRID): parse_ascii_grid,
            cv.Optional(CONF_EDGE_GRID): parse_ascii_grid,

            cv.Optional(CONF_TARGET_TRACKING): text_sensor_.text_sensor_schema(entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
            cv.Optional(CONF_LOCATION_REPORT_SWITCH): switch.switch_schema(
                FP2LocationSwitch
            ),

            cv.Optional("edge_label_grid_sensor"): text_sensor_.text_sensor_schema(entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
            cv.Optional("entry_exit_grid_sensor"): text_sensor_.text_sensor_schema(entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
            cv.Optional("interference_grid_sensor"): text_sensor_.text_sensor_schema(entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
            cv.Optional("mounting_position_sensor"): text_sensor_.text_sensor_schema(entity_category=ENTITY_CATEGORY_DIAGNOSTIC),

            cv.Optional(CONF_GLOBAL_ZONE): ZONE_BASE_SCHEMA,
            cv.Optional(CONF_ZONES): cv.ensure_list(ZONE_SCHEMA),

            cv.Optional(CONF_RADAR_SOFTWARE_VERSION): text_sensor_.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_RADAR_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)

SENSOR_MAP = {
    CONF_RADAR_TEMPERATURE: (sensor.new_sensor, "set_radar_temperature_sensor"),
    CONF_RADAR_SOFTWARE_VERSION: (text_sensor_.new_text_sensor, "set_radar_software_sensor"),
    CONF_LOCATION_REPORT_SWITCH: (switch.new_switch, "set_location_report_switch"),
    CONF_TARGET_TRACKING: (text_sensor_.new_text_sensor, "set_target_tracking_sensor"),

    # Text config sensors
    "edge_label_grid_sensor": (text_sensor_.new_text_sensor, "set_edge_label_grid_sensor"),
    "entry_exit_grid_sensor": (text_sensor_.new_text_sensor, "set_entry_exit_grid_sensor"),
    "interference_grid_sensor": (text_sensor_.new_text_sensor, "set_interference_grid_sensor"),
    "mounting_position_sensor": (text_sensor_.new_text_sensor, "set_mounting_position_sensor"),
}

ZONE_SENSOR_MAP = {
    CONF_PRESENCE: (binary_sensor.new_binary_sensor, "set_presence_sensor"),
    CONF_MOTION: (binary_sensor.new_binary_sensor, "set_motion_sensor"),

    # Text config sensors
    "zone_map_sensor": (text_sensor_.new_text_sensor, "set_map_sensor"),
}

async def to_code(config):
    zones = []
    if CONF_ZONES in config:
        for i, zone_conf in enumerate(config[CONF_ZONES]):
            var = cg.new_Pvariable(
                zone_conf[CONF_ID],
                i + 1,
                zone_conf[CONF_GRID],
                zone_conf[CONF_PRESENCE_SENSITIVITY],
            )
            await cg.register_component(var, zone_conf)

            # Create sensors if provided
            for key, (new, funcName) in ZONE_SENSOR_MAP.items():
                if key in zone_conf:
                    sens = await new(zone_conf[key])
                    cg.add(getattr(var, funcName)(sens))

            zones.append(var)

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if CONF_RADAR_RESET_PIN in config:
        reset_pin = await cg.gpio_pin_expression(config[CONF_RADAR_RESET_PIN])
        cg.add(var.set_radar_reset_pin(reset_pin))

    cg.add(var.set_mounting_position(config[CONF_MOUNTING_POSITION]))
    cg.add(var.set_left_right_reverse(config[CONF_LEFT_RIGHT_REVERSE]))

    if CONF_GLOBAL_ZONE in config:
        global_zone_conf = config[CONF_GLOBAL_ZONE]

        cg.add(var.set_presence_sensitivity(global_zone_conf[CONF_PRESENCE_SENSITIVITY]))

        for key, (new, funcName) in ZONE_SENSOR_MAP.items():
            if key in global_zone_conf:
                sens = await new(global_zone_conf[key])
                cg.add(getattr(var, funcName)(sens))


    if CONF_INTERFERENCE_GRID in config:
        cg.add(var.set_interference_grid(config[CONF_INTERFERENCE_GRID]))

    if CONF_EXIT_GRID in config:
        cg.add(var.set_exit_grid(config[CONF_EXIT_GRID]))

    if CONF_EDGE_GRID in config:
        cg.add(var.set_edge_grid(config[CONF_EDGE_GRID]))

    cg.add(var.set_zones(zones))

    for key, (new, funcName) in SENSOR_MAP.items():
        if key in config:
            sens = await new(config[key])
            cg.add(getattr(var, funcName)(sens))

    # Generate map config JSON data at compile time
    map_config_data = {
        "mounting_position": config[CONF_MOUNTING_POSITION],
        "left_right_reverse": config[CONF_LEFT_RIGHT_REVERSE],
    }

    # Add grids if present
    if CONF_INTERFERENCE_GRID in config:
        map_config_data["interference_grid"] = grid_to_hex_string(
            config[CONF_INTERFERENCE_GRID]
        )

    if CONF_EXIT_GRID in config:
        map_config_data["exit_grid"] = grid_to_hex_string(config[CONF_EXIT_GRID])

    if CONF_EDGE_GRID in config:
        map_config_data["edge_grid"] = grid_to_hex_string(config[CONF_EDGE_GRID])

    # Add zones
    if CONF_ZONES in config:
        zones_data = []
        for zone_conf in config[CONF_ZONES]:
            zone_data = {
                "sensitivity": zone_conf[CONF_PRESENCE_SENSITIVITY],
                "grid": grid_to_hex_string(zone_conf[CONF_GRID]),
            }
            zones_data.append(zone_data)
        map_config_data["zones"] = zones_data

    # Store as JSON string constant
    map_config_json = json.dumps(map_config_data, separators=(",", ":"))
    cg.add(var.set_map_config_json(map_config_json))

    accel = await cg.get_variable(config["accel"])
    cg.add(var.set_fp2_accel(accel))
