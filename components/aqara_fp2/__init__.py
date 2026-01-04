import json

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import binary_sensor, switch, uart
from esphome.components import text_sensor as text_sensor_
from esphome.const import (
    CONF_DEVICE_CLASS,
    CONF_DEVICE_ID,
    CONF_DISABLED_BY_DEFAULT,
    CONF_ENTITY_CATEGORY,
    CONF_ICON,
    CONF_ID,
    CONF_NAME,
    CONF_RESET_PIN,
    CONF_SECOND,
)
from esphome.core import CORE
from esphome.util import Registry

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["binary_sensor", "text_sensor", "switch"]

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
CONF_PRESENCE_SENSITIVITY = "presence_sensitivity"
CONF_CLOSING_SETTING = "closing_setting"
CONF_FALL_DETECTION_SENSITIVITY = "fall_detection_sensitivity"
CONF_PEOPLE_COUNTING_REPORT_ENABLE = "people_counting_report_enable"
CONF_PEOPLE_NUMBER_ENABLE = "people_number_enable"
CONF_TARGET_TYPE_ENABLE = "target_type_enable"
CONF_DWELL_TIME_ENABLE = "dwell_time_enable"
CONF_WALKING_DISTANCE_ENABLE = "walking_distance_enable"
CONF_TARGET_TRACKING = "target_tracking"
CONF_LOCATION_REPORT_SWITCH = "location_report_switch"

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
    lines = [l.strip() for l in lines if l.strip()]

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

    offset_row = 3
    offset_col = 1

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

    return list(grid_data)


def grid_to_hex_string(grid_data):
    """Convert a 40-byte grid to a compact hex string for storage."""
    return "".join(f"{b:02x}" for b in grid_data)


ZONE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(FP2Zone),
        cv.Required(CONF_GRID): parse_ascii_grid,
        cv.Optional(CONF_SENSITIVITY, default="medium"): cv.enum(SENSITIVITY_LEVELS),
    }
)


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(FP2Component),
            cv.Optional(CONF_RESET_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_MOUNTING_POSITION, default="left_corner"): cv.enum(
                MOUNTING_POSITIONS
            ),
            cv.Optional(CONF_LEFT_RIGHT_REVERSE, default=False): cv.boolean,
            cv.Optional(CONF_PRESENCE_SENSITIVITY, default="medium"): cv.enum(
                SENSITIVITY_LEVELS
            ),
            cv.Optional(CONF_CLOSING_SETTING, default=1): cv.int_range(min=1),
            cv.Optional(CONF_FALL_DETECTION_SENSITIVITY, default="low"): cv.enum(
                SENSITIVITY_LEVELS
            ),
            cv.Optional(CONF_PEOPLE_COUNTING_REPORT_ENABLE, default=False): cv.boolean,
            cv.Optional(CONF_PEOPLE_NUMBER_ENABLE, default=False): cv.boolean,
            cv.Optional(CONF_TARGET_TYPE_ENABLE, default=False): cv.boolean,
            cv.Optional(CONF_DWELL_TIME_ENABLE, default=False): cv.boolean,
            cv.Optional(CONF_WALKING_DISTANCE_ENABLE, default=False): cv.boolean,
            cv.Optional(CONF_INTERFERENCE_GRID): parse_ascii_grid,
            cv.Optional(CONF_EXIT_GRID): parse_ascii_grid,
            cv.Optional(CONF_EDGE_GRID): parse_ascii_grid,
            cv.Optional(CONF_ZONES): cv.ensure_list(ZONE_SCHEMA),
            cv.Optional(CONF_TARGET_TRACKING): text_sensor_.text_sensor_schema(),
            cv.Optional(CONF_LOCATION_REPORT_SWITCH): switch.switch_schema(
                FP2LocationSwitch
            ),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    zones = []
    if CONF_ZONES in config:
        for i, zone_conf in enumerate(config[CONF_ZONES]):
            var = cg.new_Pvariable(
                zone_conf[CONF_ID],
                i + 1,
                zone_conf[CONF_GRID],
                zone_conf[CONF_SENSITIVITY],
            )
            await cg.register_component(var, zone_conf)
            zones.append(var)

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if CONF_RESET_PIN in config:
        reset_pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
        cg.add(var.set_reset_pin(reset_pin))

    cg.add(var.set_mounting_position(config[CONF_MOUNTING_POSITION]))
    cg.add(var.set_left_right_reverse(config[CONF_LEFT_RIGHT_REVERSE]))

    cg.add(var.set_presence_sensitivity(config[CONF_PRESENCE_SENSITIVITY]))
    cg.add(var.set_closing_setting(config[CONF_CLOSING_SETTING]))
    cg.add(var.set_fall_detection_sensitivity(config[CONF_FALL_DETECTION_SENSITIVITY]))
    cg.add(
        var.set_people_counting_report_enable(
            config[CONF_PEOPLE_COUNTING_REPORT_ENABLE]
        )
    )
    cg.add(var.set_people_number_enable(config[CONF_PEOPLE_NUMBER_ENABLE]))
    cg.add(var.set_target_type_enable(config[CONF_TARGET_TYPE_ENABLE]))
    cg.add(var.set_dwell_time_enable(config[CONF_DWELL_TIME_ENABLE]))
    cg.add(var.set_walking_distance_enable(config[CONF_WALKING_DISTANCE_ENABLE]))

    if CONF_INTERFERENCE_GRID in config:
        cg.add(var.set_interference_grid(config[CONF_INTERFERENCE_GRID]))

    if CONF_EXIT_GRID in config:
        cg.add(var.set_exit_grid(config[CONF_EXIT_GRID]))

    if CONF_EDGE_GRID in config:
        cg.add(var.set_edge_grid(config[CONF_EDGE_GRID]))

    cg.add(var.set_zones(zones))

    if CONF_TARGET_TRACKING in config:
        sens = await text_sensor_.new_text_sensor(config[CONF_TARGET_TRACKING])
        cg.add(var.set_target_tracking_sensor(sens))

    if CONF_LOCATION_REPORT_SWITCH in config:
        sw = await switch.new_switch(config[CONF_LOCATION_REPORT_SWITCH])
        cg.add(var.set_location_report_switch(sw))

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
        for i, zone_conf in enumerate(config[CONF_ZONES]):
            zone_data = {
                "id": i + 1,
                "sensitivity": zone_conf[CONF_SENSITIVITY],
                "grid": grid_to_hex_string(zone_conf[CONF_GRID]),
            }
            zones_data.append(zone_data)
        map_config_data["zones"] = zones_data

    # Store as JSON string constant
    map_config_json = json.dumps(map_config_data, separators=(",", ":"))
    cg.add(var.set_map_config_json(map_config_json))
