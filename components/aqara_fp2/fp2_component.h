#pragma once

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"

#include "../aqara_fp2_accel/aqara_fp2_accel.h"

#include <ArduinoJson.h>
#include <array>
#include <cstdint>
#include <deque>
#include <vector>

namespace esphome {
namespace aqara_fp2 {

static const char *const TAG = "aqara_fp2";

// 40-byte Grid Map
using GridMap = std::array<uint8_t, 40>;

struct FP2Zone : public Component {
  FP2Zone(uint8_t zone_id, const GridMap grid, uint8_t sensitivity) : id(zone_id), grid(grid), sensitivity(sensitivity) {}

  void set_presence_sensor(binary_sensor::BinarySensor *sensor) {
    this->presence_sensor = sensor;
  }

  void set_motion_sensor(binary_sensor::BinarySensor *sensor) {
    this->motion_sensor = sensor;
  }

  void set_map_sensor(text_sensor::TextSensor *sensor) {
    this->map_sensor = sensor;
  }

  void publish_presence(bool state) {
    if (this->presence_sensor != nullptr) {
      this->presence_sensor->publish_state(state);
    }
  }

  void publish_motion(bool state) {
    if (this->motion_sensor != nullptr) {
      this->motion_sensor->publish_state(state);
    }
  }

  void publish_map(const std::string &map_hex) {
    if (this->map_sensor != nullptr) {
      this->map_sensor->publish_state(map_hex);
    }
  }

  uint8_t id;
  esphome::binary_sensor::BinarySensor *presence_sensor{nullptr};
  esphome::binary_sensor::BinarySensor *motion_sensor{nullptr};
  esphome::text_sensor::TextSensor *map_sensor{nullptr};
  GridMap grid;
  uint8_t sensitivity; // 1=Low, 2=Med, 3=High
};

class FP2Component;

enum class DataType : uint8_t {
    UINT8 = 0x00,
    UINT16 = 0x01,
    UINT32 = 0x02,
    VOID = 0x03,
    BOOL = 0x04,
    STRING = 0x05,
    BINARY = 0x06,
};

enum class OpCode : uint8_t {
  // Device -> Host: Standard Response to Read (Values).
  // Device -> Host: Reverse Read Request (SubID only, len=2).
  RESPONSE = 0x01,

  // Host -> Device: Write Attribute (Values).
  WRITE = 0x02,

  // Both: Acknowledge.
  ACK = 0x03,

  // Host -> Device: Standard Read Request (SubID only).
  // Host -> Device: Reverse Read Response (Values, in response to 0x01 Query).
  READ = 0x04,

  // Device -> Host: Async Report.
  REPORT = 0x05,
};

enum class AttrId : uint16_t {
    RADAR_SW_VERSION                = 0x0102,
    WORK_MODE                       = 0x0116,
    MONITOR_MODE                    = 0x0105, // Detection direction (0=default, 1=L/R)
    LEFT_RIGHT_REVERSE              = 0x0122, // L/R swap (0/1/2)
    PRESENCE_DETECT_SENSITIVITY     = 0x0111, // Sensitivity (1-3)
    CLOSING_SETTING                 = 0x0106, // Proximity (0=far, 1=med, 2=close)
    ZONE_CLOSE_AWAY_ENABLE          = 0x0153, // Zone N close/away enable
    FALL_SENSITIVITY                = 0x0123, // Fall sensitivity
    PEOPLE_COUNT_REPORT_ENABLE      = 0x0158, // People counting enable
    PEOPLE_NUMBER_ENABLE            = 0x0162, // People number enable
    TARGET_TYPE_ENABLE              = 0x0163, // AI person detection
    SLEEP_MOUNT_POSITION            = 0x0168, // Sleep mount position
    SLEEP_ZONE_SIZE                 = 0x0169, // Sleep zone dimensions
    WALL_CORNER_POS                 = 0x0170, // Wall/corner position
    DWELL_TIME_ENABLE               = 0x0172, // Dwell tracking
    WALK_DISTANCE_ENABLE            = 0x0173, // Walking distance
    INTERFERENCE_MAP                = 0x0110, // Interference map (40B)
    ENTRY_EXIT_MAP                  = 0x0109, // Enter/exit zones (40B)
    EDGE_MAP                        = 0x0107, // Detection boundary (40B)
    ZONE_MAP                        = 0x0114, // Zone N area map (1B ID + 40B)
    ZONE_SENSITIVITY                = 0x0151, // Zone N sensitivity
    ZONE_ACTIVATION_LIST            = 0x0202, // Auxiliary config (32B)
    DETECT_ZONE_TYPE                = 0x0152, // Zone N type
    DEVICE_DIRECTION                = 0x0143,
    ANGLE_SENSOR_DATA               = 0x0120,
    LOCATION_REPORT_ENABLE          = 0x0112,
    ZONE_PRESENCE                   = 0x0142,
    LOCATION_TRACKING_DATA          = 0x0117,
    THERMO_EN                       = 0x0138,
    THERMO_DATA                     = 0x0141,
    TEMPERATURE                     = 0x0128,
    DETECT_ZONE_MOTION              = 0x0115,
    MOTION_DETECT                   = 0x0103,
    PRESENCE_DETECT                 = 0x0104,
    ONTIME_PEOPLE_NUMBER            = 0x0165,
    INVALID                         = 0xFFFF,
};

struct FP2Command {
  OpCode type;
  AttrId attr_id;
  std::vector<uint8_t> data;
  uint32_t last_send_time;
  uint8_t retry_count;
};

class FP2LocationSwitch : public switch_::Switch {
public:
  void set_parent(FP2Component *parent) { parent_ = parent; }

protected:
  void write_state(bool state) override;
  FP2Component *parent_{nullptr};
};

class FP2Component : public Component, public uart::UARTDevice {
public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  // Configuration setters
  void set_radar_reset_pin(GPIOPin *pin) { reset_pin_ = pin; }
  void set_mounting_position(uint8_t pos) { mounting_position_ = pos; }
  void set_left_right_reverse(bool val) { left_right_reverse_ = val; }

  void set_fall_detection_sensitivity(uint8_t val) {
    fall_detection_sensitivity_ = val;
  }

  void set_interference_grid(const std::vector<uint8_t> &grid);
  void set_exit_grid(const std::vector<uint8_t> &grid);
  void set_edge_grid(const std::vector<uint8_t> &grid);

  void set_presence_sensitivity(uint8_t val) { global_presence_sensitivity_ = val; }
  void set_motion_sensor(binary_sensor::BinarySensor *sensor) { global_motion_sensor_ = sensor; }
  void set_presence_sensor(binary_sensor::BinarySensor *sensor) { global_presence_sensor_ = sensor; }

  //void add_zone(uint8_t id, binary_sensor::BinarySensor *sens,
  //              const std::vector<uint8_t> &grid, uint8_t sensitivity);
  void set_zones(const std::vector<FP2Zone*> &zones);

  void set_target_tracking_sensor(text_sensor::TextSensor *sensor) {
    target_tracking_sensor_ = sensor;
  }
  void set_location_report_switch(FP2LocationSwitch *sw) {
    location_report_switch_ = sw;
    sw->set_parent(this);
  }

  void set_edge_label_grid_sensor(text_sensor::TextSensor *sensor) {
    ESP_LOGI(TAG, "set_edge_label_grid_sensor called (has_edge_grid_=%d)", has_edge_grid_);
    edge_label_grid_sensor_ = sensor;
    if (has_edge_grid_ && edge_label_grid_sensor_ != nullptr) {
      ESP_LOGI(TAG, "Publishing edge label grid from setter");
      edge_label_grid_sensor_->publish_state(grid_to_hex_card_format(edge_grid_));
    } else {
      ESP_LOGW(TAG, "NOT publishing edge label grid from setter (has_grid=%d, sensor=%p)", has_edge_grid_, edge_label_grid_sensor_);
    }
  }
  void set_entry_exit_grid_sensor(text_sensor::TextSensor *sensor) {
    ESP_LOGI(TAG, "set_entry_exit_grid_sensor called (has_exit_grid_=%d)", has_exit_grid_);
    entry_exit_grid_sensor_ = sensor;
    if (has_exit_grid_ && entry_exit_grid_sensor_ != nullptr) {
      ESP_LOGI(TAG, "Publishing entry/exit grid from setter");
      entry_exit_grid_sensor_->publish_state(grid_to_hex_card_format(exit_grid_));
    } else {
      ESP_LOGW(TAG, "NOT publishing entry/exit grid from setter (has_grid=%d, sensor=%p)", has_exit_grid_, entry_exit_grid_sensor_);
    }
  }
  void set_interference_grid_sensor(text_sensor::TextSensor *sensor) {
    ESP_LOGI(TAG, "set_interference_grid_sensor called (has_interference_grid_=%d)", has_interference_grid_);
    interference_grid_sensor_ = sensor;
    if (has_interference_grid_ && interference_grid_sensor_ != nullptr) {
      ESP_LOGI(TAG, "Publishing interference grid from setter");
      interference_grid_sensor_->publish_state(grid_to_hex_card_format(interference_grid_));
    } else {
      ESP_LOGW(TAG, "NOT publishing interference grid from setter (has_grid=%d, sensor=%p)", has_interference_grid_, interference_grid_sensor_);
    }
  }
  void set_mounting_position_sensor(text_sensor::TextSensor *sensor) {
    mounting_position_sensor_ = sensor;
    if (mounting_position_sensor_ != nullptr) {
      const char* pos_str;
      switch (mounting_position_) {
        case 0x02: pos_str = "left_upper_corner"; break;
        case 0x03: pos_str = "right_upper_corner"; break;
        default: pos_str = "wall"; break;
      }
      mounting_position_sensor_->publish_state(pos_str);
    }
  }
  void set_radar_temperature_sensor(sensor::Sensor *sensor) {
      radar_temperature_sensor_ = sensor;
  }
  void set_radar_software_sensor(text_sensor::TextSensor *sensor) {
      radar_software_sensor_ = sensor;
  }

  void set_fp2_accel(aqara_fp2_accel::AqaraFP2Accel *accel) {
      fp2_accel_ = accel;
  }

  void set_location_reporting_enabled(bool enabled);

  // Grid format conversion
  std::string grid_to_hex_card_format(const GridMap &grid);

  // Map configuration
  void set_map_config_json(const std::string &json) { map_config_json_ = json; }
  JsonDocument get_map_config_json();
  void json_get_map_data(JsonObject root);

protected:
  // Internal logic
  void process_command_queue_();
  void send_next_command_();
  void handle_incoming_byte_(uint8_t byte);
  const char* get_mounting_position_string_();
  void handle_parsed_frame_(uint8_t type, AttrId attr_id,
                            const std::vector<uint8_t> &payload);
  void handle_ack_(AttrId attr_id);
  void handle_report_(AttrId attr_id, const std::vector<uint8_t> &payload);
  void handle_location_tracking_report_(const std::vector<uint8_t> &payload);
  void handle_temperature_report_(const std::vector<uint8_t> &payload);
  void handle_response_(AttrId attr_id, const std::vector<uint8_t> &payload);
  void handle_reverse_read_request_(AttrId attr_id);
  void send_ack_(AttrId attr_id);

  // Initialization
  void perform_reset_();
  void check_initialization_();

  aqara_fp2_accel::AqaraFP2Accel *fp2_accel_{nullptr};

  GPIOPin *reset_pin_{nullptr};
  bool init_done_{false};
  uint32_t last_heartbeat_millis_{0};

  // Configuration State
  uint8_t mounting_position_{0x01}; // Default Wall
  bool left_right_reverse_{false};
  uint8_t fall_detection_sensitivity_{1};

  // Grids (Optional)
  GridMap interference_grid_{};
  bool has_interference_grid_{false};
  GridMap exit_grid_{};
  bool has_exit_grid_{false};
  GridMap edge_grid_{};
  bool has_edge_grid_{false};

  // Global zone
  uint8_t global_presence_sensitivity_{2}; // Default Medium
  binary_sensor::BinarySensor *global_presence_sensor_{nullptr};
  binary_sensor::BinarySensor *global_motion_sensor_{nullptr};

  // Zones
  std::vector<FP2Zone*> zones_;
  text_sensor::TextSensor *target_tracking_sensor_{nullptr};
  FP2LocationSwitch *location_report_switch_{nullptr};
  bool location_reporting_active_{false};

  // Grid text sensors
  text_sensor::TextSensor *edge_label_grid_sensor_{nullptr};
  text_sensor::TextSensor *entry_exit_grid_sensor_{nullptr};
  text_sensor::TextSensor *interference_grid_sensor_{nullptr};
  text_sensor::TextSensor *mounting_position_sensor_{nullptr};

  sensor::Sensor *radar_temperature_sensor_{nullptr};
  text_sensor::TextSensor *radar_software_sensor_{nullptr};

  // Map Configuration (compile-time generated)
  std::string map_config_json_;

  // Communication State
  std::deque<FP2Command> command_queue_;

  // Frame Decoder State
  enum DecoderState {
    SYNC,
    VER_H,
    VER_L,
    SEQ,
    OPCODE,
    LEN_H,
    LEN_L,
    H_CHECK,
    PAYLOAD,
    CRC_L,
    CRC_H
  } state_{SYNC};

  uint8_t rx_seq_;
  uint8_t rx_opcode_;
  uint16_t rx_len_;
  std::vector<uint8_t> rx_payload_;
  uint16_t rx_crc_;

  // Rolling checksum for header
  uint16_t header_sum_{0};

  // Ack Manager
  // We track the SubID of tAttrId::INVALID command we are currently waiting for an ACK for.
  // 0xFFFF = Not waiting.
  AttrId waiting_for_ack_attr_id_{AttrId::INVALID};
  uint32_t last_command_sent_millis_{0};
  static const uint32_t ACK_TIMEOUT_MS = 500;
  static const uint8_t MAX_RETRIES = 3;

  void enqueue_command_(OpCode type, AttrId attr_id, uint8_t byte_val);
  void enqueue_command_(OpCode type, AttrId attr_id, uint16_t word_val);
  void enqueue_command_(OpCode type, AttrId attr_id, bool bool_val);
  void enqueue_command_blob2_(AttrId attr_id,
                              const std::vector<uint8_t> &blob_content);
  void enqueue_read_(AttrId attr_id);
  void send_reverse_response_(AttrId attr_id, uint8_t byte_val);
};

} // namespace aqara_fp2
} // namespace esphome
