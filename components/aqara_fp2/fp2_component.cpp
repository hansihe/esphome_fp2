#include "fp2_component.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include <cstdint>
#include <vector>

namespace esphome {
namespace aqara_fp2 {

// CRC16-MODBUS
static uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void FP2Component::setup() {
  ESP_LOGI(TAG, "Setting up Aqara FP2...");

  // Reset internal state
  waiting_for_ack_sub_id_ = 0xFFFF;
  init_done_ = false;

  // GPIO Reset
  perform_reset_();
}

void FP2Component::perform_reset_() {
  if (reset_pin_ != nullptr) {
    ESP_LOGI(TAG, "Performing Hardware Reset via Pin...");
    reset_pin_->setup();
    reset_pin_->digital_write(false);
    delay(100);
    reset_pin_->digital_write(true);
    ESP_LOGI(TAG, "Hardware Reset Done. Waiting for heartbeat...");
  } else {
    ESP_LOGI(TAG, "No Reset Pin configured. Waiting for heartbeat...");
  }

  if (this->location_report_switch_ != nullptr) {
    // Default to OFF for internal state until we know better, or optimistic?
    // Device usually starts with it OFF.
    this->location_report_switch_->publish_state(false);
  }
}

void FP2Component::set_location_reporting_enabled(bool enabled) {
  this->location_reporting_active_ = enabled;
  this->enqueue_command_(OpCode::WRITE, 0x0112, (uint8_t)(enabled ? 1 : 0));
  if (!enabled && this->target_tracking_sensor_ != nullptr) {
    this->target_tracking_sensor_->publish_state("[]");
  }
}

void FP2LocationSwitch::write_state(bool state) {
  if (this->parent_ != nullptr) {
    this->parent_->set_location_reporting_enabled(state);
  }
  this->publish_state(state);
}

void FP2Component::loop() {
  while (available()) {
    uint8_t byte;
    read_byte(&byte);
    handle_incoming_byte_(byte);
  }

  check_initialization_();
  process_command_queue_();
}

void FP2Component::check_initialization_() {
  if (init_done_)
    return;

  // We rely on handle_parsed_frame_ to set a flag or we check
  // last_heartbeat_millis_
  if (last_heartbeat_millis_ > 0) {
    ESP_LOGI(TAG, "Heartbeat received. Starting initialization sequence...");
    init_done_ = true;

    // 1. Basic Settings
    enqueue_command_(OpCode::WRITE, 0x0105, (uint8_t) 0);
    enqueue_command_(OpCode::WRITE, 0x0122,
                     (uint8_t)(left_right_reverse_ ? 2 : 0));
    enqueue_command_(OpCode::WRITE, 0x0111, presence_sensitivity_);
    enqueue_command_(OpCode::WRITE, 0x0106, closing_setting_);
    enqueue_command_(OpCode::WRITE, 0x0153, (uint16_t) 0x0001);
    enqueue_command_(OpCode::WRITE, 0x0123, fall_detection_sensitivity_);
    enqueue_command_(OpCode::WRITE, 0x0158, people_counting_report_enable_); // BOOL
    enqueue_command_(OpCode::WRITE, 0x0162, people_number_enable_); // BOOL
    enqueue_command_(OpCode::WRITE, 0x0163, target_type_enable_); // BOOL
    enqueue_command_(OpCode::WRITE, 0x0168, (uint8_t) 0); // sleep zone mount pos
    enqueue_command_(OpCode::WRITE, 0x0170, mounting_position_);
    enqueue_command_(OpCode::WRITE, 0x0172, (uint8_t) 0); // dwell time enable
    enqueue_command_(OpCode::WRITE, 0x0173, (uint8_t) 0); // walking distance enable

    // 2. Grids
    if (has_interference_grid_) {
      // 0x0110 Interference Source
      enqueue_command_blob2_(0x0110,
                             std::vector<uint8_t>(interference_grid_.begin(),
                                                  interference_grid_.end()));
    }
    if (has_exit_grid_) {
      // 0x0109 Enter/Exit Label
      enqueue_command_blob2_(
          0x0109, std::vector<uint8_t>(exit_grid_.begin(), exit_grid_.end()));
    }
    if (has_edge_grid_) {
      // 0x0107 Edge Label
      enqueue_command_blob2_(
          0x0107, std::vector<uint8_t>(edge_grid_.begin(), edge_grid_.end()));
    }

    // 3. Zones
    std::vector<uint8_t> activations(32, 0);
    for (const auto &zone : zones_) {
      // a. Send Zone Detect Setting (0x0114)
      // Structure: [ZoneID] [40 byte Map]
      std::vector<uint8_t> payload;
      payload.push_back(zone->id);
      payload.insert(payload.end(), zone->grid.begin(), zone->grid.end());
      enqueue_command_blob2_(0x0114, payload);

      // b. Send Sensitivity (0x0151)
      // Structure: UINT16 (High=ID, Low=Sens)
      uint16_t sens_val = (zone->id << 8) | (zone->sensitivity & 0xFF);
      enqueue_command_(OpCode::WRITE, 0x0151, sens_val);

      activations[zone->id] = zone->id;
    }

    enqueue_command_blob2_(0x0202, activations);

    for (const auto &zone : zones_) {
        // Close/Away Enable default?
        // Trace: 0x0153 Zone Close Away Enable.
        // We can enable it by default for now or add config options later.
        enqueue_command_(OpCode::WRITE, 0x0153, (uint16_t)((zone->id << 8) | 1));
    }

    // 5. Publish grid sensors once initialization completes
    ESP_LOGI(TAG, "Publishing grid sensors: has_edge=%d edge_sensor=%p has_exit=%d exit_sensor=%p has_interference=%d interference_sensor=%p",
             has_edge_grid_, edge_label_grid_sensor_, has_exit_grid_, entry_exit_grid_sensor_, has_interference_grid_, interference_grid_sensor_);

    if (has_edge_grid_ && edge_label_grid_sensor_ != nullptr) {
      ESP_LOGI(TAG, "Publishing edge label grid");
      edge_label_grid_sensor_->publish_state(grid_to_hex_card_format(edge_grid_));
    } else {
      ESP_LOGW(TAG, "NOT publishing edge label grid (has_grid=%d, sensor=%p)", has_edge_grid_, edge_label_grid_sensor_);
    }

    if (has_exit_grid_ && entry_exit_grid_sensor_ != nullptr) {
      ESP_LOGI(TAG, "Publishing entry/exit grid");
      entry_exit_grid_sensor_->publish_state(grid_to_hex_card_format(exit_grid_));
    } else {
      ESP_LOGW(TAG, "NOT publishing entry/exit grid (has_grid=%d, sensor=%p)", has_exit_grid_, entry_exit_grid_sensor_);
    }

    if (has_interference_grid_ && interference_grid_sensor_ != nullptr) {
      ESP_LOGI(TAG, "Publishing interference grid");
      interference_grid_sensor_->publish_state(grid_to_hex_card_format(interference_grid_));
    } else {
      ESP_LOGW(TAG, "NOT publishing interference grid (has_grid=%d, sensor=%p)", has_interference_grid_, interference_grid_sensor_);
    }

    // 6. Publish zone map sensors
    for (const auto &zone : zones_) {
      if (zone->map_sensor != nullptr) {
        zone->map_sensor->publish_state(grid_to_hex_card_format(zone->grid));
      }
    }
  }
}

void FP2Component::process_command_queue_() {
  uint32_t now = millis();

  // If waiting for ACK
  if (waiting_for_ack_sub_id_ != 0xFFFF) {
    if (now - last_command_sent_millis_ > ACK_TIMEOUT_MS) {
      // Timeout
      if (!command_queue_.empty()) {
        auto &cmd = command_queue_.front();
        cmd.retry_count++;
        if (cmd.retry_count >= MAX_RETRIES) {
          ESP_LOGW(TAG, "Command 0x%04X timed out after %d retries. Dropping.",
                   cmd.sub_id, MAX_RETRIES);
          command_queue_.pop_front();
          waiting_for_ack_sub_id_ = 0xFFFF;
        } else {
          ESP_LOGW(TAG, "Command 0x%04X timed out. Retrying (%d/%d)...",
                   cmd.sub_id, cmd.retry_count, MAX_RETRIES);
          // Resend handled by send_next_command_ logic once waiting state calls
          // reset? Actually, we should just resend immediately
          send_next_command_();
        }
      } else {
        // Queue empty but waiting state mismatch?
        waiting_for_ack_sub_id_ = 0xFFFF;
      }
    }
    return; // Still waiting
  }

  // Not waiting, send next
  if (!command_queue_.empty()) {
    send_next_command_();
  }
}

void FP2Component::send_next_command_() {
  if (command_queue_.empty())
    return;

  auto &cmd = command_queue_.front();
  static uint8_t next_tx_seq = 0;

  // Build frame: [Sync][Ver][Ver][Seq][Op][Len][Len][Check][Payload][CRC][CRC]
  std::vector<uint8_t> frame;
  frame.push_back(0x55);  // Sync
  frame.push_back(0x00);  // Version High
  frame.push_back(0x01);  // Version Low
  frame.push_back(next_tx_seq++);
  frame.push_back((uint8_t)cmd.type);

  uint16_t len = cmd.data.size();
  frame.push_back((len >> 8) & 0xFF);
  frame.push_back(len & 0xFF);

  // Header checksum: NOT((Sum(bytes 0-6) - 1))
  uint8_t sum = 0;
  for (int i = 0; i < 7; i++)
    sum += frame[i];
  frame.push_back((uint8_t)(~((sum - 1))));

  // Append payload
  frame.insert(frame.end(), cmd.data.begin(), cmd.data.end());

  // Append CRC16 (Little Endian)
  uint16_t crc = crc16(frame.data(), frame.size());
  frame.push_back(crc & 0xFF);
  frame.push_back((crc >> 8) & 0xFF);

  write_array(frame);
  last_command_sent_millis_ = millis();

  // Only WRITE commands expect an ACK from the radar
  // ACK and Reverse Read Response packets don't get ACKed
  if (cmd.type == OpCode::WRITE) {
    waiting_for_ack_sub_id_ = cmd.sub_id;
  } else {
    command_queue_.pop_front();
  }
}

void FP2Component::send_ack_(uint16_t sub_id) {
  FP2Command cmd;
  cmd.type = OpCode::ACK;
  cmd.sub_id = sub_id;
  cmd.retry_count = 0;
  cmd.last_send_time = 0;

  // ACK payload: [SubID 2 bytes] [DataType VOID]
  cmd.data.push_back((sub_id >> 8) & 0xFF);
  cmd.data.push_back(sub_id & 0xFF);
  cmd.data.push_back(0x03);  // DataType: VOID

  // ACKs are high priority - push to front of queue
  command_queue_.push_front(cmd);
}

void FP2Component::send_reverse_response_(uint16_t sub_id, uint8_t byte_val) {
  FP2Command cmd;
  cmd.type = OpCode::READ;  // Reverse Read Response uses READ opcode
  cmd.sub_id = sub_id;
  cmd.retry_count = 0;

  // Payload: [SubID 2 bytes] [DataType UINT8] [Value 1 byte]
  cmd.data.push_back((sub_id >> 8) & 0xFF);
  cmd.data.push_back(sub_id & 0xFF);
  cmd.data.push_back(0x00);  // DataType: UINT8
  cmd.data.push_back(byte_val);

  command_queue_.push_back(cmd);
}

void FP2Component::handle_incoming_byte_(uint8_t byte) {
  if (state_ == SYNC) {
    if (byte == 0x55) {
      state_ = VER_H;
      rx_payload_.clear();
      header_sum_ = byte; // Start sum
    }
    return;
  }

  // Update sum for header fields (0..6)
  if (state_ < H_CHECK) {
    header_sum_ += byte;
  }

  switch (state_) {
  case VER_H:
    state_ = (byte == 0x00) ? VER_L : SYNC;
    break;
  case VER_L:
    state_ = (byte == 0x01) ? SEQ : SYNC;
    break;
  case SEQ:
    rx_seq_ = byte;
    state_ = OPCODE;
    break;
  case OPCODE:
    rx_opcode_ = byte;
    state_ = LEN_H;
    break;
  case LEN_H:
    rx_len_ = byte << 8;
    state_ = LEN_L;
    break;
  case LEN_L:
    rx_len_ |= byte;
    state_ = H_CHECK;
    break;

  case H_CHECK: {
    uint8_t expected = (uint8_t)(~((header_sum_ - 1)));
    if (byte != expected) {
      ESP_LOGW(TAG, "Header Checksum Fail: Exp %02X, Got %02X", expected, byte);
      state_ = SYNC;
    } else {
      if (rx_len_ > 4096) { // Sanity check, increased for potential BLOBs
        state_ = SYNC;
      } else if (rx_len_ == 0) {
        state_ = CRC_L;
      } else {
        state_ = PAYLOAD;
      }
    }
    break;
  }

  case PAYLOAD:
    rx_payload_.push_back(byte);
    if (rx_payload_.size() == rx_len_) {
      state_ = CRC_L;
    }
    break;

  case CRC_L:
    rx_crc_ = byte;
    state_ = CRC_H;
    break;

  case CRC_H: {
    rx_crc_ |= (byte << 8);
    // Validate CRC
    std::vector<uint8_t> frame;
    frame.push_back(0x55);
    frame.push_back(0x00);
    frame.push_back(0x01);
    frame.push_back(rx_seq_);
    frame.push_back(rx_opcode_);
    frame.push_back((rx_len_ >> 8) & 0xFF);
    frame.push_back(rx_len_ & 0xFF);
    frame.push_back((uint8_t)(~((header_sum_ - 1))));
    frame.insert(frame.end(), rx_payload_.begin(), rx_payload_.end());

    uint16_t calc = crc16(frame.data(), frame.size());
    if (calc == rx_crc_) {
      // Parse Payload
      // Note: rx_len_ >= 2 to allow Reverse Read Requests (RESPONSE with just SubID, no data)
      if (rx_len_ >= 2) {
        uint16_t sub_id = (rx_payload_[0] << 8) | rx_payload_[1];
        // DataType = rx_payload_[2] (if present)
        handle_parsed_frame_(rx_opcode_, sub_id, rx_payload_);
      }
    } else {
      ESP_LOGW(TAG, "CRC Fail: Exp %04X, Got %04X", calc, rx_crc_);
    }
    state_ = SYNC;
    break;
  }

  default:
    state_ = SYNC;
  }
}

void FP2Component::handle_parsed_frame_(uint8_t type, uint16_t sub_id,
                                        const std::vector<uint8_t> &payload) {
  OpCode op = (OpCode)type;
  //ESP_LOGI(TAG, "Received t:%d sub_id:%d", type, sub_id);

  switch (op) {
    case OpCode::ACK:
      handle_ack_(sub_id);
      break;
    case OpCode::REPORT:
      handle_report_(sub_id, payload);
      break;
    case OpCode::RESPONSE:
      handle_response_(sub_id, payload);
      break;
    default:
      ESP_LOGW(TAG, "Unhandled OpCode: %d", type);
      break;
  }
}

void FP2Component::handle_ack_(uint16_t sub_id) {
  if (waiting_for_ack_sub_id_ == sub_id) {
    ESP_LOGD(TAG, "ACK Received for 0x%04X", sub_id);
    waiting_for_ack_sub_id_ = 0xFFFF;
    if (!command_queue_.empty()) {
      command_queue_.pop_front();
    }
  } else {
    ESP_LOGW(TAG, "Unexpected ACK 0x%04X (Waiting for 0x%04X)", sub_id,
             waiting_for_ack_sub_id_);
  }
}

void FP2Component::handle_report_(uint16_t sub_id, const std::vector<uint8_t> &payload) {
  // Send ACK for all reports except heartbeat
  if (sub_id != 0x0102) {
    send_ack_(sub_id);
  }

  // Process specific report types
  switch (sub_id) {
    case 0x0102:  // Heartbeat
      last_heartbeat_millis_ = millis();
      break;

    case 0x0142:  // Zone Presence
      handle_zone_presence_report_(payload);
      break;

    case 0x0117:  // Location Tracking Data
      handle_location_tracking_report_(payload);
      break;

    default:
      // Unknown report type - already logged in main handler
      break;
  }
}

void FP2Component::handle_zone_presence_report_(const std::vector<uint8_t> &payload) {
  // Payload: [SubID 2B] [Type 0x01(UINT16)] [ValH] [ValL]
  // ValH = ZoneID, ValL = State (1=Occ, 0=Empty)
  if (payload.size() >= 5 && payload[2] == 0x01) {
    uint8_t zone_id = payload[3];
    uint8_t state = payload[4];
    ESP_LOGD(TAG, "Zone Presence Report: Zone %d = %s", zone_id, state ? "ON" : "OFF");

    for (auto &z : zones_) {
      if (z->id == zone_id) {
        z->publish_presence(state == 1);
        break;
      }
    }
  }
}

void FP2Component::handle_location_tracking_report_(const std::vector<uint8_t> &payload) {
  // Ignore stale data if location reporting has been disabled
  if (!this->location_reporting_active_) {
    return;
  }

  // Payload: [SubID 2] [Type 0x06(BLOB2)] [Len 2] [Count 1] [Target 14]...
  if (payload.size() < 6 || payload[2] != 0x06) {
    return;
  }

  uint8_t count = payload[5];
  std::string json_str = "[";
  char buf[256];

  for (int i = 0; i < count; i++) {
    int offset = 6 + (i * 14);
    if (offset + 14 > payload.size())
      break;

    const uint8_t *p = &payload[offset];

    // Parse target data (Big Endian)
    uint8_t tid = p[0];
    int16_t x = (int16_t)((p[1] << 8) | p[2]);
    int16_t y = (int16_t)((p[3] << 8) | p[4]);
    int16_t z = (int16_t)((p[5] << 8) | p[6]);
    int16_t velocity = (int16_t)((p[7] << 8) | p[8]);
    int16_t snr = (int16_t)((p[9] << 8) | p[10]);
    uint8_t classifier = p[11];
    uint8_t posture = p[12];
    uint8_t active = p[13];

    snprintf(buf, sizeof(buf),
             "{\"id\":%d,\"x\":%d,\"y\":%d,\"z\":%d,\"velocity\":%d,\"snr\":"
             "%d,\"classifier\":%d,\"posture\":%d,\"active\":%d}",
             tid, x, y, z, velocity, snr, classifier, posture, active);

    if (i > 0)
      json_str += ",";
    json_str += buf;
  }
  json_str += "]";

  if (this->target_tracking_sensor_ != nullptr) {
    this->target_tracking_sensor_->publish_state(json_str);
  }
}

void FP2Component::handle_response_(uint16_t sub_id, const std::vector<uint8_t> &payload) {
  // RESPONSE packets with only 2 bytes (just SubID) are Reverse Read Requests from the radar
  if (payload.size() == 2) {
    handle_reverse_read_request_(sub_id);
  } else {
    // Normal Response with data (currently unused)
    ESP_LOGD(TAG, "Received Response for 0x%04X with %d bytes", sub_id, payload.size());
  }
}

void FP2Component::handle_reverse_read_request_(uint16_t sub_id) {
  ESP_LOGI(TAG, "Received Reverse Query for SubID 0x%04X", sub_id);

  switch (sub_id) {
    case 0x0143:  // device_direction
      send_reverse_response_(sub_id, (uint8_t)fp2_accel_->get_orientation());
      ESP_LOGD(TAG, "Sending Device Direction: %d", fp2_accel_->get_orientation());
      break;

    case 0x0120:  // angle_sensor_data
      {
        uint8_t angle = fp2_accel_->get_output_angle_z();
        send_reverse_response_(sub_id, angle);
        ESP_LOGD(TAG, "Sending Angle Sensor Data: %d", angle);
      }
      break;

    default:
      ESP_LOGW(TAG, "Unknown Reverse Query SubID 0x%04X", sub_id);
      break;
  }
}

// Command Queue Helpers
void FP2Component::enqueue_command_(OpCode type, uint16_t sub_id,
                                    uint8_t byte_val) {
  FP2Command cmd;
  cmd.type = type;
  cmd.sub_id = sub_id;
  cmd.retry_count = 0;

  // Payload: [SubID 2] [Type 1] [Data 1]
  cmd.data.push_back((sub_id >> 8) & 0xFF);
  cmd.data.push_back(sub_id & 0xFF);
  cmd.data.push_back(0x00); // UINT8
  cmd.data.push_back(byte_val);

  command_queue_.push_back(cmd);
}

void FP2Component::enqueue_command_(OpCode type, uint16_t sub_id,
                                    uint16_t word_val) {
  FP2Command cmd;
  cmd.type = type;
  cmd.sub_id = sub_id;
  cmd.retry_count = 0;

  // Payload: [SubID 2] [Type 1] [Data 2]
  cmd.data.push_back((sub_id >> 8) & 0xFF);
  cmd.data.push_back(sub_id & 0xFF);
  cmd.data.push_back(0x01); // UINT16
  cmd.data.push_back((word_val >> 8) & 0xFF);
  cmd.data.push_back(word_val & 0xFF);

  command_queue_.push_back(cmd);
}

void FP2Component::enqueue_command_(OpCode type, uint16_t sub_id,
                                    bool bool_val) {
  FP2Command cmd;
  cmd.type = type;
  cmd.sub_id = sub_id;
  cmd.retry_count = 0;

  // Payload: [SubID 2] [Type 1] [Data 1]
  cmd.data.push_back((sub_id >> 8) & 0xFF);
  cmd.data.push_back(sub_id & 0xFF);
  cmd.data.push_back(0x04); // BOOL
  cmd.data.push_back((uint8_t) bool_val);

  command_queue_.push_back(cmd);
}


void FP2Component::enqueue_command_blob2_(
    uint16_t sub_id, const std::vector<uint8_t> &blob_content) {
  FP2Command cmd;
  cmd.type = OpCode::WRITE; // Always Write for these configs
  cmd.sub_id = sub_id;
  cmd.retry_count = 0;

  // Payload: [SubID 2] [Type 1 (0x06)] [Len 2] [Content N]
  cmd.data.push_back((sub_id >> 8) & 0xFF);
  cmd.data.push_back(sub_id & 0xFF);
  cmd.data.push_back(0x06); // BLOB2

  uint16_t len = blob_content.size();
  cmd.data.push_back((len >> 8) & 0xFF);
  cmd.data.push_back(len & 0xFF);

  cmd.data.insert(cmd.data.end(), blob_content.begin(), blob_content.end());

  command_queue_.push_back(cmd);
}

void FP2Component::set_interference_grid(const std::vector<uint8_t> &grid) {
  ESP_LOGI(TAG, "set_interference_grid called with size: %d", grid.size());
  if (grid.size() == 40) {
    std::copy(grid.begin(), grid.end(), interference_grid_.begin());
    has_interference_grid_ = true;
    ESP_LOGI(TAG, "Interference grid configured successfully");
  } else {
    ESP_LOGW(TAG, "Interference grid size mismatch! Expected 40, got %d", grid.size());
  }
}

void FP2Component::set_exit_grid(const std::vector<uint8_t> &grid) {
  ESP_LOGI(TAG, "set_exit_grid called with size: %d", grid.size());
  if (grid.size() == 40) {
    std::copy(grid.begin(), grid.end(), exit_grid_.begin());
    has_exit_grid_ = true;
    ESP_LOGI(TAG, "Exit grid configured successfully");
  } else {
    ESP_LOGW(TAG, "Exit grid size mismatch! Expected 40, got %d", grid.size());
  }
}

void FP2Component::set_edge_grid(const std::vector<uint8_t> &grid) {
  ESP_LOGI(TAG, "set_edge_grid called with size: %d", grid.size());
  if (grid.size() == 40) {
    std::copy(grid.begin(), grid.end(), edge_grid_.begin());
    has_edge_grid_ = true;
    ESP_LOGI(TAG, "Edge grid configured successfully");
  } else {
    ESP_LOGW(TAG, "Edge grid size mismatch! Expected 40, got %d", grid.size());
  }
}

void FP2Component::set_zones(const std::vector<FP2Zone*> &zones) {
    zones_ = zones;
}

void FP2TextSensor::setup() {
  // Link sensor to zone for zone_map type
  if (sensor_type_ == FP2TextSensorType::ZONE_MAP && zone_ != nullptr) {
    zone_->set_map_sensor(this);
  }
  // Link grid sensors to parent component
  if (parent_ != nullptr) {
    if (sensor_type_ == FP2TextSensorType::EDGE_LABEL_GRID) {
      parent_->set_edge_label_grid_sensor(this);
    } else if (sensor_type_ == FP2TextSensorType::ENTRY_EXIT_GRID) {
      parent_->set_entry_exit_grid_sensor(this);
    } else if (sensor_type_ == FP2TextSensorType::INTERFERENCE_GRID) {
      parent_->set_interference_grid_sensor(this);
    } else if (sensor_type_ == FP2TextSensorType::MOUNTING_POSITION) {
      parent_->set_mounting_position_sensor(this);
    }
  }
}

std::string FP2Component::grid_to_hex_card_format(const GridMap &grid) {
  std::string result;
  result.reserve(56);  // 14 rows * 2 bytes * 2 hex chars

  // For each row R (0-13) in the card format
  for (int R = 0; R < 14; R++) {
    // Map to internal row (rows 3-16 in the 20-row grid)
    int internal_row_idx = R;
    int byte_idx = internal_row_idx * 2;

    // Encode the 2 bytes for this row as 4 hex chars
    char hex[5];
    snprintf(hex, sizeof(hex), "%02x%02x", grid[byte_idx], grid[byte_idx + 1]);
    result += hex;
  }

  return result;
}

// void FP2Component::add_zone(uint8_t id, binary_sensor::BinarySensor *sens,
//                             const std::vector<uint8_t> &grid,
//                             uint8_t sensitivity) {
//   FP2Zone z;
//   z.id = id;
//   z.occupancy = sens;
//   z.sensitivity = sensitivity;
//
//   if (grid.size() == 40) {
//     std::copy(grid.begin(), grid.end(), z.grid.begin());
//     zones_.push_back(z);
//   } else {
//     ESP_LOGE(TAG, "Zone %d grid size mismatch: %d != 40", id, grid.size());
//   }
// }

void FP2Component::dump_config() {
  ESP_LOGCONFIG(TAG, "Aqara FP2:");
  ESP_LOGCONFIG(TAG, "  Mounting Position: %d", mounting_position_);
  ESP_LOGCONFIG(TAG, "  Zones: %d", zones_.size());
  if (reset_pin_ != nullptr) {
    LOG_PIN("  Reset Pin: ", reset_pin_);
  }
  for (auto &z : zones_) {
    if (z->presence_sensor != nullptr) {
      LOG_BINARY_SENSOR("  ", "Zone Presence", z->presence_sensor);
    }
    if (z->motion_sensor != nullptr) {
      LOG_BINARY_SENSOR("  ", "Zone Motion", z->motion_sensor);
    }
  }
}

JsonDocument FP2Component::get_map_config_json() {
  JsonDocument doc;

  // Deserialize the compile-time JSON
  DeserializationError error = deserializeJson(doc, this->map_config_json_);

  if (error) {
    ESP_LOGE(TAG, "Failed to parse map config JSON: %s", error.c_str());
    return doc;
  }

  // The base structure is already in the JSON, but we can add runtime data if needed
  // For example, current zone states could be added here in the future

  return doc;
}

} // namespace aqara_fp2
} // namespace esphome
