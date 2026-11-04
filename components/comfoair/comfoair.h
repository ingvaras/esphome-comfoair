#pragma once

#include "esphome.h"
#include "esphome/components/button/button.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/climate/climate_mode.h"
#include "esphome/components/climate/climate_traits.h"
#include "esphome/components/number/number.h"
#include "esphome/components/select/select.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "proxy_bus.h"
#include "registers.h"

#include <algorithm>

#ifdef USE_ESP32
#include <driver/gpio.h>
#include <driver/uart.h>
#include "esphome/components/uart/uart_component_esp_idf.h"
#endif

namespace esphome
{
namespace comfoair
{

// These are the possible status of the two receive functions for CA and CS
typedef enum
{
  RX_STATUS_DEFAULT,                   // default (should not happen. coding error?)
  RX_STATUS_RECEIVED_ACK,              // ACK received
  RX_STATUS_RECEIVED_MESSAGE,          // message received
  RX_STATUS_RECEIVED_INVALID_MESSAGE,  // invalid message received (rx error)
  RX_STATUS_RECEIVED_START_OF_MESSAGE, // found rx-Start and adjusted buffer accordingly (just 4 info. no error.)
  RX_STATUS_WRAPPED_BUFFER_INDEX       // wrapped buffer index (just 4 info. no error.)
} rx_status;

static const uint32_t TEST_PANEL_FRAME_INTERVAL_MS = 2000U;
// The wiring check starts if the unit or panel is still silent this long after boot.
static const uint32_t WIRING_CHECK_DELAY_MS = 30000U;
static const uint32_t WIRING_CHECK_TICK_MS = 1000U;
static const int WIRING_CHECK_LISTEN_TICKS = 15;
// unit_write_only: the unit can't confirm a level, so the remembered one is re-sent this often.
static const uint32_t WRITE_ONLY_LEVEL_RESEND_MS = 300000U;
static const uint8_t COMFOAIR_MIN_SUPPORTED_TEMP = 12;
static const uint8_t COMFOAIR_MAX_SUPPORTED_TEMP = 29;
static const float COMFOAIR_SUPPORTED_TEMP_STEP = 0.5f;
static const uint8_t MAX_MESSAGE_SIZE = 70U;
// Reserve enough room for start/stop markers, command, length, and a
// worst-case escaped checksum in addition to a fully escaped payload.
static const uint8_t MAX_PROTOCOL_DATA_SIZE = (MAX_MESSAGE_SIZE - 9U) / 2U;
static_assert(MAX_MESSAGE_SIZE == PROXY_MAX_FRAME_SIZE, "proxy_bus.h frame size must match MAX_MESSAGE_SIZE");

static const char *TAG = "comfoair";

class ComfoAirComponent;

// What unit_write_only remembers across restarts, because the unit can't be asked.
struct WriteOnlyState
{
  uint8_t level{0x02};   // low
  uint8_t comfort{80U};  // 20 degrees C
};

class ComfoAirSizeSelect : public select::Select
{
public:
  void set_parent(ComfoAirComponent *parent) { this->parent_ = parent; }

protected:
  void control(const std::string &value) override;

private:
  ComfoAirComponent *parent_{nullptr};
};

class ComfoAirNumber : public number::Number
{
public:
  void set_parent(ComfoAirComponent *parent) { this->parent_ = parent; }

protected:
  void control(float value) override;

private:
  ComfoAirComponent *parent_{nullptr};
};

class ComfoAirSyncButton : public button::Button
{
public:
  void set_parent(ComfoAirComponent *parent) { this->parent_ = parent; }

protected:
  void press_action() override;

private:
  ComfoAirComponent *parent_{nullptr};
};

class ComfoAirComponent : public climate::Climate, public PollingComponent, public uart::UARTDevice
{
  friend class ComfoAirSizeSelect;
  friend class ComfoAirNumber;
  friend class ComfoAirSyncButton;

public:
  // Poll every 2000ms (previously 600ms)
  ComfoAirComponent() : Climate(), PollingComponent(2000), UARTDevice() {}

  // Return the traits of this controller.
  climate::ClimateTraits traits() override
  {
    auto traits = climate::ClimateTraits();
    traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
    traits.set_supported_modes({climate::CLIMATE_MODE_FAN_ONLY});
    traits.clear_feature_flags(climate::CLIMATE_REQUIRES_TWO_POINT_TARGET_TEMPERATURE);
    traits.clear_feature_flags(climate::CLIMATE_SUPPORTS_ACTION);
    traits.set_visual_min_temperature(COMFOAIR_MIN_SUPPORTED_TEMP);
    traits.set_visual_max_temperature(COMFOAIR_MAX_SUPPORTED_TEMP);
    traits.set_visual_temperature_step(COMFOAIR_SUPPORTED_TEMP_STEP);
    traits.set_supported_fan_modes({
        climate::CLIMATE_FAN_AUTO,
        climate::CLIMATE_FAN_LOW,
        climate::CLIMATE_FAN_MEDIUM,
        climate::CLIMATE_FAN_HIGH,
        climate::CLIMATE_FAN_OFF,
    });
    if (homekit_climate_)
    {
      // HomeKit gives fan-only a two-ended range, and an auto fan moves the speed slider off the tile.
      // Without a fan Off, HomeKit snaps the slider back from zero instead of turning the unit off.
      traits.set_supported_modes({climate::CLIMATE_MODE_OFF, climate::CLIMATE_MODE_COOL});
      traits.set_supported_fan_modes({
          climate::CLIMATE_FAN_LOW,
          climate::CLIMATE_FAN_MEDIUM,
          climate::CLIMATE_FAN_HIGH,
          climate::CLIMATE_FAN_OFF,
      });
    }
    return traits;
  }

  // Override control to change settings of the climate device.
  void control(const climate::ClimateCall &call) override
  {
    if (homekit_climate_ && call.get_mode().has_value() && !call.get_fan_mode().has_value())
    {
      // The unit has no off, so Off is the Absent level. Turning back on starts at Low.
      const bool off = *call.get_mode() == climate::CLIMATE_MODE_OFF;
      if (off || mode == climate::CLIMATE_MODE_OFF)
      {
        const uint8_t level = off ? 0x01 : 0x02;
        set_level_(level);
        apply_level_(level);
      }
    }
    if (call.get_fan_mode().has_value())
    {
      int level;

      fan_mode = *call.get_fan_mode();
      switch (fan_mode.value())
      {
      case climate::CLIMATE_FAN_HIGH:
        level = 0x04;
        break;
      case climate::CLIMATE_FAN_MEDIUM:
        level = 0x03;
        break;
      case climate::CLIMATE_FAN_LOW:
        level = 0x02;
        break;
      case climate::CLIMATE_FAN_OFF:
        level = 0x01;
        break;
      case climate::CLIMATE_FAN_AUTO:
        level = 0x00;
        break;
      case climate::CLIMATE_FAN_ON:
      case climate::CLIMATE_FAN_MIDDLE:
      case climate::CLIMATE_FAN_DIFFUSE:
      default:
        level = -1;
        break;
      }

      if (level >= 0)
      {
        set_level_(level);
        if (homekit_climate_)
        {
          apply_level_(level);
        }
      }
    }
    if (call.get_target_temperature().has_value())
    {
      target_temperature = *call.get_target_temperature();
      set_comfort_temperature_(target_temperature);
    }

    publish_state();
  }

  void dump_config() override
  {
    uint8_t *p;
    ESP_LOGCONFIG(TAG, "ComfoAir:");
    // LOG_UPDATE_INTERVAL(this);
    p = bootloader_version_;
    ESP_LOGCONFIG(TAG, "  Bootloader %.10s v%0d.%02d b%2d", p + 3, *p, *(p + 1), *(p + 2));
    p = firmware_version_;
    ESP_LOGCONFIG(TAG, "  Firmware %.10s v%0d.%02d b%2d", p + 3, *p, *(p + 1), *(p + 2));
    p = connector_board_version_;
    ESP_LOGCONFIG(TAG, "  Connector Board %.10s v%0d.%02d", p + 2, *p, *(p + 1));

    if (*(p + 12) != 0)
    {
      ESP_LOGCONFIG(TAG, "  CC-Ease v%0d.%02d", *(p + 12) >> 4, *(p + 12) & 0x0f);
    }
    if (*(p + 13) != 0)
    {
      ESP_LOGCONFIG(TAG, "  CC-Luxe v%0d.%02d", *(p + 13) >> 4, *(p + 13) & 0x0f);
    }
    if (proxy_uart_ != nullptr)
    {
      ESP_LOGCONFIG(TAG, "  ComfoSense proxy: enabled (frames from Home Assistant wait for an idle bus)");
    }
    if (unit_write_only_)
    {
      ESP_LOGCONFIG(TAG, "  Unit write-only: commands only, nothing is sent to the panel, level %u",
                    write_only_state_.level);
    }
    if (homekit_climate_)
    {
      ESP_LOGCONFIG(TAG, "  HomeKit climate: modes Off and Cool, no auto fan");
    }
#ifdef USE_ESP32
    if (wiring_check_running_)
    {
      ESP_LOGCONFIG(TAG, "  Wiring check: running, the result follows in about a minute");
    }
    if (wiring_check_done_)
    {
      log_wiring_check_result_();
    }
#endif
  }

  void setup() override
  {
    boot_ms_ = millis();
    last_test_panel_frame_ms_ = boot_ms_;
    if (unit_write_only_)
    {
      write_only_pref_ = global_preferences->make_preference<WriteOnlyState>(fnv1_hash("comfoair_write_only"), true);
      WriteOnlyState stored;
      if (write_only_pref_.load(&stored) && stored.level <= 4)
      {
        write_only_state_ = stored;
        write_only_level_known_ = true;
      }
    }
    if (proxy_uart_ != nullptr)
    {
      // Relaying bytes between two UARTs needs a much faster loop than the default ~16ms.
      high_freq_.start();
    }
  }

  void update() override
  {
    if (wiring_check_running_)
    {
      return;
    }
    if (unit_write_only_)
    {
      write_only_update_();
      return;
    }
    ESP_LOGD(TAG, "Component update: %d", update_counter_);
    switch (update_counter_)
    {
    case -4:
      write_command_(CMD_GET_BOOTLOADER_VERSION, nullptr, 0);
      break;
    case -3:
      write_command_(CMD_GET_FIRMWARE_VERSION, nullptr, 0);
      break;
    case -2:
      write_command_(CMD_GET_CONNECTOR_BOARD_VERSION, nullptr, 0);
      break;
    case -1:
      write_command_(CMD_GET_STATUS, nullptr, 0);
      break;
    case 0:
      get_fan_status_();
      break;
    case 1:
      get_valve_status_();
      break;
    case 2:
      get_sensor_data_();
      break;
    case 3:
      get_ventilation_level_();
      break;
    case 4:
      get_temperatures_();
      break;
    case 5:
      get_error_status_();
      break;
    case 6:
      get_bypass_control_status_();
      break;
    case 7:
      get_operation_hours_();
      break;
    case 8:
      get_preheating_status_();
      break;
    case 9:
      get_time_delay_();
      break;
    default:
      ESP_LOGI(TAG, "Component update: %d", update_counter_);
      break;
    }

    update_counter_++;
    if (update_counter_ > 9) // num_update_counter_elements_
    {
      update_counter_ = 0;
    }
  }

  void loop() override
  {
    /*
    TX:
    1. Build TX-array:
      - 2 Byte Command
      - 1 Byte Size = amount of data bytes (0...n)
      - 0...n data bytes = payload data (without doubled 0x07 bytes yet)
    2. Calculate and append checksum across TX-array.
    3. transmit function:
      - transmit 2 Byte "START" (0x07 0xF0)
      - transmit TX-array
        - transmit byte 1 to byte 3
        - transmit byte 4 to byte [3.byte + 3] while transmitting each 0x07 twice.
        - transmit byte [3.byte + 4].
      - transmit 2 Byte "STOP" (0x07 0x0F)

    RX:
    1. read uart byte by byte into RX-array
      - search for ACK (0x07 0xF3) -> announce new ACK
      - search for START (0x07 0xF0) and STOP (0x07 0x0F) -> announce new message
    2. remove START, STOP and doubled 0x07 from new RX-array to get clean data.
    3. calculate and verify checksum of new message.
      - send ACK?
    4. process RX data
    */

    // CA350
    // caRxSerial();       // receive ACKs and messages from CA350

    if (proxy_uart_ != nullptr)
    {
#ifdef USE_ESP32
      if (wiring_check_loop_())
      {
        return;
      }
#endif
      proxy_loop_();
      return;
    }

    while (available() != 0)
    {
      uint8_t rx_byte_u8;

      // fetch byte for RX buffer and process it by readRx()
      read_byte(&rx_byte_u8);
      log_raw_byte_(unit_raw_, "unit", rx_byte_u8);

      process_unit_rx_(checkRx_(caRxBuffer_au8, &caRxIdx_u8, rx_byte_u8), true);
    }
    flush_raw_logs_();

    // ComfoSense
    // csRxSerial();       // receive ACKs and messages from ComfoSense
  }

  float get_setup_priority() const override { return setup_priority::DATA; }

  void reset_errors(void) { reset_errors_(false, true); }

  void reset_filters(void) { reset_errors_(true, false); }

  // Backward-compatible names used by existing YAML configurations.
  void error_reset(void) { reset_errors(); }
  void filter_reset(void) { reset_filters(); }

  void set_name(const char *value) { name = value; }
  void set_name(const char *value, uint32_t name_hash)
  {
    (void)name_hash;
    name = value;
  }
  void set_uart_component(uart::UARTComponent *parent) { set_uart_parent(parent); }
  // Optional second UART connected to the ComfoSense panel (proxy mode).
  void set_proxy_uart(uart::UARTComponent *proxy_uart) { proxy_uart_ = proxy_uart; }
  // Log every byte received from the unit (and the panel) as hex, for protocol debugging.
  void set_log_raw_bytes(bool log_raw_bytes) { log_raw_bytes_ = log_raw_bytes; }
  // Test aid: answer the panel's temperature poll with fake values (proxy mode only).
  void set_test_fake_unit_reply(bool value) { test_fake_unit_reply_ = value; }
  // Test aid: periodically send the panel a frame that it should acknowledge (proxy mode only).
  void set_test_panel_frame(bool value) { test_panel_frame_ = value; }
  // Test aid: acknowledge every valid frame from the panel ourselves (proxy mode only).
  void set_test_ack_panel_frames(bool value) { test_ack_panel_frames_ = value; }
  // Enable the automatic wiring check (proxy mode on ESP32), with the GPIO numbers as configured.
  void set_wiring_check_pins(uint8_t unit_rx, uint8_t unit_tx, uint8_t panel_rx, uint8_t panel_tx)
  {
    wiring_check_pins_[0] = unit_rx;
    wiring_check_pins_[1] = unit_tx;
    wiring_check_pins_[2] = panel_rx;
    wiring_check_pins_[3] = panel_tx;
    wiring_check_pending_ = true;
  }
  // The unit can't answer: only send it commands, and relay the panel's. Nothing goes to the panel.
  void set_unit_write_only(bool value) { unit_write_only_ = value; }
  void set_homekit_climate(bool value) { homekit_climate_ = value; }
  const WiringCheck &wiring_check() const { return wiring_check_; }
  bool wiring_check_running() const { return wiring_check_running_; }
  bool wiring_check_done() const { return wiring_check_done_; }
  bool set_unit_size(uint8_t raw_size);
  void set_size_select(ComfoAirSizeSelect *size_select);

  void set_level(int level) { set_level_(level); }
  void set_comfort_temperature(float temperature) { set_comfort_temperature_(temperature); }

protected:
  // Handle one result of checkRx_() for the stream coming from the unit.
  // `ack_messages` is false in proxy mode for relayed frames: the panel
  // acknowledges those itself.
  void process_unit_rx_(rx_status status, bool ack_messages)
  {
    switch (status)
    {
    case RX_STATUS_DEFAULT:
      break;
    case RX_STATUS_RECEIVED_ACK:
      ESP_LOGVV(TAG, "RX: ACK");
      // caProcessNewACK(); // do something if an ACK is received
      break;
    case RX_STATUS_RECEIVED_MESSAGE:
      ESP_LOGVV(TAG, "RX: message cmd: %02X", caRxBuffer_au8[1]);
      // acknowledge message
      if (ack_messages)
      {
        txACK_();
      }
      // at this point caRxBuffer_au8 consists of
      // 2-byte command, length, data and checksum.
      // caProcessNewData(caRxBuffer); // do something with received message
      parseRxMessage_(caRxBuffer_au8);
      break;
    case RX_STATUS_RECEIVED_INVALID_MESSAGE:
      ESP_LOGW(TAG, "RX: Invalid checksum (from CA350).");
      break;
    case RX_STATUS_RECEIVED_START_OF_MESSAGE:
    case RX_STATUS_WRAPPED_BUFFER_INDEX:
    default:
      break;
    }
  }

  // --- raw byte logging (log_raw_bytes: true) ---

  void log_raw_byte_(RawByteCollector &collector, const char *direction, uint8_t byte)
  {
    if (!log_raw_bytes_)
    {
      return;
    }
    if (collector.add(byte, millis()))
    {
      flush_raw_(collector, direction);
    }
  }

  void flush_raw_(RawByteCollector &collector, const char *direction)
  {
    if (collector.length() == 0)
    {
      return;
    }
    char hex[RawByteCollector::CAPACITY * 3];
    format_hex_pretty_to(hex, collector.data(), collector.length(), ' ');
    ESP_LOGI(TAG, "RAW %s: %s", direction, hex);
    collector.clear();
  }

  void flush_raw_logs_()
  {
    if (!log_raw_bytes_)
    {
      return;
    }
    const uint32_t now = millis();
    if (unit_raw_.should_flush(now))
    {
      flush_raw_(unit_raw_, proxy_uart_ != nullptr ? "unit->panel" : "unit");
    }
    if (panel_raw_.should_flush(now))
    {
      flush_raw_(panel_raw_, "panel->unit");
    }
  }

  // --- proxy mode (ComfoSense connected to a second UART) ---

  // Commands that the unit answers with a bare ACK instead of a response frame.
  static bool is_ack_only_command_(uint8_t command)
  {
    switch (command)
    {
    case CMD_SET_LEVEL:
    case CMD_SET_ANALOG_VALUES:
    case CMD_SET_TIME_DELAY:
    case CMD_SET_VENTILATION_LEVEL:
    case CMD_SET_COMFORT_TEMPERATURE:
    case CMD_SET_STATUS:
    case CMD_RESET_AND_SELF_TEST:
    case CMD_SET_EWT_POSTHEATING:
      return true;
    default:
      return false;
    }
  }

  void proxy_loop_()
  {
    const uint32_t now = millis();
    uint8_t byte_u8;
    uint8_t relay_au8[MAX_MESSAGE_SIZE];
    uint8_t relay_len_u8 = 0;

    // Panel -> unit. Bytes are relayed untouched and immediately, unless we
    // currently own the bus; then they are held back until our exchange ends.
    while (proxy_uart_->available() != 0 && proxy_uart_->read_byte(&byte_u8))
    {
      arbiter_.note_panel_byte(millis());
      log_raw_byte_(panel_raw_, "panel->unit", byte_u8);
      // Track frames to know when the bus is busy. Only unit_write_only uses the content.
      if (checkRx_(csRxBuffer_au8, &csRxIdx_u8, byte_u8) == RX_STATUS_RECEIVED_MESSAGE)
      {
        if (unit_write_only_)
        {
          note_panel_frame_(csRxBuffer_au8);
        }
        else
        {
          arbiter_.panel_frame_complete(millis());
        }
        if (test_ack_panel_frames_)
        {
          // Acknowledge the frame ourselves, as the unit would. A panel that receives this
          // stops resending the frame.
          proxy_uart_->write_byte(COMMAND_PREFIX);
          proxy_uart_->write_byte(COMMAND_HEAD_ACK);
        }
        if (test_fake_unit_reply_)
        {
          send_fake_unit_reply_(csRxBuffer_au8[1]);
        }
      }

      if (arbiter_.own_exchange() || !hold_.empty())
      {
        if (!hold_.push(byte_u8))
        {
          ESP_LOGW(TAG, "Proxy: hold buffer full, dropping byte from ComfoSense");
        }
      }
      else
      {
        relay_au8[relay_len_u8++] = byte_u8;
        if (relay_len_u8 == sizeof(relay_au8))
        {
          write_array(relay_au8, relay_len_u8);
          relay_len_u8 = 0;
        }
      }
    }
    if (relay_len_u8 != 0)
    {
      write_array(relay_au8, relay_len_u8);
      relay_len_u8 = 0;
    }

    // Unit -> panel. Everything is parsed. While we own the bus the unit is
    // talking to us, so nothing is relayed to the panel.
    while (available() != 0 && read_byte(&byte_u8))
    {
      log_raw_byte_(unit_raw_, "unit->panel", byte_u8);
      if (unit_write_only_)
      {
        // Nothing goes to the panel in this mode.
        if (!unit_byte_seen_)
        {
          unit_byte_seen_ = true;
          ESP_LOGW(TAG, "Received data from the unit although unit_write_only is set; its TX may work after all");
        }
        continue;
      }
      arbiter_.note_unit_byte(millis());
      const bool own = arbiter_.own_exchange();
      if (!own)
      {
        relay_au8[relay_len_u8++] = byte_u8;
        if (relay_len_u8 == sizeof(relay_au8))
        {
          proxy_uart_->write_array(relay_au8, relay_len_u8);
          relay_len_u8 = 0;
        }
      }

      const rx_status status = checkRx_(caRxBuffer_au8, &caRxIdx_u8, byte_u8);
      process_unit_rx_(status, own);
      if (status == RX_STATUS_RECEIVED_MESSAGE)
      {
        arbiter_.unit_frame_complete();
      }
      if (own && ((status == RX_STATUS_RECEIVED_MESSAGE && inflight_expects_response_) ||
                  (status == RX_STATUS_RECEIVED_ACK && !inflight_expects_response_)))
      {
        finish_own_exchange_(now, true);
      }
    }
    if (relay_len_u8 != 0)
    {
      proxy_uart_->write_array(relay_au8, relay_len_u8);
    }

    flush_raw_logs_();

    if (test_panel_frame_ && (now - last_test_panel_frame_ms_) >= TEST_PANEL_FRAME_INTERVAL_MS)
    {
      last_test_panel_frame_ms_ = now;
      send_test_panel_frame_();
    }

    if (arbiter_.own_exchange_timed_out(now))
    {
      finish_own_exchange_(now, false);
    }

    // Inject our own frame when the bus is idle.
    if (!arbiter_.own_exchange() && hold_.empty() && arbiter_.can_inject(now))
    {
      start_own_exchange_(now);
    }
  }

  // Hardware test aid (test_fake_unit_reply: true): answer the panel's temperature poll
  // ourselves, as if we were the unit, to prove that the ESP -> ComfoSense path works.
  // The panel should display these values: comfort 22.0, outside 11.5, supply 17.5,
  // extract 21.5 and exhaust 8.5 degrees C.
  void send_fake_unit_reply_(uint8_t request_command)
  {
    if (request_command != CMD_GET_TEMPERATURES)
    {
      return;
    }
    const uint8_t data[9] = {84, 63, 75, 83, 57, 0x0F, 0, 0, 0};
    proxy_uart_->write_byte(COMMAND_PREFIX);
    proxy_uart_->write_byte(COMMAND_HEAD_ACK);
    send_panel_frame_(RES_GET_TEMPERATURES, data, sizeof(data));
    proxy_uart_->flush();
    ESP_LOGI(TAG, "Test: answered ComfoSense temperature request with fake values");
  }

  void send_panel_frame_(uint8_t command, const uint8_t *data, uint8_t length)
  {
    uint8_t message_au8[MAX_MESSAGE_SIZE]{};
    message_au8[1] = command;
    message_au8[2] = length;
    memcpy(&message_au8[3], data, length);

    uint8_t frame_au8[MAX_MESSAGE_SIZE];
    const uint8_t frame_len = encodeMessage_(message_au8, frame_au8);
    proxy_uart_->write_array(frame_au8, frame_len);
  }

  // --- unit_write_only: the unit's TX doesn't work ---

  // Keep up with what the panel commands. The frame itself is relayed to the unit as usual.
  void note_panel_frame_(const uint8_t *message_au8)
  {
    const uint8_t command = message_au8[1];
    const uint8_t length = message_au8[2];
    if (command == CMD_SET_LEVEL && length == CMD_SET_LEVEL_LENGTH)
    {
      ESP_LOGI(TAG, "Panel set level to: %u", message_au8[3]);
      remember_level_(message_au8[3]);
    }
    else if (command == CMD_SET_COMFORT_TEMPERATURE && length == CMD_SET_COMFORT_TEMPERATURE_LENGTH)
    {
      remember_comfort_(message_au8[3]);
    }
  }

  void apply_level_(uint8_t level)
  {
    if (ventilation_level != nullptr)
    {
      ventilation_level->publish_state(level == 0 ? 0 : level - 1);
    }

    // Fan Speed
    switch (level)
    {
    case 0x00:
      fan_mode = climate::CLIMATE_FAN_AUTO;
      mode = climate::CLIMATE_MODE_FAN_ONLY;
      break;
    case 0x01:
      fan_mode = climate::CLIMATE_FAN_OFF;
      mode = climate::CLIMATE_MODE_OFF;
      break;
    case 0x02:
      fan_mode = climate::CLIMATE_FAN_LOW;
      mode = climate::CLIMATE_MODE_FAN_ONLY;
      break;
    case 0x03:
      fan_mode = climate::CLIMATE_FAN_MEDIUM;
      mode = climate::CLIMATE_MODE_FAN_ONLY;
      break;
    case 0x04:
      fan_mode = climate::CLIMATE_FAN_HIGH;
      mode = climate::CLIMATE_MODE_FAN_ONLY;
      break;
    }
    if (homekit_climate_ && mode == climate::CLIMATE_MODE_FAN_ONLY)
    {
      mode = climate::CLIMATE_MODE_COOL;
    }
  }

  void remember_level_(uint8_t level)
  {
    if (level > 4)
    {
      return;
    }
    write_only_level_known_ = true;
    write_only_level_sent_ms_ = millis();
    write_only_state_.level = level;
    write_only_pref_.save(&write_only_state_);
    apply_level_(level);
    publish_state();
  }

  void remember_comfort_(uint8_t comfort)
  {
    write_only_state_.comfort = comfort;
    write_only_pref_.save(&write_only_state_);
    target_temperature = (float)comfort / 2.0f - 20.0f;
    publish_state();
  }

  // Polling is pointless without replies. This only keeps the unit on the remembered level.
  void write_only_update_()
  {
    const uint32_t now = millis();
    const bool first_update = !write_only_started_;
    if (first_update)
    {
      write_only_started_ = true;
      apply_level_(write_only_state_.level);
      target_temperature = (float)write_only_state_.comfort / 2.0f - 20.0f;
      publish_state();
    }
    if (write_only_level_known_ && (first_update || (now - write_only_level_sent_ms_) >= WRITE_ONLY_LEVEL_RESEND_MS))
    {
      write_only_level_sent_ms_ = now;
      write_command_(CMD_SET_LEVEL, &write_only_state_.level, CMD_SET_LEVEL_LENGTH);
    }
  }

  // Hardware test aid (test_panel_frame: true): send the panel a frame a real unit sends it
  // (clock and backlight parameters, Monday 12:34). A panel that receives it acknowledges with
  // 07 F3, which shows up as "RAW panel->unit: 07 F3" when log_raw_bytes is enabled.
  void send_test_panel_frame_()
  {
    uint8_t frame_au8[MAX_MESSAGE_SIZE];
    const uint8_t frame_len = encode_panel_clock_frame_(frame_au8);
    proxy_uart_->write_array(frame_au8, frame_len);
    proxy_uart_->flush();
    ESP_LOGI(TAG, "Test: sent clock frame to ComfoSense, expecting an ACK (07 F3) from the panel");
  }

  uint8_t encode_panel_clock_frame_(uint8_t frame_au8[])
  {
    uint8_t message_au8[MAX_MESSAGE_SIZE]{};
    message_au8[1] = CMD_SET_PARAMETER;
    message_au8[2] = 5;
    const uint8_t data[5] = {0x02, 12, 34, 30, 100};
    memcpy(&message_au8[3], data, sizeof(data));
    return encodeMessage_(message_au8, frame_au8);
  }

  void start_own_exchange_(uint32_t now)
  {
    if (!retry_pending_)
    {
      if (!tx_queue_.pop(&inflight_))
      {
        return;
      }
      inflight_attempts_ = 0;
    }
    retry_pending_ = false;
    inflight_attempts_++;
    inflight_expects_response_ = !is_ack_only_command_(inflight_.command());

    // The unit's reply must not be mistaken for a half-finished relayed frame.
    caRxIdx_u8 = 0;
    arbiter_.begin_own_exchange(now);
    ESP_LOGVV(TAG, "Proxy: injecting cmd %02X", inflight_.command());
    write_array(inflight_.data, inflight_.length);
    flush();
  }

  void finish_own_exchange_(uint32_t now, bool success)
  {
    arbiter_.end_own_exchange();
    caRxIdx_u8 = 0;
    if (!success)
    {
      // With unit_write_only no answer can arrive, so every command simply goes out twice.
      if (inflight_attempts_ < 2)
      {
        if (!unit_write_only_)
        {
          ESP_LOGD(TAG, "Proxy: no answer to cmd %02X, retrying", inflight_.command());
        }
        retry_pending_ = true;
      }
      else if (!unit_write_only_)
      {
        ESP_LOGW(TAG, "Proxy: no answer to cmd %02X, giving up", inflight_.command());
      }
    }

    // Replay what the panel sent while we were busy.
    if (!hold_.empty())
    {
      write_array(hold_.data(), hold_.length());
      hold_.clear();
      arbiter_.note_panel_byte(now);
    }
  }

#ifdef USE_ESP32
  // --- wiring check (proxy mode on ESP32, wiring_check: true) ---

  // Runs once per boot if the unit or panel stays silent. Takes over both UARTs while it
  // runs, then restores them. Returns true while it owns the UARTs.
  bool wiring_check_loop_()
  {
    const uint32_t now = millis();
    if (!wiring_check_running_)
    {
      const bool links_seen = arbiter_.unit_seen() && arbiter_.panel_seen();
      if (!wiring_check_pending_ || links_seen || (now - boot_ms_) < WIRING_CHECK_DELAY_MS ||
          arbiter_.own_exchange() || !hold_.empty())
      {
        return false;
      }
      ESP_LOGW(TAG, "Wiring check: no data from the %s, testing the pins for about a minute",
               arbiter_.unit_seen() ? "panel" : (arbiter_.panel_seen() ? "unit" : "unit or panel"));
      wiring_check_pending_ = false;
      wiring_check_running_ = true;
      wiring_check_tick_ = 0;
      wiring_check_tick_ms_ = now - WIRING_CHECK_TICK_MS;
    }
    if ((now - wiring_check_tick_ms_) >= WIRING_CHECK_TICK_MS)
    {
      wiring_check_tick_ms_ = now;
      wiring_check_step_();
    }
    return wiring_check_running_;
  }

  void wiring_check_step_()
  {
    const int tick = wiring_check_tick_++;
    if (tick == 0)
    {
      wiring_check_measure_pins_();
      wiring_check_listen_(0, 2);
    }
    else if (tick <= WIRING_CHECK_LISTEN_TICKS)
    {
      wiring_check_drain_(0, 2);
      if (tick == WIRING_CHECK_LISTEN_TICKS)
      {
        wiring_check_listen_(1, 3);
      }
    }
    else if (tick <= 2 * WIRING_CHECK_LISTEN_TICKS)
    {
      wiring_check_drain_(1, 3);
      if (tick == 2 * WIRING_CHECK_LISTEN_TICKS)
      {
        wiring_check_.plan_tests();
      }
    }
    else
    {
      // Each pin combination takes four ticks: send, read, send, read.
      const int step = tick - 2 * WIRING_CHECK_LISTEN_TICKS - 1;
      if (step / 4 >= wiring_check_.test_count())
      {
        wiring_check_finish_();
      }
      else
      {
        wiring_check_test_step_(step / 4, step % 4);
      }
    }
  }

  static uart_port_t wiring_check_port_(uart::UARTComponent *uart)
  {
    return static_cast<uart_port_t>(static_cast<uart::IDFUARTComponent *>(uart)->get_hw_serial_number());
  }

  gpio_num_t wiring_check_gpio_(uint8_t pin) const { return static_cast<gpio_num_t>(wiring_check_pins_[pin]); }

  static uint8_t wiring_check_count_high_(gpio_num_t pin)
  {
    uint8_t high_u8 = 0;
    delay(1);
    for (uint8_t i = 0; i < WiringCheck::SAMPLES_PER_PULL; i++)
    {
      high_u8 += gpio_get_level(pin) != 0 ? 1 : 0;
      delay(1);
    }
    return high_u8;
  }

  void wiring_check_release_pin_(uint8_t pin)
  {
    const gpio_num_t gpio = wiring_check_gpio_(pin);
    gpio_reset_pin(gpio);
    gpio_set_direction(gpio, GPIO_MODE_INPUT);
    gpio_set_pull_mode(gpio, GPIO_PULLUP_ONLY);
  }

  void wiring_check_measure_pins_()
  {
    for (uint8_t pin = 0; pin < WiringCheck::PIN_COUNT; pin++)
    {
      const gpio_num_t gpio = wiring_check_gpio_(pin);
      wiring_check_release_pin_(pin);
      const uint8_t high_with_pullup = wiring_check_count_high_(gpio);
      gpio_set_pull_mode(gpio, GPIO_PULLDOWN_ONLY);
      const uint8_t high_with_pulldown = wiring_check_count_high_(gpio);
      gpio_set_pull_mode(gpio, GPIO_PULLUP_ONLY);
      wiring_check_.set_drive(pin, WiringCheck::classify_pin(high_with_pullup, high_with_pulldown));
    }
  }

  void wiring_check_listen_(uint8_t unit_uart_pin, uint8_t panel_uart_pin)
  {
    const uart_port_t unit_port = wiring_check_port_(parent_);
    const uart_port_t panel_port = wiring_check_port_(proxy_uart_);
    uart_set_pin(unit_port, UART_PIN_NO_CHANGE, wiring_check_gpio_(unit_uart_pin), UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_set_pin(panel_port, UART_PIN_NO_CHANGE, wiring_check_gpio_(panel_uart_pin), UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_flush_input(unit_port);
    uart_flush_input(panel_port);
  }

  void wiring_check_drain_(uint8_t unit_uart_pin, uint8_t panel_uart_pin)
  {
    uint8_t byte_u8;
    while (available() != 0 && read_byte(&byte_u8))
    {
      wiring_check_.note_heard(unit_uart_pin, byte_u8);
    }
    while (proxy_uart_->available() != 0 && proxy_uart_->read_byte(&byte_u8))
    {
      wiring_check_.note_heard(panel_uart_pin, byte_u8);
    }
  }

  void wiring_check_test_step_(uint8_t test, int step)
  {
    const uart_port_t port = wiring_check_port_(parent_);
    const uint8_t tx = wiring_check_.test_tx(test);
    const uint8_t rx = wiring_check_.test_rx(test);
    uint8_t frame_au8[MAX_MESSAGE_SIZE];

    if (step == 0)
    {
      uart_set_pin(port, wiring_check_gpio_(tx), wiring_check_gpio_(rx), UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
      uart_flush_input(port);
      uint8_t message_au8[MAX_MESSAGE_SIZE]{};
      message_au8[1] = CMD_GET_FAN_STATUS;
      write_array(frame_au8, encodeMessage_(message_au8, frame_au8));
    }
    else if (step == 2)
    {
      uart_flush_input(port);
      write_array(frame_au8, encode_panel_clock_frame_(frame_au8));
    }
    else
    {
      uint8_t reply_au8[MAX_MESSAGE_SIZE];
      uint8_t length_u8 = 0;
      uint8_t byte_u8;
      while (available() != 0 && read_byte(&byte_u8))
      {
        if (length_u8 < sizeof(reply_au8))
        {
          reply_au8[length_u8++] = byte_u8;
        }
      }
      char hex[MAX_MESSAGE_SIZE * 3];
      format_hex_pretty_to(hex, reply_au8, length_u8, ' ');
      ESP_LOGD(TAG, "Wiring check: tx GPIO%d, rx GPIO%d, %s: %u bytes back: %s", wiring_check_pins_[tx],
               wiring_check_pins_[rx], step == 1 ? "fan poll" : "clock frame", length_u8, hex);
      if (step == 1)
      {
        wiring_check_.note_poll_reply(test, reply_au8, length_u8, RES_GET_FAN_STATUS);
      }
      else
      {
        wiring_check_.note_reply(test, reply_au8, length_u8);
        wiring_check_release_pin_(tx);
      }
    }
  }

  void wiring_check_finish_()
  {
    for (uint8_t pin = 0; pin < WiringCheck::PIN_COUNT; pin++)
    {
      gpio_reset_pin(wiring_check_gpio_(pin));
    }
    parent_->load_settings(false);
    proxy_uart_->load_settings(false);
    caRxIdx_u8 = 0;
    csRxIdx_u8 = 0;
    wiring_check_running_ = false;
    wiring_check_done_ = true;
    log_wiring_check_result_();
  }

  void log_wiring_check_adapter_(const char *name, uint8_t rx_pin, uint8_t tx_pin)
  {
    const int rx_gpio = wiring_check_pins_[rx_pin];
    const int tx_gpio = wiring_check_pins_[tx_pin];
    switch (wiring_check_.adapter_wiring(rx_pin, tx_pin))
    {
    case AdapterWiring::MATCHES_CONFIG:
      ESP_LOGCONFIG(TAG, "    %s adapter: ESP side matches the config", name);
      break;
    case AdapterWiring::SWAPPED:
      ESP_LOGCONFIG(TAG, "    %s adapter: ESP side is SWAPPED, set rx_pin: GPIO%d and tx_pin: GPIO%d", name, tx_gpio,
                    rx_gpio);
      break;
    case AdapterWiring::NO_OUTPUT:
      ESP_LOGCONFIG(TAG, "    %s adapter: no output on GPIO%d or GPIO%d, it has no power or is not connected", name,
                    rx_gpio, tx_gpio);
      break;
    case AdapterWiring::BOTH_DRIVEN:
      ESP_LOGCONFIG(TAG, "    %s adapter: GPIO%d and GPIO%d are both high, levels can't tell them apart", name, rx_gpio,
                    tx_gpio);
      break;
    case AdapterWiring::HELD_LOW:
      ESP_LOGCONFIG(TAG, "    %s adapter: GPIO%d or GPIO%d is held low, it has no power or its RS232 input is positive",
                    name, rx_gpio, tx_gpio);
      break;
    }
  }

  // Uses ESP_LOGCONFIG because dump_config() repeats this, so every new log connection shows it.
  void log_wiring_check_result_()
  {
    static const char *const PIN_NAMES[WiringCheck::PIN_COUNT] = {"unit RX", "unit TX", "panel RX", "panel TX"};
    ESP_LOGCONFIG(TAG, "  Wiring check result:");
    for (uint8_t pin = 0; pin < WiringCheck::PIN_COUNT; pin++)
    {
      const PinDrive drive = wiring_check_.drive(pin);
      char hex[WiringCheck::HEARD_SAMPLE_SIZE * 3];
      format_hex_pretty_to(hex, wiring_check_.sample(pin), wiring_check_.sample_length(pin), ' ');
      ESP_LOGCONFIG(TAG, "    GPIO%d (%s): %s, heard %u bytes: %s", wiring_check_pins_[pin], PIN_NAMES[pin],
                    drive == PinDrive::DRIVEN_HIGH  ? "driven high (adapter output)"
                    : drive == PinDrive::DRIVEN_LOW ? "held low"
                                                    : "not driven (adapter input or no connection)",
                    wiring_check_.heard(pin), hex);
    }
    log_wiring_check_adapter_("Unit", 0, 1);
    log_wiring_check_adapter_("Panel", 2, 3);
    if (wiring_check_.test_count() == 0)
    {
      ESP_LOGCONFIG(TAG, "    No pin combination could be tested");
    }
    for (uint8_t test = 0; test < wiring_check_.test_count(); test++)
    {
      ESP_LOGCONFIG(TAG, "    tx GPIO%d, rx GPIO%d: %s", wiring_check_pins_[wiring_check_.test_tx(test)],
                    wiring_check_pins_[wiring_check_.test_rx(test)],
                    wiring_check_.unit_answered(test) ? "the UNIT answered"
                    : wiring_check_.acked(test)       ? "ACK without unit data, probably the PANEL"
                    : wiring_check_.got_bytes(test)   ? "other bytes came back"
                                                      : "no answer");
    }
  }
#endif

  // --- setter ---

  void reset_errors_(bool filters, bool errors)
  {
    uint8_t reset_cmd[CMD_RESET_AND_SELF_TEST_LENGTH] = {errors ? (uint8_t)1 : (uint8_t)0, 0, 0,
                                                         filters ? (uint8_t)1 : (uint8_t)0};
    write_command_(CMD_RESET_AND_SELF_TEST, reset_cmd, sizeof(reset_cmd));
  }

  void set_level_(int level)
  {
    if (level < 0 || level > 4)
    {
      ESP_LOGI(TAG, "Ignoring invalid level request: %i", level);
      return;
    }

    ESP_LOGI(TAG, "Setting level to: %i", level);
    if (unit_write_only_)
    {
      remember_level_(level);
    }
    {
      uint8_t command[CMD_SET_LEVEL_LENGTH] = {(uint8_t)level};
      write_command_(CMD_SET_LEVEL, command, sizeof(command));
    }
  }

  void set_comfort_temperature_(float temperature)
  {
    if (temperature < COMFOAIR_MIN_SUPPORTED_TEMP || temperature > COMFOAIR_MAX_SUPPORTED_TEMP)
    {
      ESP_LOGI(TAG, "Ignoring invalid temperature request: %.1f", temperature);
      return;
    }

    ESP_LOGI(TAG, "Setting temperature to: %.1f", temperature);
    if (unit_write_only_)
    {
      remember_comfort_((uint8_t)((temperature + 20.0f) * 2.0f));
    }
    {
      uint8_t command[CMD_SET_COMFORT_TEMPERATURE_LENGTH] = {(uint8_t)((temperature + 20.0f) * 2.0f)};
      write_command_(CMD_SET_COMFORT_TEMPERATURE, command, sizeof(command));
    }
  }

  void set_fan_percentages()
  {
    if (return_air_level_absent == nullptr || return_air_level_low == nullptr || return_air_level_medium == nullptr ||
        return_air_level_high == nullptr || supply_air_level_absent == nullptr || supply_air_level_low == nullptr ||
        supply_air_level_medium == nullptr || supply_air_level_high == nullptr)
    {
      ESP_LOGW(TAG, "Cannot sync: one or more fan-level entities are missing");
      return;
    }

    uint8_t data[CMD_SET_VENTILATION_LEVEL_LENGTH];
    data[0] = static_cast<uint8_t>(std::clamp(static_cast<int>(return_air_level_absent->state), 15, 95));
    data[1] = static_cast<uint8_t>(std::clamp(static_cast<int>(return_air_level_low->state), 15, 95));
    data[2] = static_cast<uint8_t>(std::clamp(static_cast<int>(return_air_level_medium->state), 15, 95));
    data[3] = static_cast<uint8_t>(std::clamp(static_cast<int>(supply_air_level_absent->state), 15, 95));
    data[4] = static_cast<uint8_t>(std::clamp(static_cast<int>(supply_air_level_low->state), 15, 95));
    data[5] = static_cast<uint8_t>(std::clamp(static_cast<int>(supply_air_level_medium->state), 15, 95));
    data[6] = static_cast<uint8_t>(std::clamp(static_cast<int>(return_air_level_high->state), 15, 95));
    data[7] = static_cast<uint8_t>(std::clamp(static_cast<int>(supply_air_level_high->state), 15, 95));
    data[8] = 0x00;

    ESP_LOGD(TAG, "Setting fan speed percentages");
    write_command_(CMD_SET_VENTILATION_LEVEL, data, sizeof(data));
  }

  // --- TX functions ---

  // Build TX-array
  void write_command_(const uint8_t command, const uint8_t *command_data, uint8_t command_data_length)
  {
    if (command_data_length > MAX_PROTOCOL_DATA_SIZE)
    {
      ESP_LOGW(TAG, "Command 0x%02X payload is too large: %u", command, command_data_length);
      return;
    }

    uint8_t message_au8_au8[MAX_MESSAGE_SIZE]{};
    message_au8_au8[0] = 0x00;
    message_au8_au8[1] = command;
    message_au8_au8[2] = command_data_length;
    for (uint8_t idx_u8 = 0; idx_u8 < command_data_length; idx_u8++)
    {
      message_au8_au8[3 + idx_u8] = command_data[idx_u8];
    }
    txMessage_(message_au8_au8);
  }

  // add prefix, second 0x07, checksum and postfix. Returns the length of the encoded frame.
  uint8_t encodeMessage_(uint8_t message_au8[], uint8_t txBuffer_au8[])
  {
    // add 2 bytes "START"
    // copy "COMMAND" and "SIZE"
    // copy "DATA" and double each 0x07 in data area
    // add "CHECKSUM" and escape it when needed
    // add 2 bytes "STOP"

    // Start
    txBuffer_au8[0] = 0x07;
    txBuffer_au8[1] = 0xF0;
    // Command
    txBuffer_au8[2] = message_au8[0];
    txBuffer_au8[3] = message_au8[1];
    // Size
    txBuffer_au8[4] = message_au8[2];
    // Data
    uint8_t idx_message_u8 = 3U;
    uint8_t idx_txBuffer_u8 = 5U;
    while (idx_message_u8 < message_au8[2] + 3)
    {
      // copy current byte
      txBuffer_au8[idx_txBuffer_u8] = message_au8[idx_message_u8];
      if (message_au8[idx_message_u8] == 0x07U)
      {
        // repeat 0x07 in TX
        idx_txBuffer_u8++;
        txBuffer_au8[idx_txBuffer_u8] = 0x07U;
      }
      idx_message_u8++;
      idx_txBuffer_u8++;
    }
    // Checksum
    const uint8_t checksum = calcChecksum_(message_au8);
    txBuffer_au8[idx_txBuffer_u8] = checksum;
    if (checksum == COMMAND_PREFIX)
    {
      idx_txBuffer_u8++;
      txBuffer_au8[idx_txBuffer_u8] = checksum;
    }
    idx_txBuffer_u8++;
    // Stop
    txBuffer_au8[idx_txBuffer_u8] = 0x07;
    idx_txBuffer_u8++;
    txBuffer_au8[idx_txBuffer_u8] = 0x0F;
    return idx_txBuffer_u8 + 1U;
  }

  // encode and transmit that data. With a proxy UART the frame is queued instead.
  void txMessage_(uint8_t message_au8[])
  {
    ESP_LOGVV(TAG, "TX: cmd: %02X size: %u", message_au8[1], message_au8[2]);

    uint8_t txBuffer_au8[MAX_MESSAGE_SIZE]; // TX buffer
    uint8_t idx_txBuffer_u8 = encodeMessage_(message_au8, txBuffer_au8);
    if (proxy_uart_ != nullptr)
    {
      // The bus is shared with the ComfoSense panel; wait for an idle gap.
      const uint8_t command = txBuffer_au8[3];
      const bool latest_wins = command == CMD_SET_LEVEL || command == CMD_SET_COMFORT_TEMPERATURE;
      if (!tx_queue_.push(txBuffer_au8, idx_txBuffer_u8, latest_wins))
      {
        ESP_LOGW(TAG, "Proxy: TX queue full, dropping cmd %02X", command);
      }
      return;
    }
    write_array(txBuffer_au8, idx_txBuffer_u8);
    flush();
  }

  void txACK_(/* tx_serial txPort_en */)
  {
    // send an ACK to Serial

    // todo transmit
    // switch (txPort_en)
    // {
    // case SERIAL_0_PC:
    //   Serial.write(0x07);
    //   Serial.write(0xF3);
    //   break;
    // case SERIAL_1_ComfoAir:
    //   Serial1.write(0x07);
    //   Serial1.write(0xF3);
    //   break;
    // case SERIAL_2_ComfoSense:
    //   Serial2.write(0x07);
    //   Serial2.write(0xF3);
    //   break;
    // default:
    //   break;
    // }

    write_byte(COMMAND_PREFIX);
    write_byte(COMMAND_HEAD_ACK);
    flush();
  }

  // RX & TX common checksum function
  uint8_t calcChecksum_(uint8_t message_au8[])
  {
    // Calculates checksum.
    // Doubled 0x07 has to be removed beforehand!
    uint16_t sum = 0xADU; // = 173
    uint8_t idx = 0U;

    while (idx <= message_au8[2] + 2)
    {
      // sum up all message bytes including
      // "Kommando", "Anzahl Daten" and "Daten"
      sum = sum + message_au8[idx];
      idx++;
    }

    // return only LSB of sum
    return ((uint8_t)sum);
  }

  // --- RX functions ---

  rx_status checkRx_(uint8_t rxBuffer[], uint8_t *index_u8, uint8_t rc)
  {
    /* Manage and analyze rx bytes.
      return indicates ACK or MESSAGE.
    */
    rx_status return_val = RX_STATUS_DEFAULT;

    // store received byte in array
    rxBuffer[*index_u8] = rc;

    // analyze array

    // Two possible message beginnings:
    // 1. 0x07 0xF3 -> is an ACK
    // 2. 0x07 0xF0 -> is the beginning of an actual message which has to end with 0x07 0x0F

    if (*index_u8 == 1U && rxBuffer[0] == 0x07 && rxBuffer[1] == 0xF3)
    {
      // received ACK
      return_val = RX_STATUS_RECEIVED_ACK;
      *index_u8 = 0U;
    }
    else if (*index_u8 > 1U && rxBuffer[*index_u8 - 2] != 0x07 && rxBuffer[*index_u8 - 1] == 0x07 &&
             rxBuffer[*index_u8] == 0xF3)
    {
      // received ACK
      return_val = RX_STATUS_RECEIVED_ACK;
      *index_u8 = 0U;
    }
    else if (*index_u8 > 1U && rxBuffer[*index_u8 - 2] != 0x07 && rxBuffer[*index_u8 - 1] == 0x07 &&
             rxBuffer[*index_u8] == 0xF0)
    {
      // This has to be the start of a new message.
      //
      // This assumption is true as long as there is no Kommando 0x.. 0x07
      // followed by a size of 0xF0. But afaik the Kommando 0x00 0x07 is of
      // size 3 and other 0x.. 0x07 commands are unknown.
      rxBuffer[0] = 0x07;
      rxBuffer[1] = 0xF0;
      *index_u8 = 2U;
      return_val = RX_STATUS_RECEIVED_START_OF_MESSAGE;
    }
    else if (*index_u8 == 4U && rxBuffer[0] == 0x07 && rxBuffer[1] == 0xF0 && rxBuffer[4] > MAX_PROTOCOL_DATA_SIZE)
    {
      ESP_LOGW(TAG, "RX payload is too large: %u", rxBuffer[4]);
      return_val = RX_STATUS_RECEIVED_INVALID_MESSAGE;
      *index_u8 = 0U;
    }
    else if (*index_u8 >= 7U && rxBuffer[0] == 0x07 && rxBuffer[1] == 0xF0 && rxBuffer[*index_u8 - 1] == 0x07 &&
             rxBuffer[*index_u8] == 0x0F)
    {
      // new message received:
      // remove "START" and "STOP" plus
      // eliminate double 0x07 within data area
      const uint8_t checksum_index = isolateRxMessage_(rxBuffer, *index_u8);
      const uint8_t expected_checksum_index = rxBuffer[2] + 3U;

      if (checksum_index != expected_checksum_index)
      {
        ESP_LOGW(TAG, "RX length mismatch for command 0x%02X: expected %u data bytes", rxBuffer[1], rxBuffer[2]);
        return_val = RX_STATUS_RECEIVED_INVALID_MESSAGE;
        *index_u8 = 0U;
      }
      else if (calcChecksum_(rxBuffer) == rxBuffer[checksum_index])
      {
        // message is valid
        return_val = RX_STATUS_RECEIVED_MESSAGE;
        *index_u8 = 0U;
      }
      else
      {
        // message invalid
        return_val = RX_STATUS_RECEIVED_INVALID_MESSAGE;
        ESP_LOGD(TAG, "RX checksum mismatch for command 0x%02X: calculated %02X, received %02X", rxBuffer[1],
                 calcChecksum_(rxBuffer), rxBuffer[checksum_index]);
        *index_u8 = 0U;
      }
    }
    else
    {
      return_val = RX_STATUS_WRAPPED_BUFFER_INDEX;
      *index_u8 = (1 + *index_u8) % MAX_MESSAGE_SIZE;
    }
    return return_val;
  }

  uint8_t isolateRxMessage_(uint8_t rxBuffer[], uint8_t size)
  {
    // remove "START" (2 bytes)
    // copy "COMMAND" and "SIZE" (3 bytes)
    // copy "DATA" while removing doubled 0x07 in data area if applicable
    // copy "Checksum"
    // remove/ignore "STOP" (2 bytes)
    // return resulting size of array
    uint8_t read_idx_u8 = 2U; // skip/remove/ignore "START"-bytes
    uint8_t write_idx_u8 = 0U;

    // copy "COMAND" and "SIZE"
    while (read_idx_u8 < 5U)
    {
      rxBuffer[write_idx_u8] = rxBuffer[read_idx_u8];
      read_idx_u8++;
      write_idx_u8++;
    }

    // copy "DATA" while removing doubled 0x07 in data area if applicable
    while (((write_idx_u8 - 3U) < rxBuffer[2]) && (read_idx_u8 < size - 2))
    {
      rxBuffer[write_idx_u8] = rxBuffer[read_idx_u8];
      if ((rxBuffer[read_idx_u8] == 0x07U) && (rxBuffer[read_idx_u8 + 1U] == 0x07U))
      {
        // skip next byte which is expected to be 0x07 according to reverse
        // engineered protocol
        read_idx_u8++;
      }
      read_idx_u8++;
      write_idx_u8++;
    }

    // copy "Checksum". Some implementations send a checksum of 0x07 raw,
    // while others escape it like payload data. Accept both forms.
    if (read_idx_u8 == size - 3U && rxBuffer[read_idx_u8] == COMMAND_PREFIX &&
        rxBuffer[read_idx_u8 + 1U] == COMMAND_PREFIX)
    {
      rxBuffer[write_idx_u8] = COMMAND_PREFIX;
    }
    else if (read_idx_u8 == size - 2U)
    {
      rxBuffer[write_idx_u8] = rxBuffer[read_idx_u8];
    }
    else
    {
      return MAX_MESSAGE_SIZE;
    }
    // return resulting size of array
    return (write_idx_u8);
  }

  void parseRxMessage_(uint8_t *message_au8)
  {
    status_clear_warning();
    // message_au8[0] = 0x00
    // message_au8[1] = command byte
    // message_au8[2] = data_length
    // message_au8[3] = first byte of data

    uint8_t msg_command_u8 = message_au8[COMMAND_IDX_MSG_ID - 2]; // -2 because prefix is removed from data
    uint8_t msg_length_u8 = message_au8[COMMAND_IDX_PROTOCOL_LENGTH - 2];
    uint8_t *msg_data = &message_au8[COMMAND_IDX_PROTOCOL_DATA_AREA - 2];

    switch (msg_command_u8)
    {
    case RES_GET_BOOTLOADER_VERSION:
      memcpy(bootloader_version_, msg_data, std::min<size_t>(msg_length_u8, sizeof(bootloader_version_)));
      break;
    case RES_GET_FIRMWARE_VERSION:
      memcpy(firmware_version_, msg_data, std::min<size_t>(msg_length_u8, sizeof(firmware_version_)));
      break;
    case RES_GET_CONNECTOR_BOARD_VERSION:
      memcpy(connector_board_version_, msg_data, std::min<size_t>(msg_length_u8, sizeof(connector_board_version_)));
      break;
    case RES_GET_FAN_STATUS:
    {
      if (intake_fan_speed != nullptr)
      {
        intake_fan_speed->publish_state(msg_data[0]);
      }
      if (exhaust_fan_speed != nullptr)
      {
        exhaust_fan_speed->publish_state(msg_data[1]);
      }
      if (intake_fan_speed_rpm != nullptr)
      {
        const uint16_t intake_period = static_cast<uint16_t>(msg_data[2] << 8 | msg_data[3]);
        intake_fan_speed_rpm->publish_state(intake_period == 0 ? 0 : static_cast<int>(1875000.0f / intake_period));
      }
      if (exhaust_fan_speed_rpm != nullptr)
      {
        const uint16_t exhaust_period = static_cast<uint16_t>(msg_data[4] << 8 | msg_data[5]);
        exhaust_fan_speed_rpm->publish_state(exhaust_period == 0 ? 0 : static_cast<int>(1875000.0f / exhaust_period));
      }
      break;
    }
    case RES_GET_VALVE_STATUS:
    {
      if (bypass_valve != nullptr)
      {
        bypass_valve->publish_state(msg_data[0]);
      }
      if (bypass_valve_open != nullptr)
      {
        bypass_valve_open->publish_state(msg_data[0] != 0);
      }
      // Value 2 means unknown; do not publish it as a false closed state.
      if (preheating_state != nullptr && msg_data[1] <= 1)
      {
        preheating_state->publish_state(msg_data[1] == 1);
      }
      if (motor_current_bypass != nullptr)
      {
        motor_current_bypass->publish_state(msg_data[2]);
      }
      if (motor_current_preheating != nullptr)
      {
        motor_current_preheating->publish_state(msg_data[3]);
      }
      break;
    }
    case RES_GET_BYPASS_CONTROL_STATUS:
    {
      if (bypass_factor != nullptr)
      {
        bypass_factor->publish_state(msg_data[2]);
      }
      if (bypass_step != nullptr)
      {
        bypass_step->publish_state(msg_data[3]);
      }
      if (bypass_correction != nullptr)
      {
        bypass_correction->publish_state(msg_data[4]);
      }
      if (summer_mode != nullptr)
      {
        summer_mode->publish_state(msg_data[6] != 0);
      }
      break;
    }
    case RES_GET_TEMPERATURE_STATUS:
    {
      // T1 / outside air
      if (outside_air_temperature != nullptr)
      {
        outside_air_temperature->publish_state((float)msg_data[0] / 2.0f - 20.0f);
      }
      // T2 / supply air
      if (supply_air_temperature != nullptr)
      {
        supply_air_temperature->publish_state((float)msg_data[1] / 2.0f - 20.0f);
      }
      // T3 / return air
      if (return_air_temperature != nullptr)
      {
        return_air_temperature->publish_state((float)msg_data[2] / 2.0f - 20.0f);
      }
      // T4 / exhaust air
      if (exhaust_air_temperature != nullptr)
      {
        exhaust_air_temperature->publish_state((float)msg_data[3] / 2.0f - 20.0f);
      }
      break;
    }
    case RES_GET_SENSOR_DATA:
    {
      if (enthalpy_temperature != nullptr)
      {
        enthalpy_temperature->publish_state((float)msg_data[0] / 2.0f - 20.0f);
      }
      break;
    }
    case RES_GET_VENTILATION_LEVEL:
    {
      ESP_LOGD(TAG, "Level %02X", msg_data[8]);
      ESP_LOGV(
          TAG, "Off ab %i - Off zu %i - Low ab %i - Low zu %i - Middle ab %i - Middle zu %i - High ab %i - High zu %i",
          msg_data[0], msg_data[3], msg_data[1], msg_data[4], msg_data[2], msg_data[5], msg_data[10], msg_data[11]);

      if (return_air_level != nullptr)
      {
        return_air_level->publish_state(msg_data[6]);
      }
      if (supply_air_level != nullptr)
      {
        supply_air_level->publish_state(msg_data[7]);
      }

      apply_level_(msg_data[8]);

      if (return_air_level_absent != nullptr)
        return_air_level_absent->publish_state(msg_data[0]);
      if (return_air_level_low != nullptr)
        return_air_level_low->publish_state(msg_data[1]);
      if (return_air_level_medium != nullptr)
        return_air_level_medium->publish_state(msg_data[2]);
      if (return_air_level_high != nullptr)
        return_air_level_high->publish_state(msg_data[10]);
      if (supply_air_level_absent != nullptr)
        supply_air_level_absent->publish_state(msg_data[3]);
      if (supply_air_level_low != nullptr)
        supply_air_level_low->publish_state(msg_data[4]);
      if (supply_air_level_medium != nullptr)
        supply_air_level_medium->publish_state(msg_data[5]);
      if (supply_air_level_high != nullptr)
        supply_air_level_high->publish_state(msg_data[11]);

      publish_state();

      // Supply air fan active (1 = active / 0 = inactive)
      if (supply_fan_active != nullptr)
      {
        supply_fan_active->publish_state(msg_data[9] == 1);
      }
      break;
    }
    case RES_GET_FAULTS:
    {
      const uint8_t status = msg_data[8];
      if (filter_status != nullptr)
      {
        filter_status->publish_state(status == 0 ? "Ok" : (status == 1 ? "Full" : "Unknown"));
      }
      // Keep the binary sensor unknown when the protocol does not report a valid OK/full value.
      if (filter_status_full != nullptr && status <= 1)
      {
        filter_status_full->publish_state(status == 1);
      }
      break;
    }
    case RES_GET_TEMPERATURES:
    {
      // comfort temperature
      target_temperature = (float)msg_data[0] / 2.0f - 20.0f;
      publish_state();

      // T1 / outside air
      if (outside_air_temperature != nullptr && msg_data[5] & 0x01)
      {
        outside_air_temperature->publish_state((float)msg_data[1] / 2.0f - 20.0f);
      }
      // T2 / supply air
      if (supply_air_temperature != nullptr && msg_data[5] & 0x02)
      {
        supply_air_temperature->publish_state((float)msg_data[2] / 2.0f - 20.0f);
      }
      // T3 / exhaust air
      if (return_air_temperature != nullptr && msg_data[5] & 0x04)
      {
        return_air_temperature->publish_state((float)msg_data[3] / 2.0f - 20.0f);
        current_temperature = (float)msg_data[3] / 2.0f - 20.0f;
      }
      // T4 / continued air
      if (exhaust_air_temperature != nullptr && msg_data[5] & 0x08)
      {
        exhaust_air_temperature->publish_state((float)msg_data[4] / 2.0f - 20.0f);
      }
      // EWT
      if (ewt_temperature != nullptr && msg_data[5] & 0x10)
      {
        ewt_temperature->publish_state((float)msg_data[6] / 2.0f - 20.0f);
      }
      // reheating
      if (reheating_temperature != nullptr && msg_data[5] & 0x20)
      {
        reheating_temperature->publish_state((float)msg_data[7] / 2.0f - 20.0f);
      }
      // kitchen hood
      if (kitchen_hood_temperature != nullptr && msg_data[5] & 0x40)
      {
        kitchen_hood_temperature->publish_state((float)msg_data[8] / 2.0f - 20.0f);
      }
      break;
    }
    case RES_GET_STATUS:
    {
      if (preheating_present != nullptr)
      {
        preheating_present->publish_state(msg_data[0]);
      }

      if (bypass_present != nullptr)
      {
        bypass_present->publish_state(msg_data[1]);
      }

      if (type != nullptr)
      {
        type->publish_state(msg_data[2] == 1 ? "Left" : (msg_data[2] == 2 ? "Right" : "Unknown"));
      }

      publish_size_entities_(msg_data[3]);

      if (options_present != nullptr)
      {
        options_present->publish_state(msg_data[4]);
      }

      if (fireplace_present != nullptr)
      {
        fireplace_present->publish_state(msg_data[4] & 0x01);
      }

      if (kitchen_hood_present != nullptr)
      {
        kitchen_hood_present->publish_state(msg_data[4] & 0x02);
      }

      if (postheating_present != nullptr)
      {
        postheating_present->publish_state(msg_data[4] & 0x04);
      }

      if (postheating_pwm_mode_present != nullptr)
      {
        postheating_pwm_mode_present->publish_state(msg_data[4] & 0x40);
      }

      if (p10_active != nullptr)
      {
        p10_active->publish_state(msg_data[6] & 0x01);
      }

      if (p11_active != nullptr)
      {
        p11_active->publish_state(msg_data[6] & 0x02);
      }

      if (p12_active != nullptr)
      {
        p12_active->publish_state(msg_data[6] & 0x04);
      }

      if (p13_active != nullptr)
      {
        p13_active->publish_state(msg_data[6] & 0x08);
      }

      if (p14_active != nullptr)
      {
        p14_active->publish_state(msg_data[6] & 0x10);
      }

      if (p15_active != nullptr)
      {
        p15_active->publish_state(msg_data[6] & 0x20);
      }

      if (p16_active != nullptr)
      {
        p16_active->publish_state(msg_data[6] & 0x40);
      }

      if (p17_active != nullptr)
      {
        p17_active->publish_state(msg_data[6] & 0x80);
      }

      if (p18_active != nullptr)
      {
        p18_active->publish_state(msg_data[7] & 0x01);
      }

      if (p19_active != nullptr)
      {
        p19_active->publish_state(msg_data[7] & 0x02);
      }

      if (p90_active != nullptr)
      {
        p90_active->publish_state(msg_data[8] & 0x01);
      }

      if (p91_active != nullptr)
      {
        p91_active->publish_state(msg_data[8] & 0x02);
      }

      if (p92_active != nullptr)
      {
        p92_active->publish_state(msg_data[8] & 0x04);
      }

      if (p93_active != nullptr)
      {
        p93_active->publish_state(msg_data[8] & 0x08);
      }

      if (p94_active != nullptr)
      {
        p94_active->publish_state(msg_data[8] & 0x10);
      }

      if (p95_active != nullptr)
      {
        p95_active->publish_state(msg_data[8] & 0x20);
      }

      if (p96_active != nullptr)
      {
        p96_active->publish_state(msg_data[8] & 0x40);
      }

      if (p97_active != nullptr)
      {
        p97_active->publish_state(msg_data[8] & 0x80);
      }

      if (enthalpy_present != nullptr)
      {
        enthalpy_present->publish_state(msg_data[9]);
      }

      if (ewt_present != nullptr)
      {
        ewt_present->publish_state(msg_data[10]);
      }

      if (msg_length_u8 >= 11)
      {
        status_payload_[0] = msg_data[0];
        status_payload_[1] = msg_data[1];
        status_payload_[2] = msg_data[2];
        status_payload_[3] = msg_data[3];
        status_payload_[4] = msg_data[4];
        status_payload_[5] = msg_data[5];
        status_payload_[6] = msg_data[9];
        status_payload_[7] = msg_data[10];
        status_payload_valid_ = true;
      }
      else
      {
        status_payload_valid_ = false;
      }
      break;
    }
    case RES_GET_OPERATION_HOURS:
    {
      if (level0_hours != nullptr)
      {
        level0_hours->publish_state((msg_data[0] << 16) | (msg_data[1] << 8) | msg_data[2]);
      }

      if (level1_hours != nullptr)
      {
        level1_hours->publish_state((msg_data[3] << 16) | (msg_data[4] << 8) | msg_data[5]);
      }

      if (level2_hours != nullptr)
      {
        level2_hours->publish_state((msg_data[6] << 16) | (msg_data[7] << 8) | msg_data[8]);
      }

      if (level3_hours != nullptr)
      {
        level3_hours->publish_state((msg_data[17] << 16) | (msg_data[18] << 8) | msg_data[19]);
      }

      if (frost_protection_hours != nullptr)
      {
        frost_protection_hours->publish_state((msg_data[9] << 8) | msg_data[10]);
      }

      if (bypass_open_hours != nullptr)
      {
        bypass_open_hours->publish_state((msg_data[13] << 8) | msg_data[14]);
      }

      if (preheating_hours != nullptr)
      {
        preheating_hours->publish_state((msg_data[11] << 8) | msg_data[12]);
      }

      if (filter_hours != nullptr)
      {
        filter_hours->publish_state((msg_data[15] << 8) | msg_data[16]);
      }
      break;
    }
    case RES_GET_PREHEATING_STATUS:
    {
      if (preheating_valve != nullptr)
      {
        std::string name_preheating_valve;
        switch (msg_data[0])
        {
        case 0:
          name_preheating_valve = "Closed";
          break;

        case 1:
          name_preheating_valve = "Open";
          break;

        default:
          name_preheating_valve = "Unknown";
          break;
        }
        preheating_valve->publish_state(name_preheating_valve);
      }

      if (frost_protection_active != nullptr)
      {
        frost_protection_active->publish_state(msg_data[1] != 0);
      }

      if (preheating_active != nullptr && msg_data[2] <= 1)
      {
        preheating_active->publish_state(msg_data[2] == 1);
      }

      if (frost_protection_minutes != nullptr)
      {
        frost_protection_minutes->publish_state((msg_data[3] << 8) | msg_data[4]);
      }

      if (frost_protection_level != nullptr)
      {
        std::string name_frost_protection_level;
        switch (msg_data[5])
        {
        case 0:
          name_frost_protection_level = "GuaranteedProtection";
          break;

        case 1:
          name_frost_protection_level = "HighProtection";
          break;

        case 2:
          name_frost_protection_level = "NominalProtection";
          break;

        case 3:
          name_frost_protection_level = "Economy";
          break;

        default:
          name_frost_protection_level = "Unknown";
          break;
        }
        frost_protection_level->publish_state(name_frost_protection_level);
      }
      break;
    }
    case RES_GET_TIME_DELAY:
    {
      if (bathroom_switch_on_delay_minutes != nullptr)
      {
        bathroom_switch_on_delay_minutes->publish_state(msg_data[0]);
      }

      if (bathroom_switch_off_delay_minutes != nullptr)
      {
        bathroom_switch_off_delay_minutes->publish_state(msg_data[1]);
      }

      if (l1_switch_off_delay_minutes != nullptr)
      {
        l1_switch_off_delay_minutes->publish_state(msg_data[2]);
      }

      if (boost_ventilation_minutes != nullptr)
      {
        boost_ventilation_minutes->publish_state(msg_data[3]);
      }

      if (filter_warning_weeks != nullptr)
      {
        filter_warning_weeks->publish_state(msg_data[4]);
      }

      if (rf_high_time_short_minutes != nullptr)
      {
        rf_high_time_short_minutes->publish_state(msg_data[5]);
      }

      if (rf_high_time_long_minutes != nullptr)
      {
        rf_high_time_long_minutes->publish_state(msg_data[6]);
      }

      if (extractor_hood_switch_off_delay_minutes != nullptr)
      {
        extractor_hood_switch_off_delay_minutes->publish_state(msg_data[7]);
      }
      break;
    }
    }
  }

  // --- getter ---

  void get_fan_status_()
  {
    if (intake_fan_speed != nullptr || exhaust_fan_speed != nullptr || intake_fan_speed_rpm != nullptr ||
        exhaust_fan_speed_rpm != nullptr)
    {
      ESP_LOGD(TAG, "getting fan status");
      write_command_(CMD_GET_FAN_STATUS, nullptr, 0);
    }
  }

  void get_valve_status_()
  {
    if (bypass_valve != nullptr || bypass_valve_open != nullptr || preheating_state != nullptr)
    {
      ESP_LOGD(TAG, "getting valve status");
      write_command_(CMD_GET_VALVE_STATUS, nullptr, 0);
    }
  }

  void get_error_status_()
  {
    if (filter_status != nullptr || filter_status_full != nullptr)
    {
      ESP_LOGD(TAG, "getting error status");
      write_command_(CMD_GET_FAULTS, nullptr, 0);
    }
  }

  void get_bypass_control_status_()
  {
    if (bypass_factor != nullptr || bypass_step != nullptr || bypass_correction != nullptr || summer_mode != nullptr)
    {
      ESP_LOGD(TAG, "getting bypass control");
      write_command_(CMD_GET_BYPASS_CONTROL_STATUS, nullptr, 0);
    }
  }

  void get_temperature_()
  {
    if (outside_air_temperature != nullptr || supply_air_temperature != nullptr || return_air_temperature != nullptr ||
        outside_air_temperature != nullptr)
    {
      ESP_LOGD(TAG, "getting temperature");
      write_command_(CMD_GET_TEMPERATURE_STATUS, nullptr, 0);
    }
  }

  void get_sensor_data_()
  {
    if (enthalpy_temperature != nullptr)
    {
      ESP_LOGD(TAG, "getting sensor data");
      write_command_(CMD_GET_SENSOR_DATA, nullptr, 0);
    }
  }

  void get_ventilation_level_()
  {
    ESP_LOGD(TAG, "getting ventilation level");
    write_command_(CMD_GET_VENTILATION_LEVEL, nullptr, 0);
  }

  void get_temperatures_()
  {
    ESP_LOGD(TAG, "getting temperatures");
    write_command_(CMD_GET_TEMPERATURES, nullptr, 0);
  }

  void get_operation_hours_()
  {
    ESP_LOGD(TAG, "getting operation hours");
    write_command_(CMD_GET_OPERATION_HOURS, nullptr, 0);
  }

  void get_preheating_status_()
  {
    ESP_LOGD(TAG, "getting preheating status");
    write_command_(CMD_GET_PREHEATING_STATUS, nullptr, 0);
  }

  void get_time_delay_()
  {
    ESP_LOGD(TAG, "getting time delay");
    write_command_(CMD_GET_TIME_DELAY, nullptr, 0);
  }

  void publish_size_entities_(uint8_t raw_size);
  const char *unit_size_text_label_(uint8_t raw_size) const;
  const char *unit_size_option_label_(uint8_t raw_size) const;

  uint8_t caRxBuffer_au8[MAX_MESSAGE_SIZE]{};
  uint8_t caRxIdx_u8 = 0;

  // Proxy mode state (unused when no proxy UART is configured).
  uart::UARTComponent *proxy_uart_{nullptr};
  uint8_t csRxBuffer_au8[MAX_MESSAGE_SIZE]{};
  uint8_t csRxIdx_u8 = 0;
  ProxyBusArbiter arbiter_;
  ProxyTxQueue tx_queue_;
  ProxyHoldBuffer hold_;
  ProxyTxQueue::Frame inflight_;
  uint8_t inflight_attempts_{0};
  bool inflight_expects_response_{false};
  bool retry_pending_{false};
  HighFrequencyLoopRequester high_freq_;
  bool log_raw_bytes_{false};
  bool test_fake_unit_reply_{false};
  bool test_panel_frame_{false};
  bool test_ack_panel_frames_{false};
  uint32_t last_test_panel_frame_ms_{0};
  uint32_t boot_ms_{0};
  bool unit_write_only_{false};
  bool homekit_climate_{false};
  bool write_only_started_{false};
  bool write_only_level_known_{false};
  uint32_t write_only_level_sent_ms_{0};
  WriteOnlyState write_only_state_;
  ESPPreferenceObject write_only_pref_;
  bool unit_byte_seen_{false};
  WiringCheck wiring_check_;
  uint8_t wiring_check_pins_[WiringCheck::PIN_COUNT]{};
  bool wiring_check_pending_{false};
  bool wiring_check_running_{false};
  bool wiring_check_done_{false};
  int wiring_check_tick_{0};
  uint32_t wiring_check_tick_ms_{0};
  RawByteCollector unit_raw_;
  RawByteCollector panel_raw_;

  int8_t update_counter_{-10};
  uint8_t status_payload_[8]{0};
  bool status_payload_valid_{false};
  uint8_t current_unit_size_{0};
  ComfoAirSizeSelect *size_select_{nullptr};

  uint8_t bootloader_version_[13]{0};
  uint8_t firmware_version_[13]{0};
  uint8_t connector_board_version_[14]{0};
  const char *name{0};

public:
  text_sensor::TextSensor *type{nullptr};
  text_sensor::TextSensor *size{nullptr};
  text_sensor::TextSensor *filter_status{nullptr};
  binary_sensor::BinarySensor *filter_status_full{nullptr};
  text_sensor::TextSensor *frost_protection_level{nullptr};
  text_sensor::TextSensor *preheating_valve{nullptr};
  sensor::Sensor *intake_fan_speed{nullptr};
  sensor::Sensor *exhaust_fan_speed{nullptr};
  sensor::Sensor *intake_fan_speed_rpm{nullptr};
  sensor::Sensor *exhaust_fan_speed_rpm{nullptr};
  sensor::Sensor *ventilation_level{nullptr};
  sensor::Sensor *outside_air_temperature{nullptr};
  sensor::Sensor *supply_air_temperature{nullptr};
  sensor::Sensor *return_air_temperature{nullptr};
  sensor::Sensor *exhaust_air_temperature{nullptr};
  sensor::Sensor *enthalpy_temperature{nullptr};
  sensor::Sensor *ewt_temperature{nullptr};
  sensor::Sensor *reheating_temperature{nullptr};
  sensor::Sensor *kitchen_hood_temperature{nullptr};
  sensor::Sensor *return_air_level{nullptr};
  sensor::Sensor *supply_air_level{nullptr};
  sensor::Sensor *bypass_factor{nullptr};
  sensor::Sensor *bypass_step{nullptr};
  sensor::Sensor *bypass_correction{nullptr};
  sensor::Sensor *bypass_open_hours{nullptr};
  sensor::Sensor *motor_current_bypass{nullptr};
  sensor::Sensor *motor_current_preheating{nullptr};
  sensor::Sensor *preheating_hours{nullptr};
  sensor::Sensor *level0_hours{nullptr};
  sensor::Sensor *level1_hours{nullptr};
  sensor::Sensor *level2_hours{nullptr};
  sensor::Sensor *level3_hours{nullptr};
  binary_sensor::BinarySensor *frost_protection_active{nullptr};
  sensor::Sensor *frost_protection_hours{nullptr};
  sensor::Sensor *frost_protection_minutes{nullptr};
  sensor::Sensor *filter_hours{nullptr};
  sensor::Sensor *bypass_valve{nullptr};
  binary_sensor::BinarySensor *bypass_present{nullptr};
  binary_sensor::BinarySensor *enthalpy_present{nullptr};
  binary_sensor::BinarySensor *ewt_present{nullptr};
  binary_sensor::BinarySensor *preheating_present{nullptr};
  binary_sensor::BinarySensor *options_present{nullptr};
  binary_sensor::BinarySensor *fireplace_present{nullptr};
  binary_sensor::BinarySensor *kitchen_hood_present{nullptr};
  binary_sensor::BinarySensor *postheating_present{nullptr};
  binary_sensor::BinarySensor *postheating_pwm_mode_present{nullptr};
  binary_sensor::BinarySensor *bypass_valve_open{nullptr};
  binary_sensor::BinarySensor *preheating_state{nullptr};
  binary_sensor::BinarySensor *preheating_active{nullptr};
  binary_sensor::BinarySensor *summer_mode{nullptr};
  binary_sensor::BinarySensor *supply_fan_active{nullptr};
  binary_sensor::BinarySensor *p10_active{nullptr};
  binary_sensor::BinarySensor *p11_active{nullptr};
  binary_sensor::BinarySensor *p12_active{nullptr};
  binary_sensor::BinarySensor *p13_active{nullptr};
  binary_sensor::BinarySensor *p14_active{nullptr};
  binary_sensor::BinarySensor *p15_active{nullptr};
  binary_sensor::BinarySensor *p16_active{nullptr};
  binary_sensor::BinarySensor *p17_active{nullptr};
  binary_sensor::BinarySensor *p18_active{nullptr};
  binary_sensor::BinarySensor *p19_active{nullptr};
  binary_sensor::BinarySensor *p90_active{nullptr};
  binary_sensor::BinarySensor *p91_active{nullptr};
  binary_sensor::BinarySensor *p92_active{nullptr};
  binary_sensor::BinarySensor *p93_active{nullptr};
  binary_sensor::BinarySensor *p94_active{nullptr};
  binary_sensor::BinarySensor *p95_active{nullptr};
  binary_sensor::BinarySensor *p96_active{nullptr};
  binary_sensor::BinarySensor *p97_active{nullptr};
  sensor::Sensor *bathroom_switch_on_delay_minutes{nullptr};
  sensor::Sensor *bathroom_switch_off_delay_minutes{nullptr};
  sensor::Sensor *l1_switch_off_delay_minutes{nullptr};
  sensor::Sensor *boost_ventilation_minutes{nullptr};
  sensor::Sensor *filter_warning_weeks{nullptr};
  sensor::Sensor *rf_high_time_short_minutes{nullptr};
  sensor::Sensor *rf_high_time_long_minutes{nullptr};
  sensor::Sensor *extractor_hood_switch_off_delay_minutes{nullptr};
  ComfoAirNumber *return_air_level_absent{nullptr};
  ComfoAirNumber *return_air_level_low{nullptr};
  ComfoAirNumber *return_air_level_medium{nullptr};
  ComfoAirNumber *return_air_level_high{nullptr};
  ComfoAirNumber *supply_air_level_absent{nullptr};
  ComfoAirNumber *supply_air_level_low{nullptr};
  ComfoAirNumber *supply_air_level_medium{nullptr};
  ComfoAirNumber *supply_air_level_high{nullptr};
  ComfoAirSyncButton *sync_button_{nullptr};

  // public functions

  void set_type(text_sensor::TextSensor *type) { this->type = type; };
  void set_size(text_sensor::TextSensor *size) { this->size = size; };
  void set_intake_fan_speed(sensor::Sensor *intake_fan_speed) { this->intake_fan_speed = intake_fan_speed; };
  void set_exhaust_fan_speed(sensor::Sensor *exhaust_fan_speed) { this->exhaust_fan_speed = exhaust_fan_speed; };
  void set_intake_fan_speed_rpm(sensor::Sensor *intake_fan_speed_rpm)
  {
    this->intake_fan_speed_rpm = intake_fan_speed_rpm;
    this->intake_fan_speed_rpm->set_accuracy_decimals(0);
  };
  void set_exhaust_fan_speed_rpm(sensor::Sensor *exhaust_fan_speed_rpm)
  {
    this->exhaust_fan_speed_rpm = exhaust_fan_speed_rpm;
    this->exhaust_fan_speed_rpm->set_accuracy_decimals(0);
  };
  void set_ventilation_level(sensor::Sensor *ventilation_level) { this->ventilation_level = ventilation_level; };
  void set_bypass_valve(sensor::Sensor *bypass_valve) { this->bypass_valve = bypass_valve; };
  void set_bypass_present(binary_sensor::BinarySensor *bypass_present) { this->bypass_present = bypass_present; };
  void set_enthalpy_present(binary_sensor::BinarySensor *enthalpy_present)
  {
    this->enthalpy_present = enthalpy_present;
  };
  void set_ewt_present(binary_sensor::BinarySensor *ewt_present) { this->ewt_present = ewt_present; };
  void set_preheating_present(binary_sensor::BinarySensor *preheating_present)
  {
    this->preheating_present = preheating_present;
  };
  void set_options_present(binary_sensor::BinarySensor *options_present) { this->options_present = options_present; };
  void set_fireplace_present(binary_sensor::BinarySensor *fireplace_present)
  {
    this->fireplace_present = fireplace_present;
  };
  void set_kitchen_hood_present(binary_sensor::BinarySensor *kitchen_hood_present)
  {
    this->kitchen_hood_present = kitchen_hood_present;
  };
  void set_postheating_present(binary_sensor::BinarySensor *postheating_present)
  {
    this->postheating_present = postheating_present;
  };
  void set_postheating_pwm_mode_present(binary_sensor::BinarySensor *postheating_pwm_mode_present)
  {
    this->postheating_pwm_mode_present = postheating_pwm_mode_present;
  };
  void set_bypass_valve_open(binary_sensor::BinarySensor *bypass_valve_open)
  {
    this->bypass_valve_open = bypass_valve_open;
  };
  void set_preheating_state(binary_sensor::BinarySensor *preheating_state)
  {
    this->preheating_state = preheating_state;
  };
  void set_preheating_active(binary_sensor::BinarySensor *preheating_active)
  {
    this->preheating_active = preheating_active;
  };
  void set_outside_air_temperature(sensor::Sensor *outside_air_temperature)
  {
    this->outside_air_temperature = outside_air_temperature;
  };
  void set_supply_air_temperature(sensor::Sensor *supply_air_temperature)
  {
    this->supply_air_temperature = supply_air_temperature;
  };
  void set_return_air_temperature(sensor::Sensor *return_air_temperature)
  {
    this->return_air_temperature = return_air_temperature;
  };
  void set_exhaust_air_temperature(sensor::Sensor *exhaust_air_temperature)
  {
    this->exhaust_air_temperature = exhaust_air_temperature;
  };
  void set_enthalpy_temperature(sensor::Sensor *enthalpy_temperature)
  {
    this->enthalpy_temperature = enthalpy_temperature;
  };
  void set_ewt_temperature(sensor::Sensor *ewt_temperature) { this->ewt_temperature = ewt_temperature; };
  void set_reheating_temperature(sensor::Sensor *reheating_temperature)
  {
    this->reheating_temperature = reheating_temperature;
  };
  void set_kitchen_hood_temperature(sensor::Sensor *kitchen_hood_temperature)
  {
    this->kitchen_hood_temperature = kitchen_hood_temperature;
  };
  void set_return_air_level(sensor::Sensor *return_air_level) { this->return_air_level = return_air_level; };
  void set_supply_air_level(sensor::Sensor *supply_air_level) { this->supply_air_level = supply_air_level; };
  void set_supply_fan_active(binary_sensor::BinarySensor *supply_fan_active)
  {
    this->supply_fan_active = supply_fan_active;
  };
  void set_filter_status(text_sensor::TextSensor *filter_status) { this->filter_status = filter_status; };
  void set_filter_status_full(binary_sensor::BinarySensor *filter_status_full)
  {
    this->filter_status_full = filter_status_full;
  };
  void set_bypass_factor(sensor::Sensor *bypass_factor) { this->bypass_factor = bypass_factor; };
  void set_bypass_step(sensor::Sensor *bypass_step) { this->bypass_step = bypass_step; };
  void set_bypass_correction(sensor::Sensor *bypass_correction) { this->bypass_correction = bypass_correction; };
  void set_bypass_open_hours(sensor::Sensor *bypass_open_hours) { this->bypass_open_hours = bypass_open_hours; };
  void set_motor_current_bypass(sensor::Sensor *motor_current_bypass)
  {
    this->motor_current_bypass = motor_current_bypass;
  };
  void set_motor_current_preheating(sensor::Sensor *motor_current_preheating)
  {
    this->motor_current_preheating = motor_current_preheating;
  };
  void set_preheating_hours(sensor::Sensor *preheating_hours) { this->preheating_hours = preheating_hours; };
  void set_preheating_valve(text_sensor::TextSensor *preheating_valve) { this->preheating_valve = preheating_valve; };
  void set_level0_hours(sensor::Sensor *level0_hours) { this->level0_hours = level0_hours; };
  void set_level1_hours(sensor::Sensor *level1_hours) { this->level1_hours = level1_hours; };
  void set_level2_hours(sensor::Sensor *level2_hours) { this->level2_hours = level2_hours; };
  void set_level3_hours(sensor::Sensor *level3_hours) { this->level3_hours = level3_hours; };
  void set_frost_protection_active(binary_sensor::BinarySensor *frost_protection_active)
  {
    this->frost_protection_active = frost_protection_active;
  };
  void set_frost_protection_hours(sensor::Sensor *frost_protection_hours)
  {
    this->frost_protection_hours = frost_protection_hours;
  };
  void set_frost_protection_minutes(sensor::Sensor *frost_protection_minutes)
  {
    this->frost_protection_minutes = frost_protection_minutes;
  };
  void set_frost_protection_level(text_sensor::TextSensor *frost_protection_level)
  {
    this->frost_protection_level = frost_protection_level;
  };
  void set_filter_hours(sensor::Sensor *filter_hours) { this->filter_hours = filter_hours; };
  void set_summer_mode(binary_sensor::BinarySensor *summer_mode) { this->summer_mode = summer_mode; };
  void set_p10_active(binary_sensor::BinarySensor *p10_active) { this->p10_active = p10_active; };
  void set_p11_active(binary_sensor::BinarySensor *p11_active) { this->p11_active = p11_active; };
  void set_p12_active(binary_sensor::BinarySensor *p12_active) { this->p12_active = p12_active; };
  void set_p13_active(binary_sensor::BinarySensor *p13_active) { this->p13_active = p13_active; };
  void set_p14_active(binary_sensor::BinarySensor *p14_active) { this->p14_active = p14_active; };
  void set_p15_active(binary_sensor::BinarySensor *p15_active) { this->p15_active = p15_active; };
  void set_p16_active(binary_sensor::BinarySensor *p16_active) { this->p16_active = p16_active; };
  void set_p17_active(binary_sensor::BinarySensor *p17_active) { this->p17_active = p17_active; };
  void set_p18_active(binary_sensor::BinarySensor *p18_active) { this->p18_active = p18_active; };
  void set_p19_active(binary_sensor::BinarySensor *p19_active) { this->p19_active = p19_active; };
  void set_p90_active(binary_sensor::BinarySensor *p90_active) { this->p90_active = p90_active; };
  void set_p91_active(binary_sensor::BinarySensor *p91_active) { this->p91_active = p91_active; };
  void set_p92_active(binary_sensor::BinarySensor *p92_active) { this->p92_active = p92_active; };
  void set_p93_active(binary_sensor::BinarySensor *p93_active) { this->p93_active = p93_active; };
  void set_p94_active(binary_sensor::BinarySensor *p94_active) { this->p94_active = p94_active; };
  void set_p95_active(binary_sensor::BinarySensor *p95_active) { this->p95_active = p95_active; };
  void set_p96_active(binary_sensor::BinarySensor *p96_active) { this->p96_active = p96_active; };
  void set_p97_active(binary_sensor::BinarySensor *p97_active) { this->p97_active = p97_active; };
  void set_bathroom_switch_on_delay_minutes(sensor::Sensor *bathroom_switch_on_delay_minutes)
  {
    this->bathroom_switch_on_delay_minutes = bathroom_switch_on_delay_minutes;
  };
  void set_bathroom_switch_off_delay_minutes(sensor::Sensor *bathroom_switch_off_delay_minutes)
  {
    this->bathroom_switch_off_delay_minutes = bathroom_switch_off_delay_minutes;
  };
  void set_l1_switch_off_delay_minutes(sensor::Sensor *l1_switch_off_delay_minutes)
  {
    this->l1_switch_off_delay_minutes = l1_switch_off_delay_minutes;
  };
  void set_boost_ventilation_minutes(sensor::Sensor *boost_ventilation_minutes)
  {
    this->boost_ventilation_minutes = boost_ventilation_minutes;
  };
  void set_filter_warning_weeks(sensor::Sensor *filter_warning_weeks)
  {
    this->filter_warning_weeks = filter_warning_weeks;
  };
  void set_rf_high_time_short_minutes(sensor::Sensor *rf_high_time_short_minutes)
  {
    this->rf_high_time_short_minutes = rf_high_time_short_minutes;
  };
  void set_rf_high_time_long_minutes(sensor::Sensor *rf_high_time_long_minutes)
  {
    this->rf_high_time_long_minutes = rf_high_time_long_minutes;
  };
  void set_extractor_hood_switch_off_delay_minutes(sensor::Sensor *extractor_hood_switch_off_delay_minutes)
  {
    this->extractor_hood_switch_off_delay_minutes = extractor_hood_switch_off_delay_minutes;
  };
  void set_return_air_level_absent(ComfoAirNumber *number)
  {
    this->return_air_level_absent = number;
    if (number != nullptr)
      number->set_parent(this);
  }
  void set_return_air_level_low(ComfoAirNumber *number)
  {
    this->return_air_level_low = number;
    if (number != nullptr)
      number->set_parent(this);
  }
  void set_return_air_level_medium(ComfoAirNumber *number)
  {
    this->return_air_level_medium = number;
    if (number != nullptr)
      number->set_parent(this);
  }
  void set_return_air_level_high(ComfoAirNumber *number)
  {
    this->return_air_level_high = number;
    if (number != nullptr)
      number->set_parent(this);
  }
  void set_supply_air_level_absent(ComfoAirNumber *number)
  {
    this->supply_air_level_absent = number;
    if (number != nullptr)
      number->set_parent(this);
  }
  void set_supply_air_level_low(ComfoAirNumber *number)
  {
    this->supply_air_level_low = number;
    if (number != nullptr)
      number->set_parent(this);
  }
  void set_supply_air_level_medium(ComfoAirNumber *number)
  {
    this->supply_air_level_medium = number;
    if (number != nullptr)
      number->set_parent(this);
  }
  void set_supply_air_level_high(ComfoAirNumber *number)
  {
    this->supply_air_level_high = number;
    if (number != nullptr)
      number->set_parent(this);
  }
  void set_sync_fan_levels(ComfoAirSyncButton *button)
  {
    this->sync_button_ = button;
    if (button != nullptr)
      button->set_parent(this);
  }
};

inline const char *ComfoAirComponent::unit_size_text_label_(uint8_t raw_size) const
{
  switch (raw_size)
  {
  case 1:
    return "Large";
  case 2:
    return "Small";
  default:
    return "Unknown";
  }
}

inline const char *ComfoAirComponent::unit_size_option_label_(uint8_t raw_size) const
{
  switch (raw_size)
  {
  case 1:
    return "Large";
  case 2:
    return "Small";
  default:
    return nullptr;
  }
}

inline void ComfoAirComponent::publish_size_entities_(uint8_t raw_size)
{
  current_unit_size_ = raw_size;
  if (size != nullptr)
  {
    size->publish_state(unit_size_text_label_(raw_size));
  }
  if (size_select_ != nullptr)
  {
    if (const char *option = unit_size_option_label_(raw_size))
    {
      size_select_->publish_state(option);
    }
    else
    {
      ESP_LOGW(TAG, "Unsupported unit size value: %u", raw_size);
    }
  }
}

inline bool ComfoAirComponent::set_unit_size(uint8_t raw_size)
{
  if (raw_size != 1 && raw_size != 2)
  {
    ESP_LOGW(TAG, "Ignoring invalid unit size request: %u", raw_size);
    return false;
  }

  if (!status_payload_valid_)
  {
    ESP_LOGW(TAG, "Unit size cannot be changed before initial status is received");
    publish_size_entities_(current_unit_size_);
    return false;
  }

  if (status_payload_[3] == raw_size)
  {
    publish_size_entities_(raw_size);
    return true;
  }

  uint8_t payload[sizeof(status_payload_)];
  memcpy(payload, status_payload_, sizeof(status_payload_));
  payload[3] = raw_size;

  ESP_LOGI(TAG, "Setting unit size to %s", unit_size_text_label_(raw_size));
  write_command_(CMD_SET_STATUS, payload, sizeof(payload));

  status_payload_[3] = raw_size;
  publish_size_entities_(raw_size);

  return true;
}

inline void ComfoAirComponent::set_size_select(ComfoAirSizeSelect *size_select)
{
  this->size_select_ = size_select;
  if (this->size_select_ != nullptr)
  {
    this->size_select_->set_parent(this);
    if (const char *option = this->unit_size_option_label_(this->current_unit_size_))
    {
      this->size_select_->publish_state(option);
    }
  }
}

inline void ComfoAirSizeSelect::control(const std::string &value)
{
  if (this->parent_ == nullptr)
  {
    ESP_LOGW(TAG, "Unit size select has no parent component configured");
    return;
  }

  auto index = this->index_of(value);
  if (!index.has_value())
  {
    ESP_LOGW(TAG, "Unit size select received invalid option: %s", value.c_str());
    return;
  }

  uint8_t raw_size = 0;
  switch (index.value())
  {
  case 0:
    raw_size = 1;
    break;
  case 1:
    raw_size = 2;
    break;
  default:
    ESP_LOGW(TAG, "Unit size select index %zu not supported", index.value());
    return;
  }

  if (!this->parent_->set_unit_size(raw_size))
  {
    if (const char *current_option = this->parent_->unit_size_option_label_(this->parent_->current_unit_size_))
    {
      this->publish_state(current_option);
    }
  }
}

inline void ComfoAirNumber::control(float value)
{
  this->publish_state(value);
  if (this->parent_ != nullptr)
  {
    this->parent_->set_fan_percentages();
  }
}

inline void ComfoAirSyncButton::press_action()
{
  if (this->parent_ != nullptr)
  {
    this->parent_->set_fan_percentages();
  }
}

} // namespace comfoair
} // namespace esphome
