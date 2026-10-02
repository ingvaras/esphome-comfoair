// Minimal stand-ins for the ESPHome APIs used by comfoair.h, just enough to
// compile it on a host and drive it from tests/proxy_sim.cpp.
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <set>
#include <string>
#include <vector>

#define ESP_LOGCONFIG(...) ((void)0)
#define ESP_LOGE(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGD(...) ((void)0)
#define ESP_LOGV(...) ((void)0)
#define ESP_LOGVV(...) ((void)0)

namespace esphome
{
uint32_t millis(); // provided by the simulation
template <size_t N> inline char *format_hex_pretty_to(char (&buffer)[N], const uint8_t *, size_t, char = ':')
{
  buffer[0] = 0;
  return buffer;
}

class Component
{
public:
  virtual ~Component() = default;
  virtual void setup() {}
  virtual void loop() {}
  virtual void dump_config() {}
  virtual float get_setup_priority() const { return 0; }
  void status_clear_warning() {}
};

class PollingComponent : public Component
{
public:
  explicit PollingComponent(uint32_t) {}
  virtual void update() {}
};

class HighFrequencyLoopRequester
{
public:
  void start() { started = true; }
  bool started{false};
};

namespace setup_priority
{
static const float DATA = 0;
}

namespace uart
{
class UARTComponent
{
public:
  virtual ~UARTComponent() = default;
  virtual void write_array(const uint8_t *data, size_t len) = 0;
  virtual size_t available() = 0;
  virtual bool read_byte(uint8_t *data) = 0;
  virtual void flush() = 0;
  void write_byte(uint8_t b) { write_array(&b, 1); }
};

class UARTDevice
{
public:
  void set_uart_parent(UARTComponent *parent) { parent_ = parent; }
  void write_array(const uint8_t *d, size_t n) { parent_->write_array(d, n); }
  void write_byte(uint8_t b) { parent_->write_byte(b); }
  size_t available() { return parent_->available(); }
  bool read_byte(uint8_t *b) { return parent_->read_byte(b); }
  void flush() { parent_->flush(); }
  void check_uart_settings(uint32_t) {}

protected:
  UARTComponent *parent_{nullptr};
};
} // namespace uart

namespace sensor
{
struct Sensor
{
  float state{0};
  int publishes{0};
  void publish_state(float v)
  {
    state = v;
    publishes++;
  }
  void set_accuracy_decimals(int) {}
};
} // namespace sensor
namespace binary_sensor
{
struct BinarySensor
{
  bool state{false};
  int publishes{0};
  void publish_state(bool v)
  {
    state = v;
    publishes++;
  }
};
} // namespace binary_sensor
namespace text_sensor
{
struct TextSensor
{
  std::string state;
  int publishes{0};
  void publish_state(const std::string &v)
  {
    state = v;
    publishes++;
  }
};
} // namespace text_sensor
namespace select
{
struct Select
{
  virtual ~Select() = default;
  virtual void control(const std::string &) {}
  void publish_state(const std::string &) {}
  bool has_index() const { return false; }
  struct Idx
  {
    bool has_value() const { return true; }
    size_t operator*() const { return 0; }
    size_t value() const { return 0; }
  };
  Idx index_of(const std::string &) const { return Idx(); }
  const char *current_option() const { return ""; }
  void publish_state(size_t) {}
};
} // namespace select
namespace number
{
struct Number
{
  float state{0};
  virtual ~Number() = default;
  virtual void control(float) {}
  void publish_state(float v) { state = v; }
};
} // namespace number
namespace button
{
struct Button
{
  virtual ~Button() = default;
  virtual void press_action() {}
};
} // namespace button

namespace climate
{
enum ClimateMode
{
  CLIMATE_MODE_OFF,
  CLIMATE_MODE_FAN_ONLY
};
enum ClimateFanMode
{
  CLIMATE_FAN_ON,
  CLIMATE_FAN_OFF,
  CLIMATE_FAN_AUTO,
  CLIMATE_FAN_LOW,
  CLIMATE_FAN_MEDIUM,
  CLIMATE_FAN_HIGH,
  CLIMATE_FAN_MIDDLE,
  CLIMATE_FAN_DIFFUSE
};
enum ClimateFeature
{
  CLIMATE_SUPPORTS_CURRENT_TEMPERATURE,
  CLIMATE_REQUIRES_TWO_POINT_TARGET_TEMPERATURE,
  CLIMATE_SUPPORTS_ACTION
};
struct ClimateTraits
{
  void add_feature_flags(ClimateFeature) {}
  void clear_feature_flags(ClimateFeature) {}
  void set_supported_modes(std::set<ClimateMode>) {}
  void set_supported_fan_modes(std::set<ClimateFanMode>) {}
  void set_visual_min_temperature(float) {}
  void set_visual_max_temperature(float) {}
  void set_visual_temperature_step(float) {}
};
template <typename T> struct Opt
{
  bool has_value() const { return set; }
  T operator*() const { return v; }
  T value() const { return v; }
  Opt &operator=(T x)
  {
    v = x;
    set = true;
    return *this;
  }
  T v{};
  bool set{false};
};
struct ClimateCall
{
  Opt<ClimateFanMode> fan;
  Opt<float> temp;
  const Opt<ClimateFanMode> &get_fan_mode() const { return fan; }
  const Opt<float> &get_target_temperature() const { return temp; }
};
class Climate
{
public:
  virtual ~Climate() = default;
  virtual ClimateTraits traits() = 0;
  virtual void control(const ClimateCall &) = 0;
  void publish_state() {}
  Opt<ClimateFanMode> fan_mode;
  ClimateMode mode{CLIMATE_MODE_OFF};
  float current_temperature{0};
  float target_temperature{0};
};
} // namespace climate
} // namespace esphome
