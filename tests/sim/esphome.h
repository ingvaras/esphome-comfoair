// Minimal stand-ins for the ESPHome APIs used by comfoair.h, just enough to
// compile it on a host and drive it from tests/proxy_sim.cpp.
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <set>
#include <string>
#include <vector>

// The ESP32-only code of comfoair.h runs against the driver stand-ins below.
#define USE_ESP32

// Build with -DSIM_PRINT_LOGS to see the component's log output.
template <typename... Args> inline void sim_log(const char *level, const char *tag, const char *format, Args... args)
{
#ifdef SIM_PRINT_LOGS
  std::printf("[%s][%s] ", level, tag);
  std::printf(format, args...);
  std::printf("\n");
#endif
}
#define ESP_LOGCONFIG(...) sim_log("C", __VA_ARGS__)
#define ESP_LOGE(...) sim_log("E", __VA_ARGS__)
#define ESP_LOGW(...) sim_log("W", __VA_ARGS__)
#define ESP_LOGI(...) sim_log("I", __VA_ARGS__)
#define ESP_LOGD(...) sim_log("D", __VA_ARGS__)
#define ESP_LOGV(...) sim_log("V", __VA_ARGS__)
#define ESP_LOGVV(...) sim_log("VV", __VA_ARGS__)

// driver/gpio.h and driver/uart.h, provided by the simulation.
typedef int gpio_num_t;
typedef int uart_port_t;
enum gpio_mode_t
{
  GPIO_MODE_INPUT
};
enum gpio_pull_mode_t
{
  GPIO_PULLUP_ONLY,
  GPIO_PULLDOWN_ONLY
};
#define UART_PIN_NO_CHANGE (-1)
void gpio_reset_pin(gpio_num_t pin);
void gpio_set_direction(gpio_num_t pin, gpio_mode_t mode);
void gpio_set_pull_mode(gpio_num_t pin, gpio_pull_mode_t pull);
int gpio_get_level(gpio_num_t pin);
void uart_set_pin(uart_port_t port, int tx, int rx, int rts, int cts);
void uart_flush_input(uart_port_t port);

namespace esphome
{
uint32_t millis(); // provided by the simulation
inline void delay(uint32_t) {}
template <size_t N>
inline char *format_hex_pretty_to(char (&buffer)[N], const uint8_t *data, size_t length, char separator = ':')
{
  size_t used = 0;
  buffer[0] = 0;
  for (size_t i = 0; i < length && used + 4 <= N; i++)
  {
    used += std::snprintf(buffer + used, N - used, "%s%02X", i == 0 ? "" : std::string(1, separator).c_str(), data[i]);
  }
  return buffer;
}

// Stored values outlive a ComfoAirComponent, as flash outlives a restart.
inline std::map<uint32_t, std::vector<uint8_t>> &sim_preferences()
{
  static std::map<uint32_t, std::vector<uint8_t>> store;
  return store;
}
class ESPPreferenceObject
{
public:
  explicit ESPPreferenceObject(uint32_t type = 0) : type_(type) {}
  template <typename T> bool load(T *dest)
  {
    auto it = sim_preferences().find(type_);
    if (it == sim_preferences().end() || it->second.size() != sizeof(T))
    {
      return false;
    }
    memcpy(dest, it->second.data(), sizeof(T));
    return true;
  }
  template <typename T> bool save(const T *src)
  {
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(src);
    sim_preferences()[type_] = std::vector<uint8_t>(bytes, bytes + sizeof(T));
    return true;
  }

private:
  uint32_t type_;
};
class ESPPreferences
{
public:
  template <typename T> ESPPreferenceObject make_preference(uint32_t type, bool) { return ESPPreferenceObject(type); }
};
inline ESPPreferences *global_preferences = new ESPPreferences();
inline uint32_t fnv1_hash(const char *) { return 1; }

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
  virtual void load_settings(bool) {}
  void write_byte(uint8_t b) { write_array(&b, 1); }
};

class IDFUARTComponent : public UARTComponent
{
public:
  uint8_t get_hw_serial_number() { return port; }
  uint8_t port{0};
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
