#pragma once

// Bus arbitration helpers for the optional ComfoSense proxy mode.
//
// These classes are deliberately free of ESPHome dependencies so that the
// timing/queueing rules can be unit tested on a host (see tests/).

#include <stdint.h>
#include <string.h>

namespace esphome
{
namespace comfoair
{

// Largest protocol frame (including START/STOP and escaping) we queue.
// Must match MAX_MESSAGE_SIZE in comfoair.h (checked by a static_assert there).
static const uint8_t PROXY_MAX_FRAME_SIZE = 70U;

// Both directions must have been silent for this long before we inject a
// frame. At 9600 baud one byte takes ~1ms, so this is far longer than any
// gap inside a frame.
static const uint32_t PROXY_IDLE_GAP_MS = 30U;
// A panel request is assumed to be answered within this time. Until the unit's
// answer is seen (or the time expires) the bus is not considered free.
static const uint32_t PROXY_PANEL_EXCHANGE_TIMEOUT_MS = 300U;
// Maximum duration of one exchange we initiated.
static const uint32_t PROXY_OWN_EXCHANGE_TIMEOUT_MS = 300U;

class ProxyBusArbiter
{
public:
  void note_unit_byte(uint32_t now_ms)
  {
    last_unit_byte_ms_ = now_ms;
    unit_seen_ = true;
  }

  void note_panel_byte(uint32_t now_ms)
  {
    last_panel_byte_ms_ = now_ms;
    panel_seen_ = true;
  }

  // A complete, valid frame from the panel was seen: the unit is expected to answer.
  void panel_frame_complete(uint32_t now_ms)
  {
    panel_exchange_start_ms_ = now_ms;
    panel_exchange_pending_ = true;
  }

  // A complete frame from the unit was seen: the panel exchange is answered.
  void unit_frame_complete() { panel_exchange_pending_ = false; }

  bool can_inject(uint32_t now_ms) const
  {
    if (own_exchange_)
    {
      return false;
    }
    if (quiet_(now_ms, last_unit_byte_ms_, unit_seen_) == false || quiet_(now_ms, last_panel_byte_ms_, panel_seen_) == false)
    {
      return false;
    }
    if (panel_exchange_pending_ && (now_ms - panel_exchange_start_ms_) < PROXY_PANEL_EXCHANGE_TIMEOUT_MS)
    {
      return false;
    }
    return true;
  }

  void begin_own_exchange(uint32_t now_ms)
  {
    own_exchange_ = true;
    own_exchange_start_ms_ = now_ms;
    panel_exchange_pending_ = false;
  }

  void end_own_exchange() { own_exchange_ = false; }

  bool own_exchange() const { return own_exchange_; }

  // Whether any byte was ever received from that side.
  bool unit_seen() const { return unit_seen_; }
  bool panel_seen() const { return panel_seen_; }

  bool own_exchange_timed_out(uint32_t now_ms) const
  {
    return own_exchange_ && (now_ms - own_exchange_start_ms_) >= PROXY_OWN_EXCHANGE_TIMEOUT_MS;
  }

private:
  static bool quiet_(uint32_t now_ms, uint32_t last_ms, bool seen)
  {
    return !seen || (now_ms - last_ms) >= PROXY_IDLE_GAP_MS;
  }

  uint32_t last_unit_byte_ms_{0};
  uint32_t last_panel_byte_ms_{0};
  uint32_t panel_exchange_start_ms_{0};
  uint32_t own_exchange_start_ms_{0};
  bool unit_seen_{false};
  bool panel_seen_{false};
  bool panel_exchange_pending_{false};
  bool own_exchange_{false};
};

// Frames waiting for a free bus. Frames are stored fully encoded
// (07 F0 00 <cmd> ... 07 0F), so the command byte is at index 3.
class ProxyTxQueue
{
public:
  static const uint8_t DEPTH = 8U;

  struct Frame
  {
    uint8_t data[PROXY_MAX_FRAME_SIZE];
    uint8_t length{0};

    uint8_t command() const { return length > 3U ? data[3] : 0U; }
  };

  // Identical frames already queued are dropped. If replace_same_command is
  // set, a queued frame with the same command is replaced (latest value wins).
  // Returns false if the queue is full or the frame is invalid.
  bool push(const uint8_t *data, uint8_t length, bool replace_same_command)
  {
    if (length < 4U || length > PROXY_MAX_FRAME_SIZE)
    {
      return false;
    }
    for (uint8_t i = 0; i < count_; i++)
    {
      Frame &queued = frames_[(head_ + i) % DEPTH];
      if (queued.length == length && memcmp(queued.data, data, length) == 0)
      {
        return true;
      }
      if (replace_same_command && queued.command() == data[3])
      {
        memcpy(queued.data, data, length);
        queued.length = length;
        return true;
      }
    }
    if (count_ == DEPTH)
    {
      return false;
    }
    Frame &slot = frames_[(head_ + count_) % DEPTH];
    memcpy(slot.data, data, length);
    slot.length = length;
    count_++;
    return true;
  }

  bool empty() const { return count_ == 0; }

  bool pop(Frame *out)
  {
    if (count_ == 0)
    {
      return false;
    }
    *out = frames_[head_];
    head_ = (head_ + 1) % DEPTH;
    count_--;
    return true;
  }

private:
  Frame frames_[DEPTH];
  uint8_t head_{0};
  uint8_t count_{0};
};

// Panel bytes received while we own the bus. They are replayed to the unit,
// in order, once our exchange has finished.
class ProxyHoldBuffer
{
public:
  static const uint16_t CAPACITY = 160U;

  // Returns false (byte dropped) when full.
  bool push(uint8_t byte)
  {
    if (length_ >= CAPACITY)
    {
      return false;
    }
    data_[length_++] = byte;
    return true;
  }

  bool empty() const { return length_ == 0; }
  const uint8_t *data() const { return data_; }
  uint16_t length() const { return length_; }
  void clear() { length_ = 0; }

private:
  uint8_t data_[CAPACITY];
  uint16_t length_{0};
};

// Collects raw bytes of one direction so they can be logged as one hex line
// per burst. Used for the optional log_raw_bytes debugging aid.
class RawByteCollector
{
public:
  static const uint8_t CAPACITY = 32U;
  static const uint32_t BURST_GAP_MS = 5U;

  // Returns true if the buffer is full and must be flushed.
  bool add(uint8_t byte, uint32_t now_ms)
  {
    data_[length_++] = byte;
    last_byte_ms_ = now_ms;
    return length_ == CAPACITY;
  }

  bool should_flush(uint32_t now_ms) const { return length_ != 0 && (now_ms - last_byte_ms_) >= BURST_GAP_MS; }
  const uint8_t *data() const { return data_; }
  uint8_t length() const { return length_; }
  void clear() { length_ = 0; }

private:
  uint8_t data_[CAPACITY];
  uint8_t length_{0};
  uint32_t last_byte_ms_{0};
};

// What a pin looks like to the ESP. An adapter's output drives the pin and ignores
// the ESP's weak pull resistors; an adapter's input or a loose wire follows them.
enum class PinDrive : uint8_t
{
  UNKNOWN,
  DRIVEN_HIGH,
  DRIVEN_LOW,
  NOT_DRIVEN,
};

enum class AdapterWiring : uint8_t
{
  MATCHES_CONFIG,
  SWAPPED,
  NO_OUTPUT,
  BOTH_DRIVEN,
  HELD_LOW,
};

// Results and test plan of the wiring check. Pins are numbered in the order
// unit RX, unit TX, panel RX, panel TX, as configured.
class WiringCheck
{
public:
  static const uint8_t PIN_COUNT = 4U;
  static const uint8_t SAMPLES_PER_PULL = 10U;
  static const uint8_t HEARD_SAMPLE_SIZE = 24U;
  static const uint8_t MAX_TESTS = 12U;

  // Arguments: how many of SAMPLES_PER_PULL reads were high with each pull. Traffic
  // can make single reads differ, so the majority decides.
  static PinDrive classify_pin(uint8_t high_with_pullup, uint8_t high_with_pulldown)
  {
    if (high_with_pulldown >= SAMPLES_PER_PULL / 2U)
    {
      return PinDrive::DRIVEN_HIGH;
    }
    if (high_with_pullup <= SAMPLES_PER_PULL / 2U)
    {
      return PinDrive::DRIVEN_LOW;
    }
    return PinDrive::NOT_DRIVEN;
  }

  void set_drive(uint8_t pin, PinDrive drive) { drive_[pin] = drive; }
  PinDrive drive(uint8_t pin) const { return drive_[pin]; }

  AdapterWiring adapter_wiring(uint8_t rx_pin, uint8_t tx_pin) const
  {
    const PinDrive rx = drive_[rx_pin];
    const PinDrive tx = drive_[tx_pin];
    if (rx == PinDrive::DRIVEN_HIGH && tx == PinDrive::NOT_DRIVEN)
    {
      return AdapterWiring::MATCHES_CONFIG;
    }
    if (rx == PinDrive::NOT_DRIVEN && tx == PinDrive::DRIVEN_HIGH)
    {
      return AdapterWiring::SWAPPED;
    }
    if (rx == PinDrive::NOT_DRIVEN && tx == PinDrive::NOT_DRIVEN)
    {
      return AdapterWiring::NO_OUTPUT;
    }
    if (rx == PinDrive::DRIVEN_HIGH && tx == PinDrive::DRIVEN_HIGH)
    {
      return AdapterWiring::BOTH_DRIVEN;
    }
    return AdapterWiring::HELD_LOW;
  }

  void note_heard(uint8_t pin, uint8_t byte)
  {
    if (sample_length_[pin] < HEARD_SAMPLE_SIZE)
    {
      sample_[pin][sample_length_[pin]++] = byte;
    }
    heard_[pin]++;
  }
  uint16_t heard(uint8_t pin) const { return heard_[pin]; }
  const uint8_t *sample(uint8_t pin) const { return sample_[pin]; }
  uint8_t sample_length(uint8_t pin) const { return sample_length_[pin]; }

  // Listen on every adapter output and only transmit on pins the adapter is not
  // driving, so two outputs never fight. If both pins of an adapter look driven,
  // the levels can't separate them, so both are tried.
  void plan_tests()
  {
    test_count_ = 0;
    for (uint8_t tx = 0; tx < PIN_COUNT; tx++)
    {
      const bool both_driven = drive_[tx] == PinDrive::DRIVEN_HIGH && drive_[tx ^ 1U] == PinDrive::DRIVEN_HIGH;
      if (drive_[tx] != PinDrive::NOT_DRIVEN && !both_driven)
      {
        continue;
      }
      for (uint8_t rx = 0; rx < PIN_COUNT; rx++)
      {
        if (rx != tx && drive_[rx] == PinDrive::DRIVEN_HIGH && test_count_ < MAX_TESTS)
        {
          test_tx_[test_count_] = tx;
          test_rx_[test_count_] = rx;
          test_count_++;
        }
      }
    }
  }
  uint8_t test_count() const { return test_count_; }
  uint8_t test_tx(uint8_t test) const { return test_tx_[test]; }
  uint8_t test_rx(uint8_t test) const { return test_rx_[test]; }

  // `response_command` is what only the unit sends in answer to our poll.
  void note_poll_reply(uint8_t test, const uint8_t *data, uint8_t length, uint8_t response_command)
  {
    const uint8_t response[] = {0x07, 0xF0, 0x00, response_command};
    if (contains_(data, length, response, sizeof(response)))
    {
      unit_answered_[test] = true;
    }
    note_reply(test, data, length);
  }
  void note_reply(uint8_t test, const uint8_t *data, uint8_t length)
  {
    const uint8_t ack[] = {0x07, 0xF3};
    if (contains_(data, length, ack, sizeof(ack)))
    {
      acked_[test] = true;
    }
    if (length != 0)
    {
      got_bytes_[test] = true;
    }
  }
  bool unit_answered(uint8_t test) const { return unit_answered_[test]; }
  bool acked(uint8_t test) const { return acked_[test]; }
  bool got_bytes(uint8_t test) const { return got_bytes_[test]; }

private:
  static bool contains_(const uint8_t *data, uint8_t length, const uint8_t *pattern, uint8_t pattern_length)
  {
    for (uint8_t i = 0; i + pattern_length <= length; i++)
    {
      if (memcmp(data + i, pattern, pattern_length) == 0)
      {
        return true;
      }
    }
    return false;
  }

  PinDrive drive_[PIN_COUNT]{PinDrive::UNKNOWN, PinDrive::UNKNOWN, PinDrive::UNKNOWN, PinDrive::UNKNOWN};
  uint16_t heard_[PIN_COUNT]{};
  uint8_t sample_[PIN_COUNT][HEARD_SAMPLE_SIZE]{};
  uint8_t sample_length_[PIN_COUNT]{};
  uint8_t test_tx_[MAX_TESTS]{};
  uint8_t test_rx_[MAX_TESTS]{};
  uint8_t test_count_{0};
  bool unit_answered_[MAX_TESTS]{};
  bool acked_[MAX_TESTS]{};
  bool got_bytes_[MAX_TESTS]{};
};

} // namespace comfoair
} // namespace esphome
