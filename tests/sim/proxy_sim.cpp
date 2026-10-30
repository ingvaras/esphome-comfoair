// Host simulation of the ComfoSense proxy mode: compiles the real comfoair.h
// against stubbed ESPHome headers and drives it with fake UARTs.
// Build/run: see tests/README.md
#include "comfoair.h"

#include <cstdio>
#include <cstdlib>
#include <deque>

static uint32_t g_now = 1000;
uint32_t esphome::millis() { return g_now; }

using namespace esphome;
using namespace esphome::comfoair;

#define CHECK(cond)                                                                                                    \
  do                                                                                                                   \
  {                                                                                                                    \
    if (!(cond))                                                                                                       \
    {                                                                                                                  \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                                      \
      std::exit(1);                                                                                                    \
    }                                                                                                                  \
  } while (0)

typedef std::vector<uint8_t> Bytes;

struct FakeUart;
static void wiring_on_write(FakeUart *uart, const uint8_t *d, size_t n);
static void wiring_on_load_settings(FakeUart *uart);

struct FakeUart : uart::IDFUARTComponent
{
  std::deque<uint8_t> rx; // bytes the component will read
  Bytes tx;               // bytes the component wrote
  bool wired{false};      // part of the wiring model below
  int load_settings_calls{0};
  void write_array(const uint8_t *d, size_t n) override
  {
    tx.insert(tx.end(), d, d + n);
    if (wired)
    {
      wiring_on_write(this, d, n);
    }
  }
  void load_settings(bool) override
  {
    load_settings_calls++;
    wiring_on_load_settings(this);
  }
  size_t available() override { return rx.size(); }
  bool read_byte(uint8_t *b) override
  {
    if (rx.empty())
    {
      return false;
    }
    *b = rx.front();
    rx.pop_front();
    return true;
  }
  void flush() override {}
  void feed(const Bytes &b) { rx.insert(rx.end(), b.begin(), b.end()); }
};

// Encode a protocol frame like the unit/panel do: 07 F0 00 cmd len data.. chk 07 0F
static Bytes frame(uint8_t cmd, const Bytes &data)
{
  Bytes f{0x07, 0xF0, 0x00, cmd, (uint8_t)data.size()};
  uint16_t sum = 0xAD + cmd + data.size();
  for (uint8_t d : data)
  {
    f.push_back(d);
    if (d == 0x07)
    {
      f.push_back(0x07);
    }
    sum += d;
  }
  f.push_back((uint8_t)sum);
  if ((uint8_t)sum == 0x07)
  {
    f.push_back(0x07);
  }
  f.push_back(0x07);
  f.push_back(0x0F);
  return f;
}
static const Bytes ACK{0x07, 0xF3};

struct Rig
{
  FakeUart unit, panel;
  ComfoAirComponent c;
  sensor::Sensor intake;
  explicit Rig(bool proxy, bool write_only = false)
  {
    c.set_uart_component(&unit);
    if (proxy)
    {
      c.set_proxy_uart(&panel);
    }
    c.set_unit_write_only(write_only);
    c.intake_fan_speed = &intake;
    c.setup();
  }
  void run(uint32_t ms)
  {
    for (uint32_t i = 0; i < ms; i++)
    {
      g_now++;
      c.loop();
    }
  }
  // Queue a Home Assistant command (set level) through the public API.
  void ha_set_level(int level) { c.set_level(level); }
};

static Bytes concat(Bytes a, const Bytes &b)
{
  a.insert(a.end(), b.begin(), b.end());
  return a;
}

// Panel polls fan status; unit answers. Everything is relayed, sensors update,
// and the component does not ACK on its own.
static void test_transparent_relay()
{
  Rig r(true);
  Bytes req = frame(CMD_GET_FAN_STATUS, {});
  Bytes ack_resp = frame(RES_GET_FAN_STATUS, {50, 60, 0x0F, 0xA0, 0x0F, 0xA0});

  r.panel.feed(req);
  r.run(10);
  CHECK(r.unit.tx == req);

  r.unit.feed(ACK);
  r.unit.feed(ack_resp);
  r.run(20);
  CHECK(r.panel.tx == concat(ACK, ack_resp)); // byte-identical relay
  CHECK(r.intake.state == 50);                // parsed from relayed traffic
  CHECK(r.unit.tx == req);                    // no ACK of our own towards the unit

  r.panel.feed(ACK); // panel acknowledges the response
  r.run(10);
  CHECK(r.unit.tx == concat(req, ACK));
}

// An HA command waits for the bus, is injected on an idle bus, its reply is not
// leaked to the panel, and panel traffic during the exchange is held and replayed.
static void test_injection_and_hold()
{
  Rig r(true);
  Bytes panel_req = frame(CMD_GET_FAN_STATUS, {});

  // Panel exchange in flight: HA command must not be injected yet.
  r.panel.feed(panel_req);
  r.run(5);
  r.ha_set_level(2);
  r.run(100); // unit never answers within this window (< 300ms timeout)
  CHECK(r.unit.tx == panel_req);

  // Unit answers; panel ACKs; bus goes quiet -> injection after the idle gap.
  r.unit.feed(ACK);
  r.unit.feed(frame(RES_GET_FAN_STATUS, {50, 60, 0x0F, 0xA0, 0x0F, 0xA0}));
  r.run(5);
  r.panel.feed(ACK);
  r.run(5);
  size_t before = r.unit.tx.size();
  r.run(PROXY_IDLE_GAP_MS + 5);
  Bytes injected = frame(CMD_SET_LEVEL, {2});
  CHECK(r.unit.tx.size() == before + injected.size());
  CHECK(Bytes(r.unit.tx.end() - injected.size(), r.unit.tx.end()) == injected);

  // While our exchange is open the panel starts a request: it must be held.
  Bytes panel_req2 = frame(CMD_GET_TEMPERATURES, {});
  size_t panel_tx_before = r.panel.tx.size();
  r.panel.feed(panel_req2);
  r.run(5);
  CHECK(r.unit.tx.size() == before + injected.size()); // held, not forwarded
  r.unit.feed(ACK);                                    // unit acks our command
  r.run(5);
  CHECK(r.panel.tx.size() == panel_tx_before);                       // ACK was not leaked to the panel
  CHECK(Bytes(r.unit.tx.end() - panel_req2.size(), r.unit.tx.end()) == panel_req2); // replayed in order
}

// A command whose answer never arrives is retried once, then dropped, and the
// panel is never blocked longer than the timeout.
static void test_timeout_retry()
{
  Rig r(true);
  r.ha_set_level(3);
  r.run(PROXY_IDLE_GAP_MS + 2);
  Bytes injected = frame(CMD_SET_LEVEL, {3});
  CHECK(r.unit.tx == injected);
  r.run(PROXY_OWN_EXCHANGE_TIMEOUT_MS + 5); // timeout -> retry (+idle gap)
  r.run(PROXY_IDLE_GAP_MS + 5);
  CHECK(r.unit.tx == concat(injected, injected));
  r.run(PROXY_OWN_EXCHANGE_TIMEOUT_MS + PROXY_IDLE_GAP_MS + 20);
  CHECK(r.unit.tx == concat(injected, injected)); // gave up
  CHECK(r.panel.tx.empty());
}

// Without a proxy UART the component behaves as before.
static void test_legacy_mode()
{
  Rig r(false);
  r.ha_set_level(2);
  CHECK(r.unit.tx == frame(CMD_SET_LEVEL, {2})); // written immediately, no queueing
  r.unit.tx.clear();
  r.unit.feed(frame(RES_GET_FAN_STATUS, {50, 60, 0x0F, 0xA0, 0x0F, 0xA0}));
  r.run(5);
  CHECK(r.intake.state == 50);
  CHECK(r.unit.tx == ACK); // ACKs every message itself
}

// The fake-reply test aid answers the panel's temperature poll, and only that.
static void test_fake_unit_reply()
{
  Rig r(true);
  r.c.set_test_fake_unit_reply(true);
  r.panel.feed(frame(CMD_GET_FAN_STATUS, {}));
  r.run(10);
  CHECK(r.panel.tx.empty()); // other requests are not answered

  r.panel.feed(frame(CMD_GET_TEMPERATURES, {}));
  r.run(10);
  Bytes expected = concat(ACK, frame(RES_GET_TEMPERATURES, {84, 63, 75, 83, 57, 0x0F, 0, 0, 0}));
  CHECK(r.panel.tx == expected);

  Rig off(true); // disabled by default
  off.panel.feed(frame(CMD_GET_TEMPERATURES, {}));
  off.run(10);
  CHECK(off.panel.tx.empty());
}

// test_panel_frame periodically sends the panel a clock frame; the panel's ACK is relayed as usual.
static void test_panel_frame()
{
  Rig r(true);
  r.c.set_test_panel_frame(true);
  r.run(100);
  CHECK(r.panel.tx.empty()); // not before the first interval

  r.run(2000);
  Bytes expected = frame(CMD_SET_PARAMETER, {0x02, 12, 34, 30, 100});
  CHECK(r.panel.tx == expected);

  r.panel.feed(ACK); // the panel's answer goes on to the unit unchanged
  r.run(10);
  CHECK(r.unit.tx == ACK);

  r.run(2000);
  CHECK(r.panel.tx == concat(expected, expected)); // repeats every interval

  Rig off(true); // disabled by default
  off.run(5000);
  CHECK(off.panel.tx.empty());
}

// test_ack_panel_frames acknowledges each valid panel frame, but not the panel's own ACKs.
static void test_ack_panel_frames()
{
  Rig r(true);
  r.c.set_test_ack_panel_frames(true);
  Bytes req = frame(CMD_GET_FAN_STATUS, {});
  r.panel.feed(req);
  r.run(10);
  CHECK(r.panel.tx == ACK);   // acknowledged once
  CHECK(r.unit.tx == req);    // and still relayed unchanged

  r.panel.feed(ACK);          // the panel's own ACK is not acknowledged
  r.run(10);
  CHECK(r.panel.tx == ACK);

  Rig off(true); // disabled by default
  off.panel.feed(req);
  off.run(10);
  CHECK(off.panel.tx.empty());
}

// --- Wiring model for the wiring check: adapter #1 on GPIO16/17, adapter #2 on GPIO18/19. ---

struct WiringModel
{
  // Levels of adapter outputs; every other pin follows the ESP's pull resistor.
  bool driven_high[40]{};
  bool driven_low[40]{};
  // The ESP pin that reaches each device's receive and transmit line (-1: not connected).
  int unit_in{-1}, unit_out{-1}, panel_in{-1}, panel_out{-1};
  bool pull_up[40]{};
  int tx[3]{-1, -1, -1}; // per UART port
  int rx[3]{-1, -1, -1};
  FakeUart *uarts[3]{nullptr, nullptr, nullptr};
  int level_reads{0};
  int writes_on_driven_pin{0};
};
static WiringModel g_wiring;

void gpio_reset_pin(gpio_num_t pin)
{
  g_wiring.pull_up[pin] = true;
  for (int port = 1; port <= 2; port++)
  {
    if (g_wiring.tx[port] == pin)
    {
      g_wiring.tx[port] = -1;
    }
  }
}
void gpio_set_direction(gpio_num_t, gpio_mode_t) {}
void gpio_set_pull_mode(gpio_num_t pin, gpio_pull_mode_t pull) { g_wiring.pull_up[pin] = pull == GPIO_PULLUP_ONLY; }
int gpio_get_level(gpio_num_t pin)
{
  g_wiring.level_reads++;
  return g_wiring.driven_high[pin] || (!g_wiring.driven_low[pin] && g_wiring.pull_up[pin]) ? 1 : 0;
}
void uart_set_pin(uart_port_t port, int tx, int rx, int, int)
{
  if (tx != UART_PIN_NO_CHANGE)
  {
    g_wiring.tx[port] = tx;
  }
  if (rx != UART_PIN_NO_CHANGE)
  {
    g_wiring.rx[port] = rx;
  }
}
void uart_flush_input(uart_port_t port) { g_wiring.uarts[port]->rx.clear(); }

// Bytes a device sends on `pin` reach every UART listening on that pin.
static void wiring_deliver(int pin, const Bytes &bytes)
{
  for (int port = 1; port <= 2; port++)
  {
    if (pin >= 0 && g_wiring.rx[port] == pin)
    {
      g_wiring.uarts[port]->feed(bytes);
    }
  }
}

// The devices answer frames (not ACKs): the unit with an ACK plus its data, the panel with an ACK.
static void wiring_on_write(FakeUart *uart, const uint8_t *d, size_t n)
{
  const int pin = g_wiring.tx[uart->port];
  if (pin < 0)
  {
    return;
  }
  if (g_wiring.driven_high[pin] || g_wiring.driven_low[pin])
  {
    g_wiring.writes_on_driven_pin++;
  }
  if (n < 4)
  {
    return;
  }
  if (pin == g_wiring.unit_in)
  {
    wiring_deliver(g_wiring.unit_out, d[3] == CMD_GET_FAN_STATUS ? concat(ACK, frame(RES_GET_FAN_STATUS, {50, 50, 0, 0, 0, 0})) : ACK);
  }
  if (pin == g_wiring.panel_in)
  {
    wiring_deliver(g_wiring.panel_out, ACK);
  }
}

static void wiring_on_load_settings(FakeUart *uart)
{
  g_wiring.tx[uart->port] = uart->port == 1 ? 17 : 19;
  g_wiring.rx[uart->port] = uart->port == 1 ? 16 : 18;
}

struct WiredRig : Rig
{
  // Pass the pins as they really are; the UARTs are always configured as 16/17 and 18/19.
  WiredRig(int unit_output_pin, int unit_input_pin, int panel_output_pin, int panel_input_pin) : Rig(true)
  {
    g_wiring = WiringModel();
    g_wiring.uarts[1] = &unit;
    g_wiring.uarts[2] = &panel;
    unit.port = 1;
    panel.port = 2;
    unit.wired = true;
    panel.wired = true;
    wiring_on_load_settings(&unit);
    wiring_on_load_settings(&panel);
    if (unit_output_pin >= 0)
    {
      g_wiring.driven_high[unit_output_pin] = true;
      g_wiring.unit_out = unit_output_pin;
      g_wiring.unit_in = unit_input_pin;
    }
    if (panel_output_pin >= 0)
    {
      g_wiring.driven_high[panel_output_pin] = true;
      g_wiring.panel_out = panel_output_pin;
      g_wiring.panel_in = panel_input_pin;
    }
    c.set_wiring_check_pins(16, 17, 18, 19);
  }

  // Polls every 2s as PollingComponent does; the panel polls by itself every second.
  void run_wired(uint32_t ms)
  {
    for (uint32_t i = 0; i < ms; i++)
    {
      g_now++;
      if (g_now % 2000 == 0)
      {
        c.update();
      }
      if (g_now % 1000 == 500)
      {
        wiring_deliver(g_wiring.panel_out, frame(CMD_GET_TEMPERATURES, {}));
      }
      c.loop();
    }
  }

  // The result of the test using these pins (numbered as in WiringCheck).
  int find_test(uint8_t tx, uint8_t rx) const
  {
    const WiringCheck &w = c.wiring_check();
    for (uint8_t test = 0; test < w.test_count(); test++)
    {
      if (w.test_tx(test) == tx && w.test_rx(test) == rx)
      {
        return test;
      }
    }
    return -1;
  }
};

static const uint32_t WIRING_CHECK_FINISHED_MS = 120000;

// Healthy links: the check never touches the pins.
static void test_wiring_check_not_needed()
{
  WiredRig r(16, 17, 18, 19);
  r.run_wired(WIRING_CHECK_FINISHED_MS);
  CHECK(!r.c.wiring_check_done());
  CHECK(!r.c.wiring_check_running());
  CHECK(g_wiring.level_reads == 0);
  CHECK(r.unit.load_settings_calls == 0);
}

// Without set_wiring_check_pins() (wiring_check: false) nothing runs, however silent the bus is.
static void test_wiring_check_disabled()
{
  Rig r(true);
  g_wiring = WiringModel();
  r.run(WIRING_CHECK_FINISHED_MS);
  CHECK(!r.c.wiring_check_done());
  CHECK(g_wiring.level_reads == 0);
}

// Unit adapter's TXD/RXD crossed at the ESP: found, answered on the swapped pins, then
// normal operation resumes on the configured pins.
static void test_wiring_check_unit_swapped()
{
  WiredRig r(17, 16, 18, 19);
  r.run_wired(WIRING_CHECK_DELAY_MS - 1000);
  CHECK(!r.c.wiring_check_running()); // not before the delay
  r.run_wired(2000);
  CHECK(r.c.wiring_check_running());
  const int writes_on_driven_pin_before = g_wiring.writes_on_driven_pin; // normal polls did, with swapped wires
  size_t unit_tx_while_running = r.unit.tx.size();
  r.run_wired(29000); // listening only: passive, and the polls are paused
  CHECK(r.c.wiring_check_running());
  CHECK(r.unit.tx.size() == unit_tx_while_running);
  while (r.c.wiring_check_running())
  {
    r.run_wired(1);
  }
  CHECK(r.c.wiring_check_done());

  const WiringCheck &w = r.c.wiring_check();
  CHECK(w.adapter_wiring(0, 1) == AdapterWiring::SWAPPED);
  CHECK(w.adapter_wiring(2, 3) == AdapterWiring::MATCHES_CONFIG);
  CHECK(w.test_count() == 4);
  CHECK(g_wiring.writes_on_driven_pin == writes_on_driven_pin_before); // the check never fought an adapter output...
  const int unit_test = r.find_test(0, 1); // ...and sent on GPIO16, listened on GPIO17
  CHECK(unit_test >= 0 && w.unit_answered(unit_test));
  const int panel_test = r.find_test(3, 2);
  CHECK(panel_test >= 0 && w.acked(panel_test) && !w.unit_answered(panel_test));
  const int crossed_test = r.find_test(0, 2);
  CHECK(crossed_test >= 0 && !w.unit_answered(crossed_test));
  CHECK(w.heard(2) > 0 && w.sample(2)[0] == 0x07 && w.sample(2)[1] == 0xF0); // the panel's polls
  CHECK(w.heard(0) == 0 && w.heard(3) == 0);

  // UARTs restored, polls resume, and the check does not run a second time.
  CHECK(r.unit.load_settings_calls == 1 && r.panel.load_settings_calls == 1);
  CHECK(g_wiring.tx[1] == 17 && g_wiring.rx[1] == 16 && g_wiring.tx[2] == 19 && g_wiring.rx[2] == 18);
  const size_t unit_tx_after = r.unit.tx.size();
  const int level_reads_after = g_wiring.level_reads;
  r.run_wired(WIRING_CHECK_FINISHED_MS);
  CHECK(r.unit.tx.size() > unit_tx_after);
  CHECK(g_wiring.level_reads == level_reads_after);
}

// Panel adapter unpowered: reported as such, the unit link is confirmed.
static void test_wiring_check_panel_no_output()
{
  WiredRig r(16, 17, -1, -1);
  r.run_wired(WIRING_CHECK_FINISHED_MS);
  CHECK(r.c.wiring_check_done());
  const WiringCheck &w = r.c.wiring_check();
  CHECK(w.adapter_wiring(0, 1) == AdapterWiring::MATCHES_CONFIG);
  CHECK(w.adapter_wiring(2, 3) == AdapterWiring::NO_OUTPUT);
  const int unit_test = r.find_test(1, 0);
  CHECK(unit_test >= 0 && w.unit_answered(unit_test));
}

// ESP side correct on both adapters but nothing beyond them (e.g. crossed cable-side wires).
static void test_wiring_check_nothing_answers()
{
  WiredRig r(16, 17, 18, 19);
  g_wiring.unit_in = g_wiring.unit_out = g_wiring.panel_in = g_wiring.panel_out = -1;
  r.run_wired(WIRING_CHECK_FINISHED_MS);
  CHECK(r.c.wiring_check_done());
  const WiringCheck &w = r.c.wiring_check();
  CHECK(w.adapter_wiring(0, 1) == AdapterWiring::MATCHES_CONFIG);
  CHECK(w.adapter_wiring(2, 3) == AdapterWiring::MATCHES_CONFIG);
  CHECK(w.test_count() == 4);
  for (uint8_t test = 0; test < w.test_count(); test++)
  {
    CHECK(!w.got_bytes(test) && !w.acked(test) && !w.unit_answered(test));
  }
}

// --- unit_write_only: the unit hears commands but never answers ---

static Bytes ask_panel(Rig &r, const Bytes &request)
{
  r.panel.tx.clear();
  r.panel.feed(request);
  r.run(2);
  return r.panel.tx;
}

// Real panel polls carry one data byte: 07 F0 00 CD 01 00 7B 07 0F
static Bytes level_reply(uint8_t level, uint8_t percent)
{
  return concat(ACK, frame(RES_GET_VENTILATION_LEVEL, {15, 35, 50, 15, 35, 50, percent, percent, level, 1, 70, 70, 0, 0}));
}

static void test_write_only_answers_panel()
{
  sim_preferences().clear();
  Rig r(true, true);
  CHECK(ask_panel(r, frame(CMD_GET_VENTILATION_LEVEL, {0})) == level_reply(2, 35));
  CHECK(ask_panel(r, frame(CMD_GET_TEMPERATURES, {0})) == concat(ACK, frame(RES_GET_TEMPERATURES, {80, 80, 80, 80, 80, 0, 0, 0, 0})));
  CHECK(ask_panel(r, frame(CMD_GET_STATUS, {0})) == concat(ACK, frame(RES_GET_STATUS, {0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0})));
  CHECK(ask_panel(r, frame(CMD_GET_FAULTS, {0})) == concat(ACK, frame(RES_GET_FAULTS, Bytes(17, 0))));
  CHECK(ask_panel(r, frame(CMD_GET_RF_STATUS, {0})) == concat(ACK, frame(RES_GET_RF_STATUS, Bytes(7, 0))));

  // No known reply format: acknowledged only.
  CHECK(ask_panel(r, frame(CMD_GET_DATA, {0})) == ACK);
  CHECK(ask_panel(r, frame(CMD_CCEASE_BUTTON_STATUS, {0, 0, 0, 0, 0, 0, 2})) == ACK);
  // The panel's own ACK is not acknowledged.
  CHECK(ask_panel(r, ACK).empty());

  // Everything still reaches the unit, unchanged.
  r.unit.tx.clear();
  r.panel.feed(frame(CMD_GET_TEMPERATURES, {0}));
  r.run(2);
  CHECK(r.unit.tx == frame(CMD_GET_TEMPERATURES, {0}));
}

static void test_write_only_panel_sets_level()
{
  sim_preferences().clear();
  Rig r(true, true);
  r.panel.tx.clear();
  CHECK(ask_panel(r, frame(CMD_SET_LEVEL, {3})) == ACK);
  CHECK(r.unit.tx == frame(CMD_SET_LEVEL, {3}));
  CHECK(r.c.fan_mode.value() == climate::CLIMATE_FAN_MEDIUM);
  CHECK(ask_panel(r, frame(CMD_GET_VENTILATION_LEVEL, {0})) == level_reply(3, 50));

  // An invalid level is passed on like any frame but not remembered.
  CHECK(ask_panel(r, frame(CMD_SET_LEVEL, {9})) == ACK);
  CHECK(ask_panel(r, frame(CMD_GET_VENTILATION_LEVEL, {0})) == level_reply(3, 50));

  CHECK(ask_panel(r, frame(CMD_SET_COMFORT_TEMPERATURE, {90})) == ACK);
  CHECK(ask_panel(r, frame(CMD_GET_TEMPERATURES, {0})) == concat(ACK, frame(RES_GET_TEMPERATURES, {90, 90, 90, 90, 90, 0, 0, 0, 0})));
}

// Home Assistant updates the unit directly and the panel at its next poll.
static void test_write_only_home_assistant_sets_level()
{
  sim_preferences().clear();
  Rig r(true, true);
  r.ha_set_level(4);
  r.run(PROXY_IDLE_GAP_MS + 2);
  const Bytes command = frame(CMD_SET_LEVEL, {4});
  CHECK(r.unit.tx == command);
  CHECK(r.panel.tx.empty());
  r.run(2 * PROXY_OWN_EXCHANGE_TIMEOUT_MS + 50);
  CHECK(r.unit.tx == concat(command, command)); // sent twice, since it can't be confirmed
  r.run(PROXY_OWN_EXCHANGE_TIMEOUT_MS + 50);
  CHECK(r.unit.tx == concat(command, command));
  CHECK(ask_panel(r, frame(CMD_GET_VENTILATION_LEVEL, {0})) == level_reply(4, 70));

  // The panel can answer while such a command is on its way to the unit.
  r.unit.tx.clear();
  r.ha_set_level(2);
  r.run(PROXY_IDLE_GAP_MS + 2);
  CHECK(r.unit.tx == frame(CMD_SET_LEVEL, {2}));
  CHECK(ask_panel(r, frame(CMD_GET_VENTILATION_LEVEL, {0})) == level_reply(2, 35));
}

// No polling, and an unknown level is never sent to the unit.
static void test_write_only_does_not_poll()
{
  sim_preferences().clear();
  Rig r(true, true);
  for (int i = 0; i < 25; i++)
  {
    r.c.update();
    r.run(2000);
  }
  CHECK(r.unit.tx.empty());
  CHECK(r.c.fan_mode.value() == climate::CLIMATE_FAN_LOW);
}

static void test_write_only_resends_level()
{
  sim_preferences().clear();
  Rig r(true, true);
  r.ha_set_level(3);
  r.c.update();
  r.run(1000);
  r.unit.tx.clear();
  r.c.update();
  r.run(1000);
  CHECK(r.unit.tx.empty()); // not due yet
  r.run(WRITE_ONLY_LEVEL_RESEND_MS);
  r.c.update();
  r.run(PROXY_IDLE_GAP_MS + 2);
  CHECK(r.unit.tx == frame(CMD_SET_LEVEL, {3}));
}

static void test_write_only_remembers_across_restart()
{
  sim_preferences().clear();
  {
    Rig r(true, true);
    r.panel.feed(frame(CMD_SET_LEVEL, {3}));
    r.panel.feed(frame(CMD_SET_COMFORT_TEMPERATURE, {90}));
    r.run(2);
  }
  Rig r(true, true);
  CHECK(ask_panel(r, frame(CMD_GET_VENTILATION_LEVEL, {0})) == level_reply(3, 50));
  CHECK(ask_panel(r, frame(CMD_GET_TEMPERATURES, {0})) == concat(ACK, frame(RES_GET_TEMPERATURES, {90, 90, 90, 90, 90, 0, 0, 0, 0})));
  r.run(PROXY_IDLE_GAP_MS + 2);
  r.unit.tx.clear();
  r.c.update();
  r.run(PROXY_IDLE_GAP_MS + 2);
  CHECK(r.unit.tx == frame(CMD_SET_LEVEL, {3})); // puts a restarted unit back on the level
  CHECK(r.c.fan_mode.value() == climate::CLIMATE_FAN_MEDIUM);
}

// Anything from the unit is dropped so that it can't collide with our replies.
static void test_write_only_ignores_unit_bytes()
{
  sim_preferences().clear();
  Rig r(true, true);
  r.unit.feed(ACK);
  r.unit.feed(frame(RES_GET_FAN_STATUS, {50, 50, 0, 0, 0, 0}));
  r.run(2);
  CHECK(r.panel.tx.empty());
  CHECK(r.intake.publishes == 0);
}

int main()
{
  test_transparent_relay();
  test_injection_and_hold();
  test_timeout_retry();
  test_legacy_mode();
  test_fake_unit_reply();
  test_panel_frame();
  test_ack_panel_frames();
  test_wiring_check_not_needed();
  test_wiring_check_disabled();
  test_wiring_check_unit_swapped();
  test_wiring_check_panel_no_output();
  test_wiring_check_nothing_answers();
  test_write_only_answers_panel();
  test_write_only_panel_sets_level();
  test_write_only_home_assistant_sets_level();
  test_write_only_does_not_poll();
  test_write_only_resends_level();
  test_write_only_remembers_across_restart();
  test_write_only_ignores_unit_bytes();
  std::printf("all proxy simulation tests passed\n");
  return 0;
}
