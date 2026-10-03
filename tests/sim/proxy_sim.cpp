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

struct FakeUart : uart::UARTComponent
{
  std::deque<uint8_t> rx; // bytes the component will read
  Bytes tx;               // bytes the component wrote
  void write_array(const uint8_t *d, size_t n) override { tx.insert(tx.end(), d, d + n); }
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
  explicit Rig(bool proxy)
  {
    c.set_uart_component(&unit);
    if (proxy)
    {
      c.set_proxy_uart(&panel);
    }
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

int main()
{
  test_transparent_relay();
  test_injection_and_hold();
  test_timeout_retry();
  test_legacy_mode();
  test_fake_unit_reply();
  test_panel_frame();
  std::printf("all proxy simulation tests passed\n");
  return 0;
}
