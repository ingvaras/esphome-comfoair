// Host-side tests for the ComfoSense proxy bus arbitration helpers.
// Build and run: g++ -std=c++17 -Wall -Wextra -I components/comfoair tests/proxy_bus_test.cpp -o /tmp/proxy_bus_test && /tmp/proxy_bus_test
#include "proxy_bus.h"

#include <cstdio>
#include <cstdlib>

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

static void test_idle_gap()
{
  ProxyBusArbiter a;
  CHECK(a.can_inject(0)); // nothing ever seen
  a.note_panel_byte(1000);
  CHECK(!a.can_inject(1010));
  CHECK(a.can_inject(1000 + PROXY_IDLE_GAP_MS));
  a.note_unit_byte(1100);
  CHECK(!a.can_inject(1100 + PROXY_IDLE_GAP_MS - 1));
  CHECK(a.can_inject(1100 + PROXY_IDLE_GAP_MS));
}

static void test_pending_panel_exchange()
{
  ProxyBusArbiter a;
  a.note_panel_byte(1000);
  a.panel_frame_complete(1000);
  // Quiet for longer than the idle gap, but the unit has not answered yet.
  CHECK(!a.can_inject(1100));
  a.note_unit_byte(1110);
  a.unit_frame_complete();
  CHECK(!a.can_inject(1120));                      // panel is still ACKing
  CHECK(a.can_inject(1110 + PROXY_IDLE_GAP_MS)); // exchange complete and quiet
  // An answer that never arrives must not block forever.
  a.panel_frame_complete(2000);
  CHECK(!a.can_inject(2000 + PROXY_PANEL_EXCHANGE_TIMEOUT_MS - 1));
  CHECK(a.can_inject(2000 + PROXY_PANEL_EXCHANGE_TIMEOUT_MS));
}

static void test_own_exchange()
{
  ProxyBusArbiter a;
  a.begin_own_exchange(5000);
  CHECK(a.own_exchange());
  CHECK(!a.can_inject(9000));
  CHECK(!a.own_exchange_timed_out(5000 + PROXY_OWN_EXCHANGE_TIMEOUT_MS - 1));
  CHECK(a.own_exchange_timed_out(5000 + PROXY_OWN_EXCHANGE_TIMEOUT_MS));
  a.end_own_exchange();
  CHECK(!a.own_exchange_timed_out(99999));
  CHECK(a.can_inject(9000));
}

static void test_millis_wraparound()
{
  ProxyBusArbiter a;
  a.note_panel_byte(0xFFFFFFF0u);
  CHECK(!a.can_inject(0x00000005u)); // 21ms later across the wrap
  CHECK(a.can_inject(0x00000020u));
}

static void test_queue()
{
  ProxyTxQueue q;
  uint8_t f1[] = {0x07, 0xF0, 0x00, 0x99, 0x01, 0x01, 0x07, 0x0F};
  uint8_t f1b[] = {0x07, 0xF0, 0x00, 0x99, 0x01, 0x03, 0x07, 0x0F};
  uint8_t g[] = {0x07, 0xF0, 0x00, 0x0B, 0x00, 0xB8, 0x07, 0x0F};
  ProxyTxQueue::Frame out;

  CHECK(q.empty());
  CHECK(q.push(f1, sizeof(f1), true));
  CHECK(q.push(g, sizeof(g), false));
  CHECK(q.push(g, sizeof(g), false)); // identical frame is dropped
  CHECK(q.push(f1b, sizeof(f1b), true)); // latest set wins
  CHECK(q.pop(&out));
  CHECK(out.command() == 0x99 && out.data[5] == 0x03);
  CHECK(q.pop(&out));
  CHECK(out.command() == 0x0B);
  CHECK(!q.pop(&out));

  // Capacity: distinct commands until full, then rejected.
  for (uint8_t i = 0; i < ProxyTxQueue::DEPTH; i++)
  {
    uint8_t f[] = {0x07, 0xF0, 0x00, (uint8_t)(0x10 + i), 0x00, 0x00, 0x07, 0x0F};
    CHECK(q.push(f, sizeof(f), false));
  }
  uint8_t extra[] = {0x07, 0xF0, 0x00, 0x50, 0x00, 0x00, 0x07, 0x0F};
  CHECK(!q.push(extra, sizeof(extra), false));
  CHECK(q.pop(&out) && out.command() == 0x10); // FIFO order
  CHECK(q.push(extra, sizeof(extra), false)); // slot freed
  CHECK(!q.push(extra, 2, false));            // too short
}

static void test_hold_buffer()
{
  ProxyHoldBuffer h;
  CHECK(h.empty());
  for (uint16_t i = 0; i < ProxyHoldBuffer::CAPACITY; i++)
  {
    CHECK(h.push((uint8_t)i));
  }
  CHECK(!h.push(0xFF));
  CHECK(h.length() == ProxyHoldBuffer::CAPACITY && h.data()[5] == 5);
  h.clear();
  CHECK(h.empty());
}

static void test_wiring_classify_pin()
{
  const uint8_t all = WiringCheck::SAMPLES_PER_PULL;
  CHECK(WiringCheck::classify_pin(all, all) == PinDrive::DRIVEN_HIGH);
  CHECK(WiringCheck::classify_pin(all, 0) == PinDrive::NOT_DRIVEN);
  CHECK(WiringCheck::classify_pin(0, 0) == PinDrive::DRIVEN_LOW);
  // Traffic on an adapter output makes single reads low.
  CHECK(WiringCheck::classify_pin(all - 2, all - 3) == PinDrive::DRIVEN_HIGH);
}

static void test_wiring_adapter_verdict()
{
  WiringCheck w;
  w.set_drive(0, PinDrive::DRIVEN_HIGH);
  w.set_drive(1, PinDrive::NOT_DRIVEN);
  CHECK(w.adapter_wiring(0, 1) == AdapterWiring::MATCHES_CONFIG);
  CHECK(w.adapter_wiring(1, 0) == AdapterWiring::SWAPPED);
  w.set_drive(0, PinDrive::NOT_DRIVEN);
  CHECK(w.adapter_wiring(0, 1) == AdapterWiring::NO_OUTPUT);
  w.set_drive(0, PinDrive::DRIVEN_HIGH);
  w.set_drive(1, PinDrive::DRIVEN_HIGH);
  CHECK(w.adapter_wiring(0, 1) == AdapterWiring::BOTH_DRIVEN);
  w.set_drive(1, PinDrive::DRIVEN_LOW);
  CHECK(w.adapter_wiring(0, 1) == AdapterWiring::HELD_LOW);
}

static void test_wiring_plan()
{
  // Normal case: transmit on undriven pins only, listen on adapter outputs.
  WiringCheck w;
  w.set_drive(0, PinDrive::DRIVEN_HIGH);
  w.set_drive(1, PinDrive::NOT_DRIVEN);
  w.set_drive(2, PinDrive::NOT_DRIVEN);
  w.set_drive(3, PinDrive::DRIVEN_HIGH);
  w.plan_tests();
  CHECK(w.test_count() == 4);
  for (uint8_t test = 0; test < w.test_count(); test++)
  {
    CHECK(w.drive(w.test_tx(test)) == PinDrive::NOT_DRIVEN);
    CHECK(w.drive(w.test_rx(test)) == PinDrive::DRIVEN_HIGH);
  }

  // Both pins of an adapter high: both are tried, in both directions.
  WiringCheck both;
  both.set_drive(0, PinDrive::DRIVEN_HIGH);
  both.set_drive(1, PinDrive::DRIVEN_HIGH);
  both.set_drive(2, PinDrive::NOT_DRIVEN);
  both.set_drive(3, PinDrive::NOT_DRIVEN);
  both.plan_tests();
  CHECK(both.test_count() == 6); // 0->1, 1->0, and each of 2 and 3 to 0 and 1

  // No adapter output anywhere: nothing can be tested.
  WiringCheck none;
  for (uint8_t pin = 0; pin < WiringCheck::PIN_COUNT; pin++)
  {
    none.set_drive(pin, PinDrive::NOT_DRIVEN);
  }
  none.plan_tests();
  CHECK(none.test_count() == 0);

  // Every pin driven: the plan stays within its capacity.
  WiringCheck all;
  for (uint8_t pin = 0; pin < WiringCheck::PIN_COUNT; pin++)
  {
    all.set_drive(pin, PinDrive::DRIVEN_HIGH);
  }
  all.plan_tests();
  CHECK(all.test_count() == WiringCheck::MAX_TESTS);
}

static void test_wiring_replies()
{
  WiringCheck w;
  const uint8_t unit_reply[] = {0x07, 0xF3, 0x07, 0xF0, 0x00, 0x0C, 0x06};
  w.note_poll_reply(0, unit_reply, sizeof(unit_reply), 0x0C);
  CHECK(w.unit_answered(0) && w.acked(0) && w.got_bytes(0));

  const uint8_t ack_only[] = {0x07, 0xF3};
  w.note_poll_reply(1, ack_only, sizeof(ack_only), 0x0C);
  CHECK(!w.unit_answered(1) && w.acked(1) && w.got_bytes(1));

  const uint8_t junk[] = {0x12, 0x34, 0x07};
  w.note_poll_reply(2, junk, sizeof(junk), 0x0C);
  CHECK(!w.unit_answered(2) && !w.acked(2) && w.got_bytes(2));

  w.note_poll_reply(3, nullptr, 0, 0x0C);
  CHECK(!w.unit_answered(3) && !w.acked(3) && !w.got_bytes(3));

  // A later reply never clears an earlier one.
  w.note_reply(1, nullptr, 0);
  CHECK(w.acked(1));
}

static void test_wiring_heard()
{
  WiringCheck w;
  for (uint16_t i = 0; i < WiringCheck::HEARD_SAMPLE_SIZE + 10U; i++)
  {
    w.note_heard(2, (uint8_t)i);
  }
  CHECK(w.heard(2) == WiringCheck::HEARD_SAMPLE_SIZE + 10U);
  CHECK(w.sample_length(2) == WiringCheck::HEARD_SAMPLE_SIZE);
  CHECK(w.sample(2)[3] == 3);
  CHECK(w.heard(0) == 0 && w.sample_length(0) == 0);
}

int main()
{
  test_idle_gap();
  test_pending_panel_exchange();
  test_own_exchange();
  test_millis_wraparound();
  test_queue();
  test_hold_buffer();
  test_wiring_classify_pin();
  test_wiring_adapter_verdict();
  test_wiring_plan();
  test_wiring_replies();
  test_wiring_heard();
  std::printf("all proxy bus tests passed\n");
  return 0;
}
