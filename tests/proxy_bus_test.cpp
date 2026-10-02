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

int main()
{
  test_idle_gap();
  test_pending_panel_exchange();
  test_own_exchange();
  test_millis_wraparound();
  test_queue();
  test_hold_buffer();
  std::printf("all proxy bus tests passed\n");
  return 0;
}
