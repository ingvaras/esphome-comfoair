# Tests

Host-side tests for the ComfoSense proxy mode. They need only a C++17 compiler.

```
# Unit tests for the bus arbitration helpers (proxy_bus.h)
g++ -std=c++17 -Wall -Wextra -I components/comfoair tests/proxy_bus_test.cpp -o /tmp/proxy_bus_test && /tmp/proxy_bus_test

# Simulation: the real comfoair.h against stubbed ESPHome headers and fake UARTs
g++ -std=c++17 -Wall -Wextra -Wno-unused-parameter -I tests/sim -I components/comfoair tests/sim/proxy_sim.cpp -o /tmp/proxy_sim && /tmp/proxy_sim
```

`tests/sim/esphome.h` is a minimal stand-in for the ESPHome APIs that
`comfoair.h` uses; extend it if the component starts using more of them.
