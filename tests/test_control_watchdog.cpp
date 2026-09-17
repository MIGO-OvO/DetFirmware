#include "../src/control_watchdog.h"
#include <cassert>

int main() {
    ControlWatchdog watchdog;
    assert(!watchdog.allows_control());
    watchdog.arm(100);
    assert(watchdog.allows_control());
    assert(!watchdog.tick(3100));
    assert(watchdog.tick(3101));
    watchdog.keepalive(3102);
    assert(!watchdog.allows_control());
    assert(!watchdog.tick(4000));
    watchdog.arm(0xfffffff0U);
    watchdog.keepalive(100);
    assert(!watchdog.tick(3000));
    assert(watchdog.tick(3101));
    watchdog.arm(0xfffffff0U);
    assert(!watchdog.tick(100));
    assert(watchdog.tick(3000));
}
