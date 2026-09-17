#pragma once
#include <stdint.h>

// Deliberately latched: a late heartbeat cannot restart an interrupted motion.
class ControlWatchdog {
public:
    void arm(uint32_t now) { last_ms = now; armed = true; tripped = false; }
    void keepalive(uint32_t now) { if (allows_control()) { last_ms = now; } }
    bool allows_control() const { return armed && !tripped; }
    bool tick(uint32_t now) {
        if (allows_control() && uint32_t(now - last_ms) > 3000U) {
            tripped = true;
            return true;
        }
        return false;
    }
private:
    uint32_t last_ms = 0;
    bool armed = false;
    bool tripped = false;
};
