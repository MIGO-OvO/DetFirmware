"""Execute the firmware PID loop with a clock and sensor, without a board."""
import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

from test_pid_test_scheduling_native import _function


ROOT = Path(__file__).resolve().parents[1]


class PIDMoveTimeoutTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('g++'), 'host g++ required; run under WSL/Linux')
    def test_travel_settling_deadlines_and_stop(self):
        source = (ROOT / 'src/main.cpp').read_text(encoding='utf-8')
        # Relative R, absolute positioning and PID-test rounds all rearm clocks.
        self.assertEqual(source.count('m->beginPIDTiming(millis());'), 3)
        start = source.index('struct MotorState {')
        end = source.index('\n};', start) + len('\n};')
        relative = source[source.index('// delta 必须 >= 0'):]
        arm_start = relative.index('            m->isPIDMode = true;')
        arm_end = relative.index('// 使用全局', arm_start)
        functions = '\n'.join(_function(source, signature) for signature in (
            'float unwrapAngle(float newRaw, float lastRaw, float currentAbs)',
            'void initPIDMove(int motorIndex, float targetAngle, float precision)',
            'void stopPIDMove(int motorIndex)',
            'void runMotorPID()',
        ))
        constants = '\n'.join(re.findall(r'^#define PID_(?:MOVE|SETTLE|PACKET)_.*$', source, re.M))
        harness = (HARNESS.replace('__CONSTANTS__', constants)
                   .replace('__MOTOR_STATE__', source[start:end])
                   .replace('__FUNCTIONS__', functions)
                   .replace('__RELATIVE_ARM__', relative[arm_start:arm_end]))
        with tempfile.TemporaryDirectory() as directory:
            cpp = Path(directory) / 'pid-timeout.cpp'
            cpp.write_text(harness, encoding='utf-8')
            executable = Path(directory) / 'pid-timeout'
            subprocess.run(['g++', '-std=c++11', '-Wall', '-Werror', str(cpp), '-o', str(executable)], check=True)
            subprocess.run([str(executable)], check=True)


HARNESS = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
using byte = uint8_t;
using std::isfinite;
__CONSTANTS__
struct PIDController {
    float Kp, Ki, Kd, outputMin, outputMax, deadband;
    void reset() {}
};
__MOTOR_STATE__
MotorState motors[4] = {};
uint32_t fakeNow = 0;
float rawAngles[4] = {};
float g_pidKp = .14f, g_pidKi = .015f, g_pidKd = .06f;
float g_pidOutputMin = .1f, g_pidOutputMax = 6.0f;
int motorMutex = 0, pdTRUE = 1, portMAX_DELAY = 0;
const byte motorPins[4][2] = {};
int xSemaphoreTake(int, int) { return pdTRUE; }
void xSemaphoreGive(int) {}
uint32_t millis() { return fakeNow; }
float getCachedAngle(int axis) { return rawAngles[axis]; }
void sendPIDDataPacket(int, float, float, float) {}
float computePIDDirect(PIDController&, float error, MotorState&) { return error > 0 ? .1f : -.1f; }
float rpmToInterval(float) { return 100; }
void updateDutyCycleTiming(MotorState&) {}
void setMotorDirection(byte, bool) {}
struct FakeSerial {
    std::string lines;
    template<typename... Args> void printf(const char* format, Args... args) {
        char line[256];
        std::snprintf(line, sizeof(line), format, args...);
        lines += line;
    }
} Serial;
__FUNCTIONS__

void reset(uint32_t now = 0) {
    fakeNow = now;
    for (int i = 0; i < 4; ++i) { motors[i] = MotorState{}; rawAngles[i] = 0; }
    Serial.lines.clear();
}

void relativeMove(int axis, float delta) {
    MotorState* m = &motors[axis];
    m->absTargetAngle = m->absAngle + delta;
    m->lastRawAngle = rawAngles[axis];
    m->absAngleValid = true;
    float precision = .1f;
    bool handled = false;
    // Initialization copied from the production R parser, including all clocks.
    __RELATIVE_ARM__
    assert(handled);
    runMotorPID();
}

void sample(uint32_t now, int axis, float raw) {
    fakeNow = now;
    rawAngles[axis] = raw;
    runMotorPID();
}

bool logged(const char* text) { return Serial.lines.find(text) != std::string::npos; }

int main() {
    // Slow multi-turn travel crosses raw zero twice. Wrapped equality at the
    // start or after one turn must not start the near-target clock.
    reset();
    rawAngles[3] = 350;
    relativeMove(3, 720);
    for (uint32_t sec = 1; sec <= 400; ++sec) {
        sample(sec * 1000, 3, std::fmod(350.0f + sec * 1.8f, 360.0f));
        assert(motors[3].isPIDMode);
        if (sec < 395) assert(!logged("PID_SETTLE:A"));
    }
    assert(logged("PID_SETTLE:A"));
    for (int n = 1; n <= 10; ++n) sample(400000 + n * 20, 3, 350);
    assert(logged("PID_DONE:A") && !logged("PID_TIMEOUT:A"));
    assert(!motors[3].enabled);

    // First entry at exactly 10 degrees starts 180 s. Crossing 1 degree,
    // overshooting, and leaving/re-entering 10 degrees cannot renew it.
    reset();
    relativeMove(3, 20);
    sample(100000, 3, 10);
    sample(110000, 3, 19.5f);
    sample(150000, 3, 21);
    sample(200000, 3, 9);
    sample(279999, 3, 19.7f);
    assert(motors[3].isPIDMode && !logged("PID_TIMEOUT:A"));
    sample(280000, 3, 19.7f);
    assert(logged("PID_TIMEOUT:A") && logged("phase=settling"));
    assert(!motors[3].enabled && !motors[3].isPIDMode);

    // A separate ten-minute ceiling applies even if it never gets close,
    // or if it only enters the settling region just before the ceiling.
    for (bool approach : {false, true}) {
        reset(); relativeMove(0, 20);
        sample(599000, 0, approach ? 10 : 0);
        sample(599999, 0, approach ? 10 : 0);
        assert(motors[0].isPIDMode);
        sample(600000, 0, approach ? 10 : 0);
        assert(logged("phase=total") && !motors[0].isPIDMode);
    }

    // Small/reverse moves start settling at time zero; axis completion is
    // independent and must not reset another motor's deadline.
    reset(); relativeMove(3, -3.6f); relativeMove(1, 0);
    for (int n = 1; n <= 10; ++n) sample(n * 20, 1, 0);
    assert(logged("PID_DONE:Y") && motors[3].isPIDMode);
    sample(179999, 3, 0);
    assert(motors[3].isPIDMode);
    sample(180000, 3, 0);
    assert(!motors[3].isPIDMode && logged("phase=settling"));

    // STOP prevents later ticks from re-enabling output; a new command gets
    // its own clocks, including the absolute-position entry point.
    reset(); relativeMove(0, 20);
    sample(1000, 0, 10); stopPIDMove(0);
    sample(500000, 0, 10);
    assert(!motors[0].enabled && !logged("PID_TIMEOUT"));
    initPIDMove(0, 30, .1f);
    sample(600000, 0, 10);
    assert(motors[0].isPIDMode);
    sample(601000, 0, 20);
    sample(780999, 0, 20);
    assert(motors[0].isPIDMode);
    sample(781000, 0, 20);
    assert(!motors[0].isPIDMode);

    // Invalid sensors still stop promptly, including non-finite feedback.
    for (float invalid : {-1.0f, NAN}) {
        reset(); relativeMove(0, 90);
        for (int n = 1; n <= 11; ++n) sample(n * 20, 0, invalid);
        assert(logged("PID_FAIL:X=SENSOR_ERR") && !motors[0].enabled);
    }

    // Both clocks survive the ESP32 uint32 millisecond wrap.
    const uint32_t start = 0xffffff00U;
    reset(start); relativeMove(0, 20);
    sample(start + 1000U, 0, 10);
    sample(start + 180999U, 0, 10);
    assert(motors[0].isPIDMode);
    sample(start + 181000U, 0, 10);
    assert(logged("phase=settling") && !motors[0].isPIDMode);
    reset(start); relativeMove(0, 20);
    sample(start + 599999U, 0, 0);
    assert(motors[0].isPIDMode);
    sample(start + 600000U, 0, 0);
    assert(logged("phase=total") && !motors[0].isPIDMode);
}
'''


if __name__ == '__main__':
    unittest.main()
