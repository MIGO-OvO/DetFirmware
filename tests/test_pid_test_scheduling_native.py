"""Run the production PID-test/watchdog scheduling functions with fake hardware."""

import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _function(source, signature):
    start = source.index(signature + ' {')
    opening = source.index('{', start)
    depth = 0
    for end in range(opening, len(source)):
        if source[end] == '{':
            depth += 1
        elif source[end] == '}':
            depth -= 1
            if depth == 0:
                return source[start:end + 1]
    raise AssertionError('unterminated production function: ' + signature)


class PIDTestSchedulingTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('g++'), 'host g++ required; run under WSL/Linux')
    def test_round_gap_keeps_watchdog_and_stop_dispatch_live(self):
        source = (ROOT / 'src/main.cpp').read_text(encoding='utf-8')
        struct_start = source.index('struct PIDTestData {')
        struct_end = source.index('\n};', struct_start) + len('\n};')
        functions = '\n\n'.join(_function(source, signature) for signature in (
            'void stopPIDMove(int motorIndex)',
            'void stopPIDTest()',
            'void stopAllOutputs()',
            'void checkControlWatchdog()',
            'void runPIDTestSampling()',
        ))
        harness = HARNESS.replace('__WATCHDOG_HEADER__', (ROOT / 'src/control_watchdog.h').as_posix())
        harness = harness.replace('__PID_TEST_STRUCT__', source[struct_start:struct_end])
        harness = harness.replace('__PRODUCTION_FUNCTIONS__', functions)
        with tempfile.TemporaryDirectory() as directory:
            cpp = Path(directory) / 'pid-test-scheduling.cpp'
            cpp.write_text(harness, encoding='utf-8')
            executable = Path(directory) / 'pid-test-scheduling'
            subprocess.run(['g++', '-std=c++11', '-Wall', '-Werror', str(cpp), '-o', str(executable)], check=True)
            subprocess.run([str(executable)], check=True)


HARNESS = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include "__WATCHDOG_HEADER__"
#define PID_TEST_MAX_SAMPLES 200
#define PID_TEST_SAMPLE_INTERVAL 20
__PID_TEST_STRUCT__
PIDTestData pidTest = {};
struct MotorState {
    bool isPIDMode = false;
    bool enabled = false;
    int stepInterval = 0;
    bool isContinuous = false;
    bool waitingToSend = false;
    bool justFinished = false;
    float lastOutputRPM = 0;
} motors[4];
ControlWatchdog g_controlWatchdog;
uint32_t fakeNow = 0;
bool pumpEnabled = false;
int starts = 0;
int finishes = 0;
int motorMutex = 1;
const int pdTRUE = 1;
const int portMAX_DELAY = 0;
int xSemaphoreTake(int, int) { return pdTRUE; }
void xSemaphoreGive(int) {}
struct FakeSerial { void println(const char *) {} } Serial;
unsigned long millis() { return fakeNow; }
void delay(unsigned long duration) { fakeNow += duration; }
float getCachedAngle(int) { return 90.0f; }
float normalizeAngleError(float target, float current) { return target - current; }
void stopCalibration() {}
void stopPIDMove(int motorIndex);
void stopAllPIDMoves() { for (int i = 0; i < 4; ++i) stopPIDMove(i); }
void setPumpEnabled(bool enabled) { pumpEnabled = enabled; }
void finishTestRun() { ++finishes; }
void startNextTestRun() { ++starts; motors[0].isPIDMode = true; }
__PRODUCTION_FUNCTIONS__

void reset(uint32_t now) {
    pidTest = PIDTestData{};
    for (auto &motor : motors) motor = MotorState{};
    pidTest.active = true;
    pidTest.totalRuns = 3;
    fakeNow = now;
    pumpEnabled = true;
    starts = finishes = 0;
    g_controlWatchdog = ControlWatchdog{};
    g_controlWatchdog.arm(now);
    motors[1].enabled = true;
    motors[1].stepInterval = 10;
    motors[1].waitingToSend = true;
    motors[1].justFinished = true;
}

// This is the control portion of TaskComms: watchdog before sampling. The
// extracted production sampler must return promptly so command/stop dispatch
// can run on the next iteration; a fake delay reproduces the old 2 s blind spot.
void commsControlTurn() {
    checkControlWatchdog();
    if (pidTest.active) runPIDTestSampling();
}

int main() {
    reset(0);
    fakeNow = 2900;
    commsControlTurn();
    assert(fakeNow == 2900);  // must not delay(2000) inside the comms task
    assert(finishes == 1 && starts == 0 && pumpEnabled);
    fakeNow = 3001;
    commsControlTurn();
    assert(!pumpEnabled && !pidTest.active && !g_controlWatchdog.allows_control());
    assert(!motors[1].enabled && !motors[1].waitingToSend && !motors[1].justFinished);
    g_controlWatchdog.keepalive(4000);
    fakeNow = 6000;
    commsControlTurn();
    assert(starts == 0);  // timeout cannot launch the queued next run

    reset(100);
    commsControlTurn();
    fakeNow = 2099;
    g_controlWatchdog.keepalive(fakeNow);
    commsControlTurn();
    assert(finishes == 1 && starts == 0);
    fakeNow = 2100;
    commsControlTurn();
    assert(finishes == 1 && starts == 1 && pidTest.currentRun == 1);

    reset(100);
    commsControlTurn();
    fakeNow = 200;
    stopPIDTest();  // explicit PIDTESTSTOP remains dispatchable during the gap
    fakeNow = 2200;
    commsControlTurn();
    assert(!pidTest.active && starts == 0);

    reset(100);
    commsControlTurn();
    stopAllOutputs();  // STOPALL cancels a queued round and the pump immediately
    fakeNow = 2200;
    commsControlTurn();
    assert(!pumpEnabled && !pidTest.active && starts == 0);

    reset(0xffffff00U);
    commsControlTurn();
    fakeNow = uint32_t(0xffffff00U + 1999U);
    g_controlWatchdog.keepalive(fakeNow);
    commsControlTurn();
    assert(starts == 0);
    ++fakeNow;
    commsControlTurn();
    assert(starts == 1);  // round-gap timing survives uint32 clock wrap
}
'''


if __name__ == '__main__':
    unittest.main()
