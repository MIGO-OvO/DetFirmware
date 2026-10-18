"""Host-native checks of actual DetFirmware ADSSTART / ADSSTOP code.

The Arduino transport and scheduler are replaced by a minimal deterministic
I2C model. Source function bodies and command bodies are extracted verbatim.
This proves the logical race/ACK defects, not their incidence on an ESP32.
"""
import subprocess
import shutil
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FW = ROOT / 'src'


def brace_block(text, start):
    opening = text.index('{', start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[opening + 1:end - 1], end


def extract_function(text, signature):
    start = text.index(signature)
    _, end = brace_block(text, start)
    return text[start:end]


@unittest.skipUnless(shutil.which('g++'), 'host g++ required; run native checks in WSL')
class AdsBusContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        header = (FW / 'ads122c04.h').read_text(encoding='utf-8')
        main = (FW / 'main.cpp').read_text(encoding='utf-8')
        # TaskSensors is always created and takes the mutex before switching
        # channel. These production locations justify the modeled competitor.
        assert 'xTaskCreatePinnedToCore(TaskSensors' in main
        sensor, _ = brace_block(main, main.index('void TaskSensors(void *pvParameters) {'))
        assert 'xSemaphoreTake(i2cMutex' in sensor and 'readMt6701AngleWithStatus' in sensor
        start_body, _ = brace_block(main, main.index('else if (inputBuffer == "ADSSTART")'))
        stop_body, _ = brace_block(main, main.index('else if (inputBuffer == "ADSSTOP")'))
        declaration = header[header.index('#define ADS_CMD_RESET'):header.index('// 默认配置')]
        helpers = '\n'.join(extract_function(header, signature) for signature in (
            'inline void adsConfigDefault', 'inline uint8_t adsRateToDRBits',
            'inline uint8_t adsGainToBits', 'inline bool adsInitAndStart', 'inline bool adsStop'))
        source = r'''
#include <cstdint>
#include <cstdio>
#include <string>
using std::string;
static int channel = -1;
static bool bus_locked = false, fail_mutex = false;
static int i2cMutex = 0;
#define pdTRUE 1
#define pdMS_TO_TICKS(x) (x)
#define I2C_SPECTRO_TAKE_TIMEOUT_MS 15
static int xSemaphoreTake(int, int) { if (fail_mutex || bus_locked) return 0; bus_locked = true; return pdTRUE; }
static void xSemaphoreGive(int) { bus_locked = false; }
static bool switch_at_delay = false;
static bool fail_powerdown = false;
static bool mux_failure = false;
static bool converter_running = false;
static string response;
struct SerialFake { void println(const char* value) { response = value; } } Serial;
static bool selectTcaChannel(uint8_t target) {
    if (mux_failure) return false;
    channel = target;
    return true;
}
static void invalidateTcaCache() {}
static void delay(int) {
    // TaskSensors holds its mutex while selecting an angle channel.
    // ADSSTART does not hold that mutex, so this schedule is legal.
    if (switch_at_delay && !bus_locked) selectTcaChannel(0);
}
static uint32_t millis() { return 1000; }
static void resetSpectroSamplingIntegrity() {}
static bool adsSendCommand(uint8_t, uint8_t command) {
    if (channel != 2) return false;
    if (command == 0x02 && fail_powerdown) return false;
    if (command == 0x08) converter_running = true;
    if (command == 0x02 || command == 0x06) converter_running = false;
    return true;
}
static bool adsWriteRegister(uint8_t, uint8_t, uint8_t) { return channel == 2; }
'''
        source += declaration + '\n' + helpers
        source += '\nADSConfig g_adsConfig; bool g_spectroRunning = false; bool g_adsStopConfirmed = false; uint32_t g_nextSpectroDueMs = 0; uint8_t g_lastSpectroStatus = 0;\n'
        source += 'void issue_start() {' + start_body + '}\n'
        source += 'void issue_stop() {' + stop_body + '}\n'
        source += r'''
int main(int argc, char** argv) {
    adsConfigDefault(g_adsConfig);
    g_adsConfig.enabled = true;
    string test = argc > 1 ? argv[1] : "baseline";
    switch_at_delay = test == "interleave";
    fail_mutex = test == "start_mutex";
    issue_start();
    if (test == "repeat_stop" || test == "stop_recovery") {
        fail_powerdown = test == "stop_recovery";
        issue_stop();
        if (fail_powerdown && g_adsStopConfirmed) return 2;
        fail_powerdown = false;
        issue_stop();
        return response == "ADS_OK:STOP" && g_adsStopConfirmed && !converter_running && !bus_locked ? 0 : 1;
    }
    if (test == "powerdown_failure" || test == "mux_failure" || test == "stop_mutex") {
        fail_powerdown = test == "powerdown_failure";
        mux_failure = test == "mux_failure";
        fail_mutex = test == "stop_mutex";
        issue_stop();
    }
    std::printf("%s response=%s host_polling=%d converter_running=%d channel=%d\n",
                test.c_str(), response.c_str(), g_spectroRunning, converter_running, channel);
    if (bus_locked) return 3;
    if (test == "start_mutex") return response == "ADS_ERR:MUTEX_TIMEOUT" && !g_spectroRunning ? 0 : 1;
    if (test == "baseline" || test == "interleave")
        return (response == "ADS_OK:START" && g_spectroRunning) ? 0 : 1;
    return (response != "ADS_OK:STOP" && !g_adsStopConfirmed && !g_spectroRunning && converter_running) ? 0 : 1;
}
'''
        cls.directory = tempfile.TemporaryDirectory()
        directory = Path(cls.directory.name)
        path = directory / 'actual_ads_functions.cpp'
        path.write_text(source, encoding='utf-8')
        cls.executable = directory / 'actual_ads_functions'
        subprocess.run(['g++', '-std=c++17', '-O0', str(path), '-o', str(cls.executable)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def check(self, scenario):
        result = subprocess.run([str(self.executable), scenario], capture_output=True, text=True)
        print(result.stdout.strip())
        self.assertEqual(result.returncode, 0, result.stdout.strip())

    def test_start_without_competing_angle_read_succeeds(self):
        self.check('baseline')

    def test_start_survives_a_legal_task_sensors_interleaving(self):
        self.check('interleave')

    def test_failed_powerdown_is_not_reported_as_applied_stop(self):
        self.check('powerdown_failure')

    def test_start_mutex_timeout_never_starts(self):
        self.check('start_mutex')

    def test_stop_mutex_timeout_is_unknown(self):
        self.check('stop_mutex')

    def test_failed_channel_selection_is_not_reported_as_applied_stop(self):
        self.check('mux_failure')

    def test_repeated_stop_remains_confirmed(self):
        self.check('repeat_stop')

    def test_failed_stop_requires_successful_retry(self):
        self.check('stop_recovery')


if __name__ == '__main__':
    unittest.main(verbosity=2)
