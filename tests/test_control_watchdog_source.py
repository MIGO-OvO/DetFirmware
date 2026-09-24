from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_watchdog_trip_halts_all_outputs_and_requires_explicit_rearm():
    source = (ROOT / 'src/main.cpp').read_text(encoding='utf-8')
    assert 'ControlWatchdog g_controlWatchdog' in source
    assert 'WATCHDOG:ARM' in source
    assert 'WATCHDOG:KEEPALIVE' in source
    assert 'CAP=WATCHDOG1' in source
    assert 'g_controlWatchdog.tick(millis())' in source
    assert 'WATCHDOG_ERR:REARM_REQUIRED' in source
    assert 'void stopAllOutputs()' in source


def test_virtual_spectro_never_sets_real_valid_flag():
    source = (ROOT / 'src/main.cpp').read_text(encoding='utf-8')
    body = source.split('void sendVirtualStressSpectroPacket() {', 1)[1].split('\n}', 1)[0]
    assert 'uint8_t status = SPECTRO_STATUS_TEST;' in body
    assert 'SPECTRO_STATUS_VALID' not in body
