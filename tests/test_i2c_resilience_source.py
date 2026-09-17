from pathlib import Path
import re


SOURCE = Path(__file__).resolve().parents[1] / "src" / "main.cpp"
I2C_MUX = Path(__file__).resolve().parents[1] / "src" / "i2c_mux.h"


def test_i2c_bus_uses_short_timeout_and_recovery_path():
    source = SOURCE.read_text(encoding="utf-8")

    assert "#define I2C_TRANSACTION_TIMEOUT_MS" in source
    assert "Wire.setTimeOut(I2C_TRANSACTION_TIMEOUT_MS)" in source
    assert "recoverI2CBus()" in source
    assert "recordI2CBusFault" in source
    assert "I2C_RECOVER:" in source


def test_sensor_task_feeds_watchdog_per_angle_channel_without_recovering_on_device_failure():
    source = SOURCE.read_text(encoding="utf-8")
    body = _function_body(source, "TaskSensors")
    read_idx = body.index("readMt6701AngleWithStatus(g_angleChannels[i], &angleStatus)")
    feed_idx = body.index("feedTaskWatchdog()", read_idx)

    assert feed_idx > read_idx
    assert "recordI2CBusFault(angleStatus == I2C_READ_MUX_ERROR)" in body
    assert "recordI2CReadResult(angleOk)" not in body


def test_ads_sampling_gets_i2c_priority_over_angle_refresh():
    source = SOURCE.read_text(encoding="utf-8")
    body = _function_body(source, "TaskSensors")

    assert "anglesPerCycle" in body
    assert "SENSOR_TASK_DELAY_SPECTRO_MS" in source
    assert "I2C_SPECTRO_TAKE_TIMEOUT_MS" in source
    assert "g_spectroRunning" in body


def test_ads_deadline_retries_mutex_timeout_without_consuming_period():
    source = SOURCE.read_text(encoding="utf-8")
    body = _function_body(source, "TaskComms")

    assert "(int32_t)(now - g_nextSpectroDueMs) >= 0" in body
    assert "g_spectroMutexTimeoutCount++" in body
    assert "advanceSpectroDeadline(now, specInterval)" in body
    read_index = body.index("ADSReadStatus spectroReadStatus")
    give_index = body.index("xSemaphoreGive(i2cMutex)", read_index)
    advance_index = body.index("advanceSpectroDeadline(now, specInterval)", read_index)
    publish_index = body.index("if (spectroPublishOk)", read_index)
    timeout_index = body.index("g_spectroMutexTimeoutCount++", read_index)
    assert give_index < advance_index < publish_index < timeout_index
    assert give_index < body.index("Serial.write", read_index)


def test_angle_freshness_tracks_each_channel_and_reports_oldest_age():
    source = SOURCE.read_text(encoding="utf-8")
    sensor_body = _function_body(source, "TaskSensors")
    health_body = _function_body(source, "sendHealthPacket")

    assert "g_angleTimestampMs[i] = millis()" in sensor_body
    assert "oldestAngleAgeMs" in health_body
    assert "ANGLE_AGE_CH_MS:" in health_body
    assert "ADS_HEALTH:" in health_body


def test_angle_reader_distinguishes_mux_failure_from_device_read_failure():
    source = I2C_MUX.read_text(encoding="utf-8")

    assert "enum I2CReadStatus" in source
    assert "I2C_READ_MUX_ERROR" in source
    assert "I2C_READ_DEVICE_ERROR" in source
    assert "readMt6701AngleWithStatus" in source


def _function_body(source: str, name: str) -> str:
    match = re.search(rf"void {name}\([^)]*\) \{{", source)
    assert match is not None, f"{name} not found"
    start = match.end()
    depth = 1
    pos = start
    while pos < len(source) and depth:
        if source[pos] == "{":
            depth += 1
        elif source[pos] == "}":
            depth -= 1
        pos += 1
    assert depth == 0, f"{name} body not closed"
    return source[start : pos - 1]
