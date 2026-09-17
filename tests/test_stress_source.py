from pathlib import Path
import re


SOURCE = Path(__file__).resolve().parents[1] / "src" / "main.cpp"


def test_stress_commands_are_dispatched_before_motor_parser():
    source = SOURCE.read_text(encoding="utf-8")

    stress_idx = source.index('inputBuffer.startsWith("STRESS:")')
    motor_parse_idx = source.index("parseCommand(inputBuffer)")

    assert stress_idx < motor_parse_idx
    assert '"STRESS:STATUS?"' in source
    assert "parseStressCommand" in source


def test_stress_tasks_are_pinned_to_both_cores_and_low_priority():
    source = SOURCE.read_text(encoding="utf-8")

    assert 'xTaskCreatePinnedToCore(TaskStressCore0, "Stress0", 2048, NULL, 0' in source
    assert 'xTaskCreatePinnedToCore(TaskStressCore1, "Stress1", 2048, NULL, 0' in source
    assert "runStressWorkerSlice" in source
    assert "vTaskDelay(1)" in source


def test_low_priority_stress_tasks_are_not_watched_by_task_wdt():
    source = SOURCE.read_text(encoding="utf-8")

    for function_name in ("TaskStressCore0", "TaskStressCore1"):
        body = _function_body(source, function_name)
        assert "registerCurrentTaskWatchdog" not in body
        assert "feedTaskWatchdog" not in body


def test_stress_start_has_motion_guard_and_fixed_responses():
    source = SOURCE.read_text(encoding="utf-8")

    assert "STRESS_ERR:BUSY_MOTION" in source
    assert "STRESS_ERR:DURATION_RANGE" in source
    assert "STRESS_OK:START,mode=%s,duration_s=%lu" in source
    assert "STRESS_STATUS:RUNNING,mode=%s,duration_s=%lu,elapsed_s=%lu,remaining_s=%lu" in source
    assert "STRESS_DONE:mode=%s,duration_s=%lu" in source


def test_full_stress_mode_uses_virtual_load_without_i2c_reads():
    source = SOURCE.read_text(encoding="utf-8")

    assert "STRESS_MODE_FULL" in source
    assert "sendVirtualStressAnglePacket" in source
    assert "sendVirtualStressSpectroPacket" in source
    assert "sendVirtualStressPIDPacket" in source
    assert "updateVirtualStressAngles" in source
    assert "runStressFullCommsLoad" in source

    for function_name in (
        "updateVirtualStressAngles",
        "sendVirtualStressSpectroPacket",
        "sendVirtualStressPIDPacket",
        "runStressFullCommsLoad",
    ):
        body = _function_body(source, function_name)
        assert "readMt6701Angle" not in body
        assert "adsReadData" not in body
        assert "selectTcaChannel" not in body


def test_full_stress_accepts_comma_mode_command_and_default_cpu_compatibility():
    source = SOURCE.read_text(encoding="utf-8")

    assert '"STRESS:START:"' in source
    assert 'params.endsWith(",FULL")' in source
    assert "STRESS_MODE_CPU" in source
    assert "STRESS_MODE_FULL" in source
    assert "STRESS:START:<seconds>,FULL" in source


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
