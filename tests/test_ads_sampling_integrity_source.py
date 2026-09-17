from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ADS_SOURCE = ROOT / "src" / "ads122c04.h"
MAIN_SOURCE = ROOT / "src" / "main.cpp"


def test_ads_read_enables_counter_crc_and_validates_six_byte_frame():
    source = ADS_SOURCE.read_text(encoding="utf-8")

    assert "ADS_CONFIG2_DCNT_CRC16" in source
    assert "ADS_CONFIG2_DCNT_CRC16 = 0x60" in source
    assert "adsWriteRegister(cfg.address, 2, ADS_CONFIG2_DCNT_CRC16)" in source
    assert "adsCrc16Ccitt" in source
    assert "uint16_t crc = 0xFFFF" in source
    assert "0x1021" in source
    assert "enum ADSReadStatus" in source
    assert "Wire.endTransmission(false)" in source
    assert "requestFrom(addr, (uint8_t)6)" in source
    assert "ADS_READ_CRC_ERROR" in source
    assert "crcExpected = adsCrc16Ccitt(frame, 4)" in source
    assert "crcReceived = ((uint16_t)frame[4] << 8) | frame[5]" in source


def test_ads_sampling_retries_crc_and_rejects_duplicate_conversion_counter():
    source = MAIN_SOURCE.read_text(encoding="utf-8")

    assert "ADS_READ_CRC_ERROR" in source
    assert "g_spectroCrcErrorCount" in source
    assert "g_spectroDuplicateCount" in source
    assert "g_lastSpectroConversionCounter" in source
    assert "spectroConversionCounter == g_lastSpectroConversionCounter" in source


def test_ads_sampling_drops_isolated_large_transient_without_changing_packet_layout():
    source = MAIN_SOURCE.read_text(encoding="utf-8")

    assert "SPECTRO_TRANSIENT_THRESHOLD_V" in source
    assert "SPECTRO_TRANSIENT_CONFIRM_TOLERANCE_V" in source
    assert "g_spectroTransientDropCount" in source
    assert "acceptSpectroSample" in source
    assert "spectroTransient" in source
    assert "fabsf(voltage - g_spectroAcceptedVoltage) <= SPECTRO_TRANSIENT_THRESHOLD_V" in source
    assert "fabsf(voltage - g_spectroPendingVoltage) <= SPECTRO_TRANSIENT_CONFIRM_TOLERANCE_V" in source
    assert "*spectroTransient = true" in source
    assert "g_spectroTransientDropCount++" in source


def test_ads_health_reports_integrity_counters_and_failed_reads_advance_deadline():
    source = MAIN_SOURCE.read_text(encoding="utf-8")

    assert "CRC_ERROR=%lu" in source
    assert "DUPLICATE=%lu" in source
    assert "TRANSIENT_DROP=%lu" in source
    assert "advanceSpectroDeadline" in source
    assert "advanceSpectroDeadline(now, specInterval)" in source
